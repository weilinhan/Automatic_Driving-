/**
 * @file controller_node.cpp
 * @brief implementation of the controller
 * @author PSAF
 * @date 2022-06-01
 */
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <fstream>

#include "psaf_controller/controller_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "psaf_shared/image_utils.hpp"

ControllerConfig ControllerConfig::fromParameters(rclcpp::Node & node)
{
  ControllerConfig c;

  c.updateFrequency = node.declare_parameter<double>("update_frequency", c.updateFrequency);
  c.debugOutput = node.declare_parameter<bool>("debug_output", c.debugOutput);
  c.ignoreState = node.declare_parameter<bool>("ignore_state", c.ignoreState);
  c.useRegulation = node.declare_parameter<bool>("use_regulation", c.useRegulation);
  c.targetSpeed = node.declare_parameter<int>("target_speed", c.targetSpeed);
  c.showImages = node.declare_parameter<bool>("show_images", c.showImages);
  c.saveImages = node.declare_parameter<bool>("save_images", c.saveImages);
  c.enableSigns = node.declare_parameter<bool>("enable_signs", c.enableSigns);
  c.signTimeout = node.declare_parameter<int>("sign_timeout", c.signTimeout);

  auto ldMultiplier = node.declare_parameter<double>("looakhead_distance", c.ldMultiplier);
  c.lookaheadDistance = ldMultiplier * c.wheelBase;

  return c;
}

std::ostream & operator<<(std::ostream & os, const ControllerConfig & cfg)
{
  os << "==== Configuration ====\n";
  os << "- updateFrequency: " << cfg.updateFrequency << "\n";
  os << "- debugOutput: " << std::boolalpha << cfg.debugOutput << "\n";
  os << "- ignoreState: " << std::boolalpha << cfg.ignoreState << "\n";
  os << "- useRegulation: " << std::boolalpha << cfg.useRegulation << "\n";
  os << "- targetSpeed: " << cfg.targetSpeed << "\n";
  os << "- wheelBase: " << cfg.wheelBase << "\n";
  os << "- lookaheadDistance: " << cfg.lookaheadDistance << "\n";
  os << "- showImages: " << std::boolalpha << cfg.showImages << "\n";
  os << "- saveImages: " << std::boolalpha << cfg.saveImages << "\n";
  os << "- enableSigns: " << std::boolalpha << cfg.enableSigns << "\n";
  os << "- signTimeout: " << cfg.signTimeout << "\n";
  os << "- pidKp: " << cfg.pidKp << "\n";
  os << "- pidKi: " << cfg.pidKi << "\n";
  os << "- pidKd: " << cfg.pidKd << "\n";
  os << "==== End of Cfg =======";

  return os;
}

ControllerNode::ControllerNode(psaf_shared::ImageSaver * imageSaver)
: ControllerInterface(
    CONTROLLER_NODE_NAME,
    STATE_TOPIC,
    TRAJECTORY_TOPIC,
    STOP_LINE_TOPIC,
    SIGN_TOPIC,
    GET_STEERING_TOPIC,
    GET_SPEED_TOPIC,
    SET_STEERING_TOPIC,
    SET_SPEED_TOPIC,
    SET_LIGHT_TOPIC,
    rclcpp::QoS(rclcpp::KeepLast {10})
), mImageSaver(imageSaver)
{
}

void ControllerNode::applyConfig(const ControllerConfig & cfg)
{
  mConfig = cfg;

  if (mConfig.debugOutput) {
    std::cout << mConfig << std::endl;
  }
}

ControllerConfig ControllerNode::getConfig() const
{
  return mConfig;
}

void ControllerNode::update()
{
  // check if we are in driving state, otherwise we dont need to process speed
  if (mConfig.ignoreState || state == 10) {
    // check if we got our first trajectory, otherwise we dont need to process speed
    if (m_got_first_trajectory) {
      // to prevent uCBridge timeout
      publishSpeed(m_last_requested_speed);

      // Problem: sensor sends nothing if the wheels don't turn or if they turn at a constant speed
      // -> we never get 0 as current speed
      // Solution: set speed value to 0 if no new values are recieved for 1 function call and
      // for speed that would not or barely move the wheels is requested
      const auto speedDuration = rclcpp::Clock().now() - m_last_speed_time;
      int timeSinceLastSpeedValueMs = static_cast<int>(
        speedDuration.to_chrono<std::chrono::milliseconds>().count());

      if ((timeSinceLastSpeedValueMs > 33) && (m_last_requested_speed < 150)) {
        m_current_speed = 0;
      }

      // set new base target speed
      auto newSpeed = mConfig.targetSpeed;

      // do sign processing if enabled
      if (mConfig.enableSigns) {
        timeoutActiveSigns();
        newSpeed = getSpeedFromActiveSigns();
      }

      // calculate target speed depending on config
      int speed_in_pwm =
        mConfig.useRegulation ? calculateSpeedPwmPID(newSpeed, m_current_speed) : convertSpeedToPWM(
        newSpeed);

      // needed because sensor can't register wheels turning bkacwards
      if (speed_in_pwm < 0) {
        speed_in_pwm = 0;
      }

      // publish new speed and store previous for debugging
      publishSpeed(speed_in_pwm);
      m_last_requested_speed = speed_in_pwm;

      // debug log output
      if (mConfig.debugOutput) {
        std::cout << "target_point: " << m_target_point << std::endl;

        if (m_trajectory.size() == 0) {
          std::cout << "Empty trajectory, steering uses old target point!" << std::endl;
        } else {
          std::cout << "current_trajectory: ";
          for (const auto & point : m_trajectory) {
            std::cout << point << ",";
          }
          std::cout << std::endl;
        }

        std::cout << "m_current_speed: " << m_current_speed << std::endl;
        std::cout << "speed_in_pwm: " << speed_in_pwm << std::endl;
        std::cout << "steering_angle: " << m_steering_angle << std::endl;
      }

      // debug images output
      if (mConfig.showImages || mConfig.saveImages) {
        auto speed_image = psaf_shared::plot_value(m_current_speed, 250, 0);
        auto angle_image = psaf_shared::plot_value(m_steering_angle, -30, 30, true);
        auto target_point_image = psaf_shared::plot_points(
          {m_trajectory, std::vector<cv::Point2f>{m_target_point}},
          {psaf_shared::colors::white, psaf_shared::colors::red},
          2.5, 0.5, true, cv::ROTATE_90_COUNTERCLOCKWISE, cv::Size(480, 640));

        if (mConfig.showImages) {
          psaf_shared::wrapped_imshow("Controller: LaneMarkingsWithTarget", target_point_image);
          psaf_shared::wrapped_imshow("Controller: Speed", speed_image);
          psaf_shared::wrapped_imshow("Controller: SteeringAngle", angle_image);
        }

        if (mConfig.saveImages && mImageSaver != nullptr) {
          mImageSaver->save(target_point_image, "target_point");
          mImageSaver->save(speed_image, "speed");
          mImageSaver->save(angle_image, "angle");
        }
      }

      // to prevent uCBridge timeout
      publishSpeed(m_last_requested_speed);
    } else {
      if (mConfig.debugOutput) {
        std::cout << "No trajectory arrived, do nothing\n";
      }
    }
  } else {
    if (mConfig.debugOutput) {
      std::cout << "Controller not in STATE_DRIVING, do nothing\n";
    }

    // reset steering and speed if we are not driving
    publishSteering(0, true);
    publishSpeed(0);
  }
}

void ControllerNode::processTrajectory(libpsaf_msgs::msg::Trajectory::SharedPtr trajectory)
{
  // check if we are in driving state, otherwise we dont need to process markings
  // if we ignore state skip this check
  if (!mConfig.ignoreState) {
    if (state != 10) {
      if (mConfig.debugOutput) {
        std::cout << "Controller not in STATE_DRIVING, do nothing\n";
      }

      // reset steering and speed if we are not driving
      publishSteering(0, true);
      publishSpeed(0);

      return;
    }
  }

  if (trajectory->points.empty()) {
    return;
  }

  if (mConfig.debugOutput) {
    std::cout << "Got new non-empty trajectory callback" << std::endl;
  }

  // replace our trajectory with new one
  m_trajectory.clear();
  for (const auto & point : trajectory->points) {
    m_trajectory.push_back(cv::Point2f(point.x, point.y));
  }

  if (m_trajectory.size() == 0) {
    // take old targetpoint if trajectory is empty
  } else {
    // update value for times when no new trajectory is available
    m_target_point = calculateTargetPoint(m_trajectory, mConfig.lookaheadDistance);
  }
  // calculate steering angle according to pure pursuit
  double steering_angle_rad = calculateSteeringAngle(m_target_point, mConfig.lookaheadDistance);
  // convert to deg
  m_steering_angle = steering_angle_rad * 180 / M_PI;

  double steering_angle_mapped = m_steering_angle * 2.5 - 2.395;  // für 60
  // values for steering_angle_mapped:
  // more than  30 degree / less than -30 degree are set to max / min value by ucBridge
  publishSteering(-steering_angle_mapped * 9, false);
  // end of trajectory processing

  // needs to be set, so that update() does something
  m_got_first_trajectory = true;
}

void ControllerNode::updateState(std_msgs::msg::Int64::SharedPtr state)
{
  this->state = state->data;
}

void ControllerNode::processSign(libpsaf_msgs::msg::Sign::SharedPtr sign)
{
  if (mConfig.enableSigns) {
    // all cases defined under
    // {ros_directory}/{ros_version}/include/libpsaf_msgs/libpsaf_msgs/msg/detail/sign__struct.hpp
    // active cases defined under
    // {workspace}/src/psaf_sign_detection/src/sign_id.cpp
    int signId;
    switch (sign->type) {
      case 0u:  // LIMIT_30_START
        signId = SIGN_LIMIT_30_START;
        break;

      case 1u:  // LIMIT_30_END
        signId = SIGN_LIMIT_30_END;
        // handle non timeouting signs
        m_active_signs[SIGN_LIMIT_30_START] = false;
        break;

      case 2u:  // CROSSWALK
        signId = SIGN_CROSSWALK;
        break;

      case 9u:  // STOP
        signId = SIGN_STOP;
        // handle non timeouting signs
        m_active_signs[SIGN_PRIORITY] = false;
        break;

      case 10u:  // PRIORITY
        signId = SIGN_PRIORITY;
        break;

      case 11u:  // YIELD
        signId = SIGN_YIELD;
        // handle non timeouting signs
        m_active_signs[SIGN_PRIORITY] = false;
        break;

      case 16u:  // UPHILL, used for SpeedLimit30 / SPEED_LIMIT_30
        signId = SIGN_SPEED_LIMIT_30;
        break;

      default:
        if (mConfig.debugOutput) {
          std::cout << "got non-handeled sign, do nothing" << std::endl;
        }
        return;
    }

    // set sign as active
    if (m_active_signs[signId] == false) {
      m_active_signs[signId] = true;
    }

    last_sign_time[signId] = rclcpp::Clock().now();
    m_got_first_sign = true;

    if (mConfig.debugOutput) {
      std::cout << "Sign detected with id " << signId << std::endl;
    }
  } else {
    if (mConfig.debugOutput) {
      std::cout << "Got Sign, but m_enable_signs is false." << std::endl;
    }
  }
}

bool ControllerNode::timeoutActiveSigns()
{
  bool retVal = false;
  // look for shield timeouts
  for (int signId = 0; signId < SIGN_LENGTH; signId++) {
    if (m_sign_has_timeout[signId] == true) {
      // get time since the last sign was published
      rclcpp::Duration signDuration = rclcpp::Clock().now() - last_sign_time[signId];

      unsigned int timeSinceLastSignMs =
        static_cast<unsigned int>(signDuration.to_chrono<std::chrono::milliseconds>().count());

      // set timeouted shield to inactive
      if (timeSinceLastSignMs >= mConfig.signTimeout) {
        m_active_signs[signId] = false;
        retVal = true;
      }
    }
  }

  return retVal;
}

int ControllerNode::getSpeedFromActiveSigns() const
{
  int speed = mConfig.targetSpeed;
  /* The if conditions need to be done here because there is a bug
   * because uncrustify and cpplint handle multiline else if conditions differently
   */
  const auto priority0 = m_active_signs[SIGN_STOP] || m_active_signs[SIGN_CROSSWALK];
  const auto priority1 = (m_active_signs[SIGN_LIMIT_30_START] ||
    m_active_signs[SIGN_SPEED_LIMIT_30]) && m_active_signs[SIGN_PRIORITY];
  const auto priority2 = m_active_signs[SIGN_LIMIT_30_START] || m_active_signs[SIGN_YIELD] ||
    m_active_signs[SIGN_SPEED_LIMIT_30];
  const auto priority3 = m_active_signs[SIGN_PRIORITY];

  // sign handeling, priotity based handeling
  if (priority0) {
    // stop, Priority 0
    speed = 0;
    if (mConfig.debugOutput) {
      std::cout << "State: Stop" << std::endl;
    }
  } else if (priority1) {
    // drive slow with priority sign (combination of Priority 2 and 3), Priority 1
    speed = mConfig.targetSpeed / 2 + 10;
    if (speed < 40) {
      speed = 40;
    }
    if (mConfig.debugOutput) {
      std::cout << "State: Drive Slow with Priority" << std::endl;
    }
  } else if (priority2) {
    // drive slow, Priority 2
    speed = mConfig.targetSpeed / 2;
    if (speed < 40) {
      speed = 40;
    }
    if (mConfig.debugOutput) {
      std::cout << "State: Drive Slow" << std::endl;
    }
  } else if (priority3) {
    // priority sign, Priority 3
    speed = mConfig.targetSpeed + 30;
    if (speed > 200) {
      speed = 200;
    }
    if (mConfig.debugOutput) {
      std::cout << "State: Priority" << std::endl;
    }
  } else {
    // normal behavior, Priority 4
    speed = mConfig.targetSpeed;
    if (mConfig.debugOutput) {
      std::cout << "State: Normal behavior, no sign" << std::endl;
    }
  }

  return speed;
}

void ControllerNode::processStopLine(libpsaf_msgs::msg::StopLine::SharedPtr line)
{
  (void) line;
}

void ControllerNode::updateSteering(std_msgs::msg::Int16::SharedPtr p)
{
  (void) p;
}

void ControllerNode::updateSpeed(std_msgs::msg::Int16::SharedPtr p)
{
  m_current_speed = p->data;
  // This callback won't be called if the wheel doesn't move.
  // This can lead to problems if the value isn't 0.
  // Thats why after some time we manually set it to 0.
  m_last_speed_time = rclcpp::Clock().now();
}

std::vector<cv::Point2f> ControllerNode::calculateCircleLineIntersection(
  const double & radius, const cv::Point2f & p1,
  const cv::Point2f & p2,
  const double tangent_tol,
  const bool segment_line)
{
  double dx = p2.x - p1.x;
  double dy = p2.y - p1.y;
  double dr = std::sqrt(pow(dx, 2) + pow(dy, 2));
  double big_d = p1.x * p2.y - p2.x * p1.y;
  double discriment = pow(radius * dr, 2) - pow(big_d, 2);
  std::vector<cv::Point2f> intersection;
  std::vector<int> signs;

  if (discriment < 0) {
    return intersection;
  }

  if (dy < 0) {
    signs = {1, -1};
  } else {
    signs = {-1, 1};
  }

  for (int sign : signs) {
    double x = (big_d * dy + sign * (dy < 0 ? -1 : 1) * dx * std::sqrt(discriment)) / (dr * dr);
    double y = (-big_d * dx + sign * std::abs(dy) * std::sqrt(discriment)) / (dr * dr);
    double temp;

    if (std::abs(dx) > std::abs(dy)) {
      temp = (x - p1.x) / dx;
    } else {
      temp = (y - p1.y) / dy;
    }

    if (!segment_line || (temp >= 0 && temp <= 1)) {
      intersection.emplace_back(cv::Point2f(x, y));
    }
  }

  if (intersection.size() == 2 && std::abs(discriment) <= tangent_tol) {
    intersection.resize(1);
  }

  return intersection;
}

int ControllerNode::convertSpeedToPWM(int speedInCmPerS)
{
  int speedInPwm = 5 * speedInCmPerS;

  if (speedInPwm > 1000) {
    speedInPwm = 1000;
  }

  return speedInPwm;
}

// in cm/s
int ControllerNode::calculateSpeedPwmPID(int target_speed, int current_speed)
{
  double derivative_term = 0;
  constexpr double dt = 1 / 30.0;
  constexpr double max_inter_term = 1000;

  double error = target_speed - current_speed;
  m_pid_inter_term += error * mConfig.pidKi * dt;

  // Anti-Windup: Limit the intern term
  if (m_pid_inter_term > max_inter_term) {
    m_pid_inter_term = max_inter_term;
  } else if (m_pid_inter_term < -max_inter_term) {
    m_pid_inter_term = -max_inter_term;
  }

  if (m_pid_last_error != 0.0) {
    derivative_term = (error - m_pid_last_error) * mConfig.pidKd / dt;
  }
  m_pid_last_error = error;
  double throttle = mConfig.pidKp * error + m_pid_inter_term + derivative_term;

  if (throttle < 0) {
    throttle = 0;
  }

  return static_cast<int>(throttle);
}

cv::Point2f ControllerNode::calculateTargetPoint(std::vector<cv::Point2f> trajectory, double ld)
{
  cv::Point2f target_point;

  // early return to prevent oob array access
  if (trajectory.empty()) {
    target_point.x = 0;
    target_point.y = 0;
    return target_point;
  }

  // choose the target point from the circle line intersections
  std::vector<cv::Point2f> intersections;
  for (size_t i = 0; i < trajectory.size() - 1; ++i) {
    const auto p1 = trajectory[i];
    const auto p2 = trajectory[i + 1];

    auto pts = calculateCircleLineIntersection(ld, p1, p2);
    intersections.insert(intersections.end(), pts.begin(), pts.end());
  }

  std::vector<cv::Point2f> filtered_intersections;

  for (const auto & p : intersections) {
    if (p.x > 0) {
      filtered_intersections.push_back(p);
    }
  }

  if (!filtered_intersections.empty()) {
    target_point = filtered_intersections.front();
  } else {
    // all points are ether closer than the ld or further away than the ld
    // that's why only one point has to be checked

    // get the radius of the circle with pythagoras
    double radius = sqrtf(pow(trajectory[0].x, 2) + pow(trajectory[0].y, 2) );

    // identifies the closest / furthest away point, starts at the first point
    int targetIdx = 0;
    // the shortest / longest distance, starts at the first point
    double distance = trajectory[0].x;
    // points are too far away, find closest
    if (radius > ld) {
      for (size_t i = 1; i < trajectory.size(); ++i) {
        if (trajectory[i].x < distance) {
          // set new distance
          distance = trajectory[i].x;
          // safe the number of the trajectory point
          targetIdx = i;
        }
      }
    } else {
      // points are too close, find furtest away
      for (size_t i = 1; i < trajectory.size(); ++i) {
        if (trajectory[i].x > distance) {
          // set new distance
          distance = trajectory[i].x;
          // safe the number of the trajectory point
          targetIdx = i;
        }
      }
    }
    target_point = trajectory[targetIdx];
  }

  return target_point;
}

double ControllerNode::calculateSteeringAngle(cv::Point2f target_point, double ld)
{
  double alpha = std::atan2(target_point.y, target_point.x);
  double steering_angle = std::atan((2 * mConfig.wheelBase * std::sin(alpha)) / ld);
  return steering_angle;
}
