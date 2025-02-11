/**
 * @file controller_node.hpp
 * @brief Controller for the psaf 1 cars
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_CONTROLLER__CONTROLLER_NODE_HPP_
#define PSAF_CONTROLLER_CONTROLLER_NODE_HPP_

#include <string>
#include <vector>

#include "libpsaf/interface/controller_interface.hpp"
#include "psaf_configuration/configuration.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "psaf_shared/image_saver.hpp"

/**
 * @brief Struct class used to encapsulate all configs for a ControllerNode
 */
struct ControllerConfig
{
  // this nodes update frequency
  double updateFrequency = 60.0;
  // whether to log debug messages or not
  bool debugOutput = false;
  // whether to drive regardless of state or not
  bool ignoreState = false;

  // whether to use the pid controller or constant speed
  bool useRegulation = false;
  // target speed in updateSpeed() value
  unsigned int targetSpeed = 40;

  // wheel base distance of the car
  static constexpr double wheelBase = 25.8;

  // ld for the pure pursuit algorithm
  static constexpr double ldMultiplier = 4.0;
  double lookaheadDistance = ldMultiplier * wheelBase;

  // whether to imshow debug images
  bool showImages = false;
  // whether to save debug images to disk
  bool saveImages = false;

  // whether to enable sign detection behaviour or not
  bool enableSigns = true;
  // time in ms after which a triggerd sign is disabled
  unsigned int signTimeout = 3000;

  // terms for the speed pid controller
  static constexpr double pidKp = 0.0506996467430549;
  static constexpr double pidKi = 1.53635293160773;
  static constexpr double pidKd = 0.000418272085630203;

  /**
   * @brief Load a ControllerConfig from parameters using the provided node
   * @param node the node to declare the parameters on
   * @return ControllerConfig the created instance
   */
  static ControllerConfig fromParameters(rclcpp::Node & node);

  /**
   * @brief Output this config to an ostream
   * @param os the ostream to output to
   * @param cfg the config to output to the stream
   * @return std::ostream& reference to the input ostream
   */
  friend std::ostream & operator<<(std::ostream & os, const ControllerConfig & cfg);
};

/**
 * @class ControllerNode
 * @implements ControllerInterface
 * @brief This class is responsible for calculating the control signals for the psaf cars
 * @details This class is the main class of the controller node. It is responsible for
 * controlling the car, i.e controlling the speed and steering angle.
 * To do so, it uses the calculated trajectory and the current speed and steering angle of the
 * car to calculate the control signals.
 */
class ControllerNode : public libpsaf::ControllerInterface
{
public:
  /**
  * @brief Construct a new Controller Node object
  */
  explicit ControllerNode(psaf_shared::ImageSaver * imageSaver = nullptr);

  /**
   * @brief Method in which the results get published
   * @details This method is called periodically by the main method of the node.
   */
  void update();

  /**
   * @brief Apply the ControllerConfig to this node
   * @param cfg the ControllerConfig to apply to this node
   */
  void applyConfig(const ControllerConfig & cfg);

  /**
   * @brief Get this nodes current ControllerConfig
   * @return ControllerConfig the current ControllerConfig
   */
  ControllerConfig getConfig() const;

protected:
  /**
   * @brief Callback method for the Trajectory message
   * @param trajectory the calculated trajectory
   */
  void processTrajectory(libpsaf_msgs::msg::Trajectory::SharedPtr trajectory) override;

  /**
   * @brief Callback method for the state
   * @param state the current state of the state machine
   */
  void updateState(std_msgs::msg::Int64::SharedPtr state) override;

  /**
   * @brief Callback method for the current sign
   * @param sign the current sign
   */
  void processSign(libpsaf_msgs::msg::Sign::SharedPtr sign) override;

  /**
   * @brief Callback method for the stop line
   * @param line the stop line
   */
  void processStopLine(libpsaf_msgs::msg::StopLine::SharedPtr line) override;

  /**
   * @brief Callback method for the measured steering angle
   * @details Please note. At this point the requested steering angle is returned by the
   * uc_bridge instead of the measured steering angle
   * @param p the current steering angle of the car
   */
  void updateSteering(std_msgs::msg::Int16::SharedPtr p) override;

  /**
   * @brief Callback method for the measured speed
   * @param p the current speed of the in cm/s
   */
  void updateSpeed(std_msgs::msg::Int16::SharedPtr p) override;

  /**
   * @brief Check all active signs for timeout expiry and disable them
   * @return true if a sign was disabled else false
   */
  bool timeoutActiveSigns();

  /**
   * @brief Check active signs to calculate the current target speed
   * @return int the current target speed
   */
  int getSpeedFromActiveSigns() const;

  /**
   * @brief Calculate the PWM value for a wanted speed with regard to the current speed
   * @details This function calculates a PWM value between 0 and 1000.
   * It uses the current speed which is read out form a hall sensor.
   * Together with the target speed in cm/s a PID regulator calculates a PWM value between -1000 and 1000.
   * Because the hall sensor can't detect the direction of the wheel negative PWM values are forbidden.
   * @param target_speed the requested speed in cm/s
   * @param current_speed of the car in cm/s
   * @return int the calculated pwm value
   */
  int calculateSpeedPwmPID(int target_speed, int current_speed);

  /**
   * @brief Roughtly calculate the PWM value for a wanted speed
   * @details Multiplies the speed with 5. A factor between 4 and 5 delivers delivers feasible results.
   * Returns 1000 if the value is bigger than 1000.
   * @param speedInCmPerS the requested speed in cm/s
   * @return int the calculated pwm value
   */
  int convertSpeedToPWM(int speedInCmPerS);

  /**
   * @brief Gets the target point for a given lookahead distance
   * @warning Returns a (0,0) point if the trajectory is empty
   * @details Part of the pure pursuit approach for steering
   * @param trajectory trajectory points to use for calculation
   * @param ld the lookahead distance
   * @return cv::Point2f the target point
   */
  cv::Point2f calculateTargetPoint(std::vector<cv::Point2f> trajectory, double ld);

  /**
   * @brief Method to caculate intersections between circle and line
   * @param radius circle radius
   * @param p1 The point1(x,y)on the line segment
   * @param p2 The point2(x,y)on the line segment
   * @param tangent_tol Numerical tolerance at which we decide the intersections are enough to consider it a tagent
   * @param segment_line True will find intersection only within the line segment, Fales will will find intersections along the line
   * @return pointer to point vector of all possible intersections
   */
  std::vector<cv::Point2f> calculateCircleLineIntersection(
    const double & radius, const cv::Point2f & p1,
    const cv::Point2f & p2,
    const double tangent_tol = 1e-9,
    const bool segment_line = true);

  /**
   * @brief Calculates the steering angle for a given target point.
   * @details Part of the pure pursuit approach for steering
   * @param target_point the point to calculate the steering angle for
   * @param ld the lookahead distance to use for the calculation
   * @return double the calculated steering angle in rad
   */
  double calculateSteeringAngle(cv::Point2f target_point, double ld);

private:
  /**
   * @brief Config of this node instance
   */
  ControllerConfig mConfig;

  /**
   * @brief The speed measured by the hall sensor.
   */
  int m_current_speed;

  /**
   * @brief The current trajectory, used to calculate a target point
   */
  std::vector<cv::Point2f> m_trajectory;

  /**
   * @brief Time since the last function call of updateSpeed() method
   */
  rclcpp::Time m_last_speed_time;

  /**
   * @brief The old target point, saved in case the camera doesn't send a new one in time
   */
  cv::Point2f m_target_point{0.0, 0.0};

  /**
   * @brief The speed requested in the previous call of the update() method
   */
  int m_last_requested_speed = 0;

  /**
   * @brief The last error of the calculateSpeedPwmPID(), used for the D term
   */
  double m_pid_last_error = 0;

  /**
   * @brief The intern term of the calculateSpeedPwmPID(), initialized here for easy usage of +=
   */
  double m_pid_inter_term = 0;

  /**
   * @brief the steering angle in degree that we get from pure pursuit
   * @details note that the angle is still mapped to a linear function before it is send to the uCBridge
   */
  double m_steering_angle = 0;

  /**
   * @brief shows that a trajectory arrived
   * @details the processTrajectory() will set this to true and only then the update() method will do something
   */
  bool m_got_first_trajectory = false;

  /**
   * @brief enum of signs the software can detect
   */
  enum sign
  {
    SIGN_LIMIT_30_START,
    SIGN_LIMIT_30_END,
    SIGN_CROSSWALK,
    SIGN_STOP,
    SIGN_PRIORITY,
    SIGN_YIELD,
    SIGN_SPEED_LIMIT_30,
    SIGN_LENGTH
  };

  /**
   * @brief shows which signs are active atm
   */
  bool m_active_signs[SIGN_LENGTH] = {0};

  /**
   * @brief Time since the last function call of processSign() method for a specific sign
   */
  rclcpp::Time last_sign_time[SIGN_LENGTH];

  /**
   * @brief shows whether a sign becomes irrelevant if its not seen anymore
   */
  const bool m_sign_has_timeout[SIGN_LENGTH] =
  {
    false,
    true,
    true,
    true,
    false,
    true,
    true
  };

  /**
   * @brief shows that a sign arrived
   * @details the processSign() will set this to true and only then the update() method will process signs
   */
  bool m_got_first_sign = false;

  // ImageSaver instance used to save debug images
  psaf_shared::ImageSaver * const mImageSaver = nullptr;

#ifdef BUILD_UNIT_TEST
  friend class ControllerUnitTest;
  friend class ControllerUnitTest_TestStateUpdate_Test;
  friend class ControllerUnitTest_TestSteeringPublishing_Test;
  friend class ControllerUnitTest_TestSpeedPublishing_Test;
  friend class ControllerUnitTest_TestSimpleCallbacks_Test;
  friend class ControllerUnitTest_TestUpdateConditions_Test;
#endif
};

#endif  // PSAF_CONTROLLER_CONTROLLER_NODE_HPP_
