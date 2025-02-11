/**
 * @file configuration.hpp
 * @brief a general file to configure the project
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_CONFIGURATION__CONFIGURATION_HPP_
#define PSAF_CONFIGURATION__CONFIGURATION_HPP_

#include <string>
#include <vector>

/**
 * Set to false for PSAF 2
 */
#define PSAF1 true

/**
 * To enable additional printouts
 */
#define DEBUG false

/**
* This header file is used to set certain constants like topic and node names
* Changing the topic names is not advised. Please make sure to set the PSAF 1 flag above to match
* the* the current car. True for PSAF 1 and false for PSAF 2.
*/


/**
 * Define Node Names
 */
#define CONTROLLER_NODE_NAME "controller"
#define LANE_DETECTION_NODE_NAME  "lane_detection"
#define MANUAL_MODE_NODE_NAME  "manual_mode"
#define OBJECT_DETECTION_NODE_NAME  "object_detection"
#define PARKING_NODE_NAME  "parking"
#define PARKING_DETECTION_NODE_NAME  "parking_detection"
#define SIGN_DETECTION_NODE_NAME  "sign_detection"
#define STARTBOX_NODE_NAME  "startbox"
#define STATE_MACHINE_NODE_NAME  "state_machine"
#define TRAJECTORY_NODE_NAME  "trajectory"
#define WATCHDOG_NODE_NAME  "watchdog"


/**
* Define realsense topics
* The number of camera streams. Note: Even if only one realsense camera is mounted to the car,
* the different camera streams are seen as separate cameras. If you want more cameras
* simply increase the NBR_OF_CAMS and add the topic name to the vector
*/
#define NBR_OF_CAMS  2
#define CAM_TOPICS {"color/image_raw", "depth/image_rect_raw"}
#define NBR_OF_CAMS_RGB 1                   // Number of cameras for lane detection
#define CAM_TOPIC_RGB {"color/image_raw"}   // Camera topic for lane detection


/**
* Define the uc_bridge topic names
* Note: The PSAF 1 cars usually only have 3 ultrasonic sensors. For the PSAF 2 car
* comment out the current NBR_OF_US_SENSORS and US_TOPICS definitions and comment in
* the others.
*/
#define BUTTON_TOPIC  "uc_bridge/button"
#define SET_SPEED_TOPIC  "uc_bridge/set_motor_level"
#define GET_SPEED_TOPIC  "uc_bridge/get_speed"
#define SET_STEERING_TOPIC  "uc_bridge/set_steering"
#define GET_STEERING_TOPIC  "uc_bridge/get_steering"
#define SET_LIGHT_TOPIC  "uc_bridge/light"
#define MANUAL_SIGNALS_TOPIC  "uc_bridge/manual_signals"
/** FOR PSAF 1 Default car **/
#if PSAF1
#define NBR_OF_US_SENSORS  3
#define US_TOPICS {"uc_bridge/us_front_center", "uc_bridge/us_mid_right", "uc_bridge/us_mid_left"}
/** FOR PSAF 2 car **/
#else
#define NBR_OF_US_SENSORS  8
#define US_TOPICS {"uc_bridge/us_front_center", "uc_bridge/us_front_right", \
    "uc_bridge/us_mid_right", "uc_bridge/us_rear_right", "uc_bridge/us_rear_center", \
    "uc_bridge/us_rear_left", "uc_bridge/us_mid_left", "uc_bridge/us_front_left"}
#endif

/**
 * Define custom topic names
 */
#define LANE_MARKINGS_TOPIC  "lane_detection/lane_markings"
#define OBJECT_TOPIC  "object_detection/obstacle"
#define PARKING_SPOT_TOPIC  "parking_detection/parking_spot"
#define STATE_TOPIC  "state_machine/state"
#define STATUS_INFO_TOPIC  "status/status_info"
#define SIGN_TOPIC  "sign_detection/sign"
#define STOP_LINE_TOPIC  "lane_detection/stop_line"
#define TRAJECTORY_TOPIC  "trajectory/trajectory"
#define WATCHDOG_ERROR_TOPIC  "watchdog/error_message"


#endif  // PSAF_CONFIGURATION__CONFIGURATION_HPP_
