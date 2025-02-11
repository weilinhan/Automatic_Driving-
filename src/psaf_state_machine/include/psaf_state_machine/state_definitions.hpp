/**
 * @file state_definitions.hpp
 * @brief The state definitions for the state machine
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef PSAF_STATE_MACHINE__STATE_DEFINITIONS_HPP_
#define PSAF_STATE_MACHINE__STATE_DEFINITIONS_HPP_

/**
 * @enum STATE
 * @brief The state definitions for the state machine
 */
enum STATE
{
  STARTBOX,           // 0 Initial State, Car in Startbox
  ERROR,              // 1 Error State, Critical error occurred. Stop car
  MANUAL_MODE,        // 2 Manual driving via remote control
  STOP,               // 3 Stop in case of error - not used yet
  SHUTDOWN,           // 4 Timeout after error, shutting down car. Recovery impossible

  // Overtaking = Obstacle evasion
  OT_CHECK,           // 5 Check if overtaking is possible
  OT_FOLLOW,          // 6 Follow dynamic obstacle
  OT_INIT,            // 7 Initiate the lanechange
  OT_PERFORM,         // 8 Perform the lanechange
  OT_FINISH,          // 9 Finish the lanechange

  // Driving
  DR_NORMAL,          // 10 Driving in the right lane
  DR_UP_HILL,         // 11 Driving UP Hill
  DR_DOWN_HILL,       // 12 Driving DOWN Hill
  DR_APPROACH,        // 13 Approach an intersection
  DR_WAIT,            // 14 Wait at an Intersection

  // Parking
  PR_SEARCH,          // 15 Search for parking spot
  PR_PARALLEL,        // 16 Parallel Park
  PR_PERPENDICULAR,   // 17 Perpendicular park
  PR_FINISH,          // 18 Finish Parking and wait for xx s
  PR_RETURN           // 19 Leave the parking spot and resume driving
};

#endif  // PSAF_STATE_MACHINE__STATE_DEFINITIONS_HPP_
