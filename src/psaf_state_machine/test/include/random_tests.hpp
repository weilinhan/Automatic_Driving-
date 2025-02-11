/**
 * @file random_tests.hpp
 * @brief Helper file containing all the transitions and states
 * @author PSAF
 * @date 2022-06-01
 */
#ifndef STATE_MACHINE__RANDOM_TESTS_HPP_
#define STATE_MACHINE__RANDOM_TESTS_HPP_

#include <vector>
#include <map>

/**
 * @enum TRANSITIONS
 * @brief Enum containing all the possible transition triggers in the state machine
 */
enum TRANSITIONS
{
  STARTBOX_OPEN,
  UPHILL_START,
  DOWNHILL_START,
  DOWNHILL_END,
  STOP_LINE_APPROACH,
  STOP_LINE_REACHED,
  CONTINUE_NO_OBJECT,
  WAIT_FOR_OBJECT,
  OBJECT_DETECTED,
  PARKING_INTENT,
  PARKING_TIMEOUT,
  PARALLEL_FOUND,
  PERPENDICULAR_FOUND,
  PARKING_FINISHED,
  PARKING_FAILED,
  PARK_TIME_REACHED,
  BACK_ON_LANE,
  STATIC_OBSTACLE,
  DYNAMIC_OBSTACLE,
  OVERTAKE_POSSIBLE,
  PASSED_OBSTACLE,
  OVERTAKE_FINISHED,
  OVERTAKE_ABORT,
  MANUAL_MODE_ENTER,
  MANUAL_MODE_EXIT,
  WATCHDOG_TIMEOUT,
};

/**
 * @enum STATE
 * @brief Enum containing all the possible states in the state machine
 */
enum STATE
{
  STARTBOX,
  ERROR,
  MANUAL_MODE,
  STOP,
  SHUTDOWN,
  OT_CHECK,
  OT_FOLLOW,
  OT_INIT,
  OT_PERFORM,
  OT_FINISH,
  DR_NORMAL,
  DR_UP_HILL,
  DR_DOWN_HILL,
  DR_APPROACH,
  DR_WAIT,
  PR_SEARCH,
  PR_PARALLEL,
  PR_PERPENDICULAR,
  PR_FINISH,
  PR_RETURN
};

std::map<int, int> startbox {
  {STARTBOX_OPEN, DR_NORMAL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> drive_dis_0 {
  {UPHILL_START, DR_UP_HILL},
  {PARKING_INTENT, PR_SEARCH},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> drive_dis_1 {
  {UPHILL_START, DR_UP_HILL},
  {OBJECT_DETECTED, OT_CHECK},
  {STOP_LINE_APPROACH, DR_APPROACH},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> dr_up_hill {
  {DOWNHILL_START, DR_DOWN_HILL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> dr_down_hill {
  {DOWNHILL_END, DR_NORMAL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> dr_approach {
  {STOP_LINE_REACHED, DR_WAIT},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> dr_wait {
  {CONTINUE_NO_OBJECT, DR_NORMAL},
  {WAIT_FOR_OBJECT, DR_WAIT},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> ot_check {
  {DYNAMIC_OBSTACLE, OT_FOLLOW},
  {STATIC_OBSTACLE, OT_INIT},
  {OVERTAKE_ABORT, DR_NORMAL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> ot_follow {
  {OVERTAKE_POSSIBLE, OT_PERFORM},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
  {OVERTAKE_ABORT, DR_NORMAL},
};

std::map<int, int> ot_init {
  {OVERTAKE_POSSIBLE, OT_PERFORM},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
  {OVERTAKE_ABORT, DR_NORMAL},
};

std::map<int, int> ot_perform {
  {PASSED_OBSTACLE, OT_FINISH},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
  {OVERTAKE_ABORT, DR_NORMAL},
};

std::map<int, int> ot_finish {
  {OVERTAKE_FINISHED, DR_NORMAL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
  {OVERTAKE_ABORT, DR_NORMAL},
};

std::map<int, int> manual_mode {
  {MANUAL_MODE_EXIT, DR_NORMAL}
};

std::map<int, int> error {
  {MANUAL_MODE_ENTER, MANUAL_MODE},
  {WATCHDOG_TIMEOUT, SHUTDOWN}
};

std::map<int, int> pr_search {
  {PARALLEL_FOUND, PR_PARALLEL},
  {PERPENDICULAR_FOUND, PR_PERPENDICULAR},
  {PARKING_TIMEOUT, DR_NORMAL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> pr_parallel {
  {PARKING_FINISHED, PR_FINISH},
  {PARKING_FAILED, PR_RETURN},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> pr_perpendicular {
  {PARKING_FINISHED, PR_FINISH},
  {PARKING_FAILED, PR_RETURN},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> pr_finish {
  {PARK_TIME_REACHED, PR_RETURN},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> pr_return {
  {BACK_ON_LANE, DR_NORMAL},
  {MANUAL_MODE_ENTER, MANUAL_MODE},
};

std::map<int, int> shutdown {
  {-1, -1}
};

std::map<int, int> empty {
  {-2, -2}
};

/**
 * The lookup table for the state machine. The table containts each state that is reachable from
 * the current state via the transition.
 */
std::vector<std::vector<std::map<int, int>>> state_table {
  {startbox, startbox},
  {error, error},
  {manual_mode, manual_mode},
  {empty, empty},
  {shutdown, shutdown},
  {empty, ot_check},
  {empty, ot_follow},
  {empty, ot_init},
  {empty, ot_perform},
  {empty, ot_finish},
  {drive_dis_0, drive_dis_1},
  {dr_up_hill, dr_up_hill},
  {dr_down_hill, dr_down_hill},
  {empty, dr_approach},
  {empty, dr_wait},
  {pr_search, empty},
  {pr_parallel, empty},
  {pr_perpendicular, empty},
  {pr_finish, empty},
  {pr_return, empty},
};

#endif  // STATE_MACHINE__RANDOM_TESTS_HPP_
