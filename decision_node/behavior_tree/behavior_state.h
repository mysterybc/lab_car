#pragma once
/**
 * @brief Behavior state
 */
enum class BehaviorState {
  RUNNING,   ///< Running state in process
  SUCCESS,   ///< Success state as result
  FAILURE,   ///< Failure state as result
  IDLE,      ///< Idle state, state as default or after cancellation
};

