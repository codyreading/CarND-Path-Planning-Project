#include <iostream>
#include "state_machine.hpp"
#include "utils.hpp"

State::State()
{
}

State::State(LongitudinalState lon_state, LateralState lat_state, int current_lane, int future_lane)
{
  this->s_state = lon_state;
  this->d_state = lat_state;
  this->current_lane = current_lane;
  this->future_lane = future_lane;
}

State::~State()
{
}

StateMachine::StateMachine()
{
}

StateMachine::StateMachine(State s)
{
  this->current_state = s;
  this->current_timestep = 0;
  this->timestep_lock = -1;
}

const State StateMachine::getCurrentState() const
{
  return this->current_state;
}

void StateMachine::updateState(State next_state, int keep_until_timestep)
{
  // cout << "**** UPDATED STATE MACHINE ***" << endl;
  this->current_state.s_state = next_state.s_state;
  this->current_state.d_state = next_state.d_state;
  this->current_state.current_lane = next_state.current_lane;
  this->current_state.future_lane = next_state.future_lane;
  ++this->current_timestep;
  this->timestep_lock = keep_until_timestep;
}

std::vector<State> StateMachine::nextPossibleStates()
{
  std::vector<State> future_states;
  switch (this->current_state.d_state)
  {
  case LateralState::STAY_IN_LANE:
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    future_states.push_back(State(LongitudinalState::ACCELERATE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    future_states.push_back(State(LongitudinalState::DECELERATE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    // TODO do we ever want to return LongitudinalState::STOP ??

    // When evaluating whether to change lane, always stay at the same speed
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::PREPARE_CHANGE_LANE_LEFT,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane - 1));
    future_states.push_back(State(LongitudinalState::ACCELERATE,
                                  LateralState::PREPARE_CHANGE_LANE_LEFT,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane - 1));
    future_states.push_back(State(LongitudinalState::DECELERATE,
                                  LateralState::PREPARE_CHANGE_LANE_LEFT,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane - 1));

    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane + 1));
    future_states.push_back(State(LongitudinalState::ACCELERATE,
                                  LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane + 1));
    future_states.push_back(State(LongitudinalState::DECELERATE,
                                  LateralState::PREPARE_CHANGE_LANE_RIGHT,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane + 1));

    break;
  case LateralState::PREPARE_CHANGE_LANE_LEFT:
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::CHANGE_LANE_LEFT,
                                  this->current_state.future_lane,
                                  this->current_state.future_lane));
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    future_states.push_back(State(LongitudinalState::DECELERATE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    break;

  case LateralState::PREPARE_CHANGE_LANE_RIGHT:
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::CHANGE_LANE_RIGHT,
                                  this->current_state.future_lane,
                                  this->current_state.future_lane));
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    future_states.push_back(State(LongitudinalState::DECELERATE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
    break;

  default:
    future_states.push_back(State(LongitudinalState::MAINTAIN_COURSE,
                                  LateralState::STAY_IN_LANE,
                                  this->current_state.current_lane,
                                  this->current_state.current_lane));
  }

  std::vector<State> possible_states;
  for (const State &future_state : future_states)
  {
    if (!isLaneValid(future_state.current_lane) || !isLaneValid(future_state.future_lane))
    {
      continue;
    }
    possible_states.push_back(future_state);
  }
  return possible_states;
}

StateMachine::~StateMachine() {}