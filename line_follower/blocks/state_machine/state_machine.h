// Copyright (c) 2024 Henrik Söderlund

#ifndef LINE_FOLLOWER_BLOCKS_STATE_MACHINE_STATE_MACHINE_H_
#define LINE_FOLLOWER_BLOCKS_STATE_MACHINE_STATE_MACHINE_H_

#include <algorithm>
#include <cstdint>
#include <list>
#include <memory>
#include <utility>

#include "line_follower/blocks/common/maybe.h"

namespace line_follower {

template <class Context>
class State {
 public:
    State() {}

    virtual ~State() noexcept = default;

    State(State const&) = delete;
    State(State&&) = delete;
    State& operator=(State const&) = delete;
    State& operator=(State&&) = delete;

    /// @brief Enter function
    ///
    /// @param[in, out] context A state machine context
    virtual void enter(Context& context) = 0;

    /// @brief State step function
    ///
    /// @param[in, out] context A state machine context
    virtual void step(Context& context) = 0;

    /// @brief State transition function
    ///
    /// @param[in, out] context A state machine context
    /// @returns A pointer to new state or noop
    virtual Maybe<State<Context>*> transition(Context& context) = 0;
};

template <class Context>
class StateMachine {
 public:
    virtual ~StateMachine() noexcept = default;

    StateMachine(State<Context>* initial_state, Context& context)
        : current_state_{initial_state}, context_{context} {
        current_state_->enter(context_);
    }

    StateMachine(StateMachine const&) = delete;
    StateMachine(StateMachine&&) = delete;
    StateMachine& operator=(StateMachine const&) = delete;
    StateMachine& operator=(StateMachine&&) = delete;

    /// Steps the state machine and transitions into new states
    void step() {
        current_state_->step(context_);
        Maybe<State<Context>*> maybe_state = current_state_->transition(context_);

        if (maybe_state) {
            current_state_ = maybe_state.value;
            current_state_->enter(context_);
        }
    }

    Context& context() { return context_; }
    Context const& context() const { return context_; }

 private:
    State<Context>* current_state_;
    Context& context_;
};

}  // namespace line_follower

#endif  // LINE_FOLLOWER_BLOCKS_STATE_MACHINE_STATE_MACHINE_H_
