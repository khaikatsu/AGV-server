#include "TestLogic.h"

namespace Bench {
void Controller::start() {
    state_ = State::Follow;
    fault_ = Fault::None;
    cardUsed_ = false;
    candidate_ = false;
    leftCleared_ = false;
}
void Controller::stop() {
    state_ = State::Stopped;
    fault_ = Fault::None;
    candidate_ = false;
}
bool Controller::card1(uint32_t now, Line) {
    if (state_ != State::Follow || cardUsed_) return false;
    cardUsed_ = true; // One turn per 'g'; repeated reads cannot trigger another turn.
    state_ = State::Turning;
    turnStarted_ = now;
    leftCleared_ = false;
    candidate_ = false;
    return true;
}
Output Controller::pivot() const {
    return tuning_.pivotRight ? Output{tuning_.turn, -tuning_.turn}
                              : Output{-tuning_.turn, tuning_.turn};
}
Output Controller::fail(Fault reason) {
    state_ = State::Fault;
    fault_ = reason;
    return {};
}
Output Controller::update(uint32_t now, Line line) {
    if (state_ == State::Stopped || state_ == State::Fault) return {};
    if (state_ == State::Turning) {
        const uint32_t elapsed = now - turnStarted_;
        if (elapsed >= tuning_.timeoutMs) return fail(Fault::TurnTimeout);
        // Require stable absence, then a fresh stable detection on LEFT.
        const bool expected = leftCleared_ ? line.left : !line.left;
        if (!expected) candidate_ = false;
        else if (!candidate_) { candidate_ = true; candidateStarted_ = now; }
        if (candidate_ && uint32_t(now - candidateStarted_) >= tuning_.stableMs) {
            if (!leftCleared_) {
                leftCleared_ = true;
                candidate_ = false;
            } else if (elapsed >= tuning_.minTurnMs) {
                state_ = State::Follow;
                candidate_ = false;
            }
        }
        if (state_ == State::Turning) return pivot();
    }
    const unsigned pattern = (line.left ? 4u : 0u) |
                             (line.center ? 2u : 0u) | (line.right ? 1u : 0u);
    switch (pattern) {
        case 2: case 7: return {tuning_.base, tuning_.base};
        case 4: case 6: return {tuning_.correction, tuning_.base};
        case 1: case 3: return {tuning_.base, tuning_.correction};
        case 0: return fail(Fault::LineLost);
        default: return fail(Fault::AmbiguousLine);
    }
}
}
