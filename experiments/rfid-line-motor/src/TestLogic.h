#pragma once
#include <cstdint>

namespace Bench {
enum class State { Stopped, Follow, Turning, Fault };
enum class Fault { None, LineLost, AmbiguousLine, TurnTimeout };
struct Line { bool left, center, right; };
struct Output { int left = 0; int right = 0; };
struct Tuning {
    int base, correction, turn;
    uint32_t minTurnMs, timeoutMs, stableMs;
    bool pivotRight;
};

// This is the actual controller used by main.cpp and native tests.
class Controller {
public:
    explicit Controller(Tuning tuning) : tuning_(tuning) {}
    void start();
    void stop();
    bool card1(uint32_t now, Line line);
    Output update(uint32_t now, Line line);
    State state() const { return state_; }
    Fault fault() const { return fault_; }
    bool cardUsed() const { return cardUsed_; }
private:
    Output pivot() const;
    Output fail(Fault reason);
    Tuning tuning_;
    State state_ = State::Stopped;
    Fault fault_ = Fault::None;
    bool cardUsed_ = false;
    bool leftCleared_ = false;
    bool candidate_ = false;
    uint32_t turnStarted_ = 0;
    uint32_t candidateStarted_ = 0;
};
}
