#pragma once
#include "Types.h"

namespace CommandCodec {
bool decode(const uint8_t* payload, size_t length, AGVCommand& command, ErrorCode& reason);
}
