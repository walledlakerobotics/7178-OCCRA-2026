#pragma once

#include <messages.hpp>

Buffer unescape(SerialBuffer &data, size_t count);

bool read_frame(SerialBuffer &data, Buffer &output);