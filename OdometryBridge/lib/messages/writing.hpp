#pragma once

#include <messages.hpp>

Buffer create_frame(Buffer &payload);

void push_escaping(Buffer &buffer, Buffer &data);