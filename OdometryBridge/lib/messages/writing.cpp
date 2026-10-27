#include <writing.hpp>
#include <xxhash.h>

void push_escaping(Buffer &buffer, Buffer &data)
{
    buffer.reserve(data.size() * 2); // Reserve space for worst-case scenario

    while (!data.empty())
    {
        uint8_t byte = data.pop<uint8_t>();
        if (byte == frame_signature || byte == escape_byte)
        {
            buffer.push(escape_byte);
            buffer.push(byte ^ escape_mask); // XOR with 0x20 to escape the byte

            continue;
        }

        buffer.push(byte);
    }
}

Buffer create_frame(Buffer &payload)
{
    // Reserve space for worst-case scenario
    Buffer buffer(sizeof(frame_signature) + sizeof(size_t) * 2 + payload.size() * 2 + sizeof(XXH32_hash_t) * 2);

    buffer.push(frame_signature);

    // Length tag
    Buffer payload_size_buffer = Buffer::from_data(payload.size());

    push_escaping(buffer, payload_size_buffer);

    payload.mark();

    // Payload
    push_escaping(buffer, payload);

    payload.reset();

    // Checksum
    XXH32_hash_t checksum = XXH32(payload.data(), payload.size(), 0); // Calculate checksum using xxHash32
    Buffer checksum_buffer = Buffer::from_data(checksum);

    push_escaping(buffer, checksum_buffer);

    return buffer;
}