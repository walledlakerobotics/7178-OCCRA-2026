#include <reading.hpp>
#include <xxhash.h>

Buffer unescape(SerialBuffer &data, size_t count)
{
    Buffer result(count);

    while (result.size() < count)
    {
        uint8_t byte = data.pop_blocking<uint8_t>(); // Skip bytes until we have enough data

        if (byte == escape_byte)
        {
            byte = data.pop_blocking<uint8_t>() ^ escape_mask; // Unescape the next byte
        }

        result.push<uint8_t>(byte);
    }

    return result;
}

bool read_frame(SerialBuffer &data, Buffer &output)
{
    while (data.pop_blocking<uint8_t>() != frame_signature); // Wait for the frame signature

    size_t length_tag = unescape(data, sizeof(size_t)).pop<size_t>(); // Read the length tag

    Buffer payload = unescape(data, length_tag); // Read the payload

    XXH32_hash_t received_checksum = unescape(data, sizeof(XXH32_hash_t)).pop<XXH32_hash_t>(); // Read the checksum

    XXH32_hash_t calculated_checksum = XXH32(payload.data(), payload.size(), 0); // Calculate the checksum of the payload

    if (received_checksum != calculated_checksum)
    {
        return false; // Checksum mismatch
    }

    output = std::move(payload); // Move the payload to the output

    return true;
}