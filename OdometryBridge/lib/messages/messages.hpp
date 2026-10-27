#pragma once

#include <buffer.hpp>

constexpr uint8_t frame_signature = 0xAA;
constexpr uint8_t escape_byte = 0xBB;
constexpr uint8_t escape_mask = 0x20;

struct p_data
{
    uint32_t encoder_x = 0;
    uint32_t encoder_y = 0;

    float position_x = 0.0;
    float position_y = 0.0;
    float position_heading = 0.0;

    float quaternion_w = 0.0;
    float quaternion_x = 0.0;
    float quaternion_y = 0.0;
    float quaternion_z = 0.0;

    float velocity_x = 0.0;
    float velocity_y = 0.0;
    float velocity_heading = 0.0;

    inline constexpr size_t size() const
    {
        return sizeof(uint32_t) * 2 + sizeof(float) * 10;
    }

    inline Buffer to_bytes() const
    {
        Buffer bytes(size());

        bytes.push(encoder_x);
        bytes.push(encoder_y);
        bytes.push(position_x);
        bytes.push(position_y);
        bytes.push(position_heading);
        bytes.push(quaternion_w);
        bytes.push(quaternion_x);
        bytes.push(quaternion_y);
        bytes.push(quaternion_z);
        bytes.push(velocity_x);
        bytes.push(velocity_y);
        bytes.push(velocity_heading);

        return bytes;
    }

    inline static p_data from_bytes(Buffer &buffer)
    {
        p_data data;

        data.encoder_x = buffer.pop<uint32_t>();
        data.encoder_y = buffer.pop<uint32_t>();

        data.position_x = buffer.pop<float>();
        data.position_y = buffer.pop<float>();
        data.position_heading = buffer.pop<float>();

        data.quaternion_w = buffer.pop<float>();
        data.quaternion_x = buffer.pop<float>();
        data.quaternion_y = buffer.pop<float>();
        data.quaternion_z = buffer.pop<float>();

        data.velocity_x = buffer.pop<float>();
        data.velocity_y = buffer.pop<float>();
        data.velocity_heading = buffer.pop<float>();

        return data;
    }
};

struct reset_data
{
    bool calibrate_imu = false;

    bool set_position = false;

    float position_x = 0.0;
    float position_y = 0.0;
    float position_heading = 0.0;

    bool set_offsets = false;
    float offset_x = 0.0;
    float offset_y = 0.0;

    bool set_directions = false;
    bool x_inverted = true;
    bool y_inverted = true;

    bool set_resolution = false;
    float resolution = 0.0;

    inline constexpr size_t size() const
    {
        return sizeof(bool) * 2 + sizeof(float) * 3 + sizeof(bool) + sizeof(float) * 2 + sizeof(bool) * 4 + sizeof(float);
    }

    inline Buffer to_bytes() const
    {
        Buffer bytes(size());

        bytes.push(calibrate_imu);
        bytes.push(set_position);

        bytes.push(position_x);
        bytes.push(position_y);
        bytes.push(position_heading);

        bytes.push(set_offsets);
        bytes.push(offset_x);
        bytes.push(offset_y);

        bytes.push(set_directions);
        bytes.push(x_inverted);
        bytes.push(y_inverted);

        bytes.push(set_resolution);
        bytes.push(resolution);

        return bytes;
    }

    inline static reset_data from_bytes(Buffer &buffer)
    {
        reset_data data;

        data.calibrate_imu = buffer.pop<bool>();
        data.set_position = buffer.pop<bool>();

        data.position_x = buffer.pop<float>();
        data.position_y = buffer.pop<float>();
        data.position_heading = buffer.pop<float>();

        data.set_offsets = buffer.pop<bool>();
        data.offset_x = buffer.pop<float>();
        data.offset_y = buffer.pop<float>();

        data.set_directions = buffer.pop<bool>();
        data.x_inverted = buffer.pop<bool>();
        data.y_inverted = buffer.pop<bool>();

        data.set_resolution = buffer.pop<bool>();
        data.resolution = buffer.pop<float>();

        return data;
    }
};