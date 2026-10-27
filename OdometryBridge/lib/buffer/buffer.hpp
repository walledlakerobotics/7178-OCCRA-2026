#pragma once

#include <vector>
#include <cstdint>
#include <cstring>
#include <stdexcept>

#include <Arduino.h>

class Buffer
{
    private:
        std::vector<uint8_t> buffer;
        size_t reading_cursor = 0;
        size_t writing_cursor = 0;

        size_t marked_reading_cursor = 0;

    public:
        Buffer(size_t initial_size = 0) : buffer(initial_size) {}

        template <typename T>
        void push(const T &data)
        {
            if (buffer.size() < writing_cursor + sizeof(T)) {
                buffer.resize(writing_cursor + sizeof(T));
            }

            std::memcpy(&buffer[writing_cursor], &data, sizeof(T));
            writing_cursor += sizeof(T);
        }

        template <typename T>
        T pop()
        {
            if (size() < sizeof(T))
            {
                throw std::out_of_range("Type size exceeds buffer size");
            }

            T data;
            std::memcpy(&data, &buffer[reading_cursor], sizeof(T));
            reading_cursor += sizeof(T);

            return data;
        }

        inline void reserve(size_t capacity)
        {
            buffer.resize(writing_cursor + capacity);
        }

        inline void clear()
        {
            buffer.erase(buffer.begin(), buffer.begin() + reading_cursor);

            reading_cursor = 0;
            writing_cursor -= reading_cursor;
        }

        inline size_t size() const
        {
            return writing_cursor - reading_cursor;
        }

        inline bool empty() const
        {
            return size() == 0;
        }

        inline const uint8_t *data() const
        {
            return &buffer[reading_cursor];
        }

        inline uint8_t *write_ptr(size_t size)
        {
            reserve(size);

            uint8_t *dest = &buffer[writing_cursor];

            writing_cursor += size;

            return dest;
        }

        inline void mark()
        {
            marked_reading_cursor = reading_cursor;
        }

        inline void reset()
        {
            reading_cursor = marked_reading_cursor;
        }

        template <typename T>
        inline static Buffer from_data(const T &data)
        {
            Buffer buffer(sizeof(T));
            buffer.push(data);

            return buffer;
        }
};

template <>
inline void Buffer::push(const uint8_t &byte)
{
    if (buffer.size() < writing_cursor + 1)
    {
        buffer.resize(writing_cursor + 1);
    }

    buffer[writing_cursor++] = byte;
}

template <>
inline uint8_t Buffer::pop()
{
    if (empty())
    {
        throw std::out_of_range("Buffer is empty");
    }

    return buffer[reading_cursor++];
}

class SerialBuffer : public Buffer
{  
    private:
        SerialUSB &serial;

    public:
        SerialBuffer(SerialUSB &serial, size_t initial_size = 0) : Buffer(initial_size), serial(serial) {}

        template <typename T>
        T pop_blocking()
        {
            while (size() < sizeof(T)) {
                serial.readBytes(write_ptr(serial.available()), serial.available());

                yield();
                delay(10);
            }

            return pop<T>();
        }
};