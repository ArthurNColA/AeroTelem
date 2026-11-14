#include "telemetry.hpp"

namespace aerotelem {

void encode_frame(const TelemetryData& data, uint8_t* buffer) {
    buffer[0] = 0xAA;
    buffer[1] = 0x55;

    buffer[2] = (data.roll_decideg >> 8);
    buffer[3] = (data.roll_decideg & 0xFF);

    buffer[4] = (data.pitch_decideg >> 8);
    buffer[5] = (data.pitch_decideg & 0xFF);

    buffer[6] = (data.battery_pct);

    uint8_t checksum = 0;
    for (int i = 2; i <= 6; i++) {
        checksum ^= buffer[i];
    }
    buffer[7] = checksum;
}

bool decode_frame(const uint8_t* buffer, TelemetryData& out_data) {
    if (buffer[0] != 0xAA || buffer[1] != 0x55) {
        return false;
    }

    uint8_t checksum = 0;
    for (int i = 2; i <= 6; i++) {
        checksum ^= buffer[i];
    }
    if (buffer[7] != checksum) {
        return false;
    }

    out_data.roll_decideg = static_cast<int16_t>((buffer[2] << 8) | buffer[3]);
    out_data.pitch_decideg = static_cast<int16_t>((buffer[4] << 8) | buffer[5]);
    out_data.battery_pct = buffer[6];

    return true;
}

FrameParser::FrameParser() {
    state_ = State::WAIT_HEADER_1;
    index_ = 0;
}

bool FrameParser::push_byte(uint8_t byte, TelemetryData& out_data) {
    switch (state_) {
        case State::WAIT_HEADER_1:
            if (byte == 0xAA) {
                buffer_[0] = byte;
                state_ = State::WAIT_HEADER_2;
                index_++;
            }
            break;

        case State::WAIT_HEADER_2:
            if (byte == 0x55) {
                buffer_[1] = byte;
                state_ = State::READ_PAYLOAD;
                index_++;
            } else {
                state_ = State::WAIT_HEADER_1;
                index_ = 0;
            }
            break;

        case State::READ_PAYLOAD:
            buffer_[index_] = byte;
            index_++;
            if (index_ == FRAME_SIZE) {
                state_ = State::WAIT_HEADER_1;
                index_ = 0;
                return decode_frame(buffer_, out_data);
            }
            break;
    }
    return false;
}

} // namespace aerotelem
