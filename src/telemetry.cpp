#include "telemetry.hpp"
#include <cmath>

namespace aerotelem {

// Binary frame format (8 bytes):
// [0: 0xAA] [1: 0x55] [2: Roll MSB] [3: Roll LSB] [4: Pitch MSB] [5: Pitch LSB] [6: Batt %] [7: Checksum XOR(2..6)]
void encode_frame(const TelemetryData& data, uint8_t* buffer) {
    buffer[0] = 0xAA; // Header 1
    buffer[1] = 0x55; // Header 2

    buffer[2] = (data.roll_decideg >> 8);   // Roll MSB
    buffer[3] = (data.roll_decideg & 0xFF); // Roll LSB

    buffer[4] = (data.pitch_decideg >> 8);   // Pitch MSB
    buffer[5] = (data.pitch_decideg & 0xFF); // Pitch LSB

    buffer[6] = (data.battery_pct); // Battery %

    uint8_t checksum = 0;
    for (int i = 2; i <= 6; i++) {
        checksum ^= buffer[i];
    }
    buffer[7] = checksum; // XOR Checksum
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

// Sliding-window Moving Average Filter
MovingAverageFilter::MovingAverageFilter(size_t window_size) {
    if (window_size == 0) {
        window_size_ = 1;
    } else {
        window_size_ = window_size;
    }
}

int16_t MovingAverageFilter::update(int16_t new_sample) {
    history_.push_back(new_sample);
    if (history_.size() > window_size_) {
        history_.pop_front();
    }
    int32_t sum = 0;
    for (int16_t val : history_) {
        sum += val;
    }
    return static_cast<int16_t>(sum / static_cast<int32_t>(history_.size()));
}

void MovingAverageFilter::reset() {
    history_.clear();
}

// High-level Telemetry Driver
TelemetryDriver::TelemetryDriver(size_t filter_window)
    : parser_(),
      roll_filter_(filter_window),
      pitch_filter_(filter_window),
      latest_data_{0, 0, 0},
      valid_frames_(0) {
}

bool TelemetryDriver::process_byte(uint8_t byte) {
    if (parser_.push_byte(byte, latest_data_)) {
        latest_data_.roll_decideg = roll_filter_.update(latest_data_.roll_decideg);
        latest_data_.pitch_decideg = pitch_filter_.update(latest_data_.pitch_decideg);
        valid_frames_++;
        return true;
    }
    return false;
}

SafetyStatus TelemetryDriver::evaluate_safety() const {
    if (std::abs(latest_data_.roll_decideg) > MAX_SAFE_ANGLE_DECIDEG ||
        std::abs(latest_data_.pitch_decideg) > MAX_SAFE_ANGLE_DECIDEG) {
        return SafetyStatus::CRITICAL_ANGLE;
    }
    if (latest_data_.battery_pct < LOW_BATTERY_THRESHOLD) {
        return SafetyStatus::LOW_BATTERY;
    }
    return SafetyStatus::NOMINAL;
}

const TelemetryData& TelemetryDriver::latest_data() const {
    return latest_data_;
}

uint32_t TelemetryDriver::valid_frames() const {
    return valid_frames_;
}

// Thread-safe Telemetry Wrapper (Multi-threading protection with std::mutex)
ThreadSafeTelemetry::ThreadSafeTelemetry(size_t filter_window)
    : driver_(filter_window) {
}

bool ThreadSafeTelemetry::process_byte(uint8_t byte) {
    std::lock_guard<std::mutex> lock(mtx_);
    return driver_.process_byte(byte);
}

TelemetrySnapshot ThreadSafeTelemetry::get_snapshot() const {
    std::lock_guard<std::mutex> lock(mtx_);
    return TelemetrySnapshot{driver_.latest_data(), driver_.evaluate_safety(), driver_.valid_frames()};
}

} // namespace aerotelem
