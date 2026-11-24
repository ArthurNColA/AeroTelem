#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <cstdint>
#include <cstddef>
#include <deque>
#include <mutex>

namespace aerotelem {

/**
 * Binary Telemetry Frame Layout (8 bytes, Big-Endian payload):
 * +--------+--------+----------+----------+-----------+-----------+--------+----------+
 * | Byte 0 | Byte 1 |  Byte 2  |  Byte 3  |  Byte 4   |  Byte 5   | Byte 6 |  Byte 7  |
 * +--------+--------+----------+----------+-----------+-----------+--------+----------+
 * |  0xAA  |  0x55  | Roll MSB | Roll LSB | Pitch MSB | Pitch LSB | Batt % | Checksum |
 * +--------+--------+----------+----------+-----------+-----------+--------+----------+
 * - Bytes 0..1 : Synchronization header (0xAA, 0x55)
 * - Bytes 2..3 : Roll angle in decidegrees (int16_t, MSB first)
 * - Bytes 4..5 : Pitch angle in decidegrees (int16_t, MSB first)
 * - Byte 6     : Battery charge percentage (uint8_t, 0..100%)
 * - Byte 7     : XOR checksum of payload bytes [2..6]
 */
constexpr uint8_t HEADER_BYTE_1 = 0xAA;
constexpr uint8_t HEADER_BYTE_2 = 0x55;
constexpr int FRAME_SIZE = 8;

constexpr int16_t MAX_SAFE_ANGLE_DECIDEG = 450; // +/- 45.0 deg
constexpr uint8_t LOW_BATTERY_THRESHOLD = 20;   // 20 %

struct TelemetryData {
    int16_t roll_decideg;   // Roll angle in decidegrees (e.g. -125 = -12.5 deg)
    int16_t pitch_decideg;  // Pitch angle in decidegrees (e.g. 340 = +34.0 deg)
    uint8_t battery_pct;    // Battery charge percentage (0..100%)
};

enum class SafetyStatus {
    NOMINAL,
    LOW_BATTERY,
    CRITICAL_ANGLE
};

// Complete thread-safe snapshot of the current telemetry state
struct TelemetrySnapshot {
    TelemetryData data;
    SafetyStatus status;
    uint32_t valid_frames;
};

// Binary frame encoding and decoding
void encode_frame(const TelemetryData& data, uint8_t* buffer);
bool decode_frame(const uint8_t* buffer, TelemetryData& out_data);

// Byte-by-byte UART stream state machine
class FrameParser {
public:
    enum class State {
        WAIT_HEADER_1,
        WAIT_HEADER_2,
        READ_PAYLOAD
    };

    FrameParser();
    bool push_byte(uint8_t byte, TelemetryData& out_data);

private:
    State state_;
    uint8_t buffer_[FRAME_SIZE];
    int index_;
};

// Real-time Moving Average Filter for sensor noise reduction
class MovingAverageFilter {
public:
    explicit MovingAverageFilter(size_t window_size);
    int16_t update(int16_t new_sample);
    void reset();

private:
    size_t window_size_;
    std::deque<int16_t> history_;
};

// High-level Telemetry Driver combining parsing, filtering, and flight safety monitoring
class TelemetryDriver {
public:
    explicit TelemetryDriver(size_t filter_window);
    bool process_byte(uint8_t byte);
    SafetyStatus evaluate_safety() const;
    const TelemetryData& latest_data() const;
    uint32_t valid_frames() const;

private:
    FrameParser parser_;
    MovingAverageFilter roll_filter_;
    MovingAverageFilter pitch_filter_;
    TelemetryData latest_data_;
    uint32_t valid_frames_;
};

// Thread-safe wrapper around TelemetryDriver for multi-threaded UART RX / ROS 2 publishing
class ThreadSafeTelemetry {
public:
    explicit ThreadSafeTelemetry(size_t filter_window);

    // Called by the high-frequency UART RX thread
    bool process_byte(uint8_t byte);

    // Called by the consumer / UI / ROS 2 publisher thread to get a consistent snapshot
    TelemetrySnapshot get_snapshot() const;

private:
    TelemetryDriver driver_;
    mutable std::mutex mtx_;
};

} // namespace aerotelem

#endif // TELEMETRY_HPP
