#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <cstdint>

namespace aerotelem {

constexpr uint8_t HEADER_BYTE_1 = 0xAA;
constexpr uint8_t HEADER_BYTE_2 = 0x55;
constexpr int FRAME_SIZE = 8;

struct TelemetryData {
    int16_t roll_decideg;   // Angle Roll en dixièmes de degré (ex: -125 = -12.5°)
    int16_t pitch_decideg;  // Angle Pitch en dixièmes de degré (ex: 340 = +34.0°)
    uint8_t battery_pct;    // Pourcentage de batterie (0 à 100 %)
};

void encode_frame(const TelemetryData& data, uint8_t* buffer);
bool decode_frame(const uint8_t* buffer, TelemetryData& out_data);

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

} // namespace aerotelem

#endif // TELEMETRY_HPP
