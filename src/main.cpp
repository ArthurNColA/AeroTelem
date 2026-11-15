#include <iostream>
#include <iomanip>
#include <vector>
#include "telemetry.hpp"

using namespace aerotelem;

int main() {
    TelemetryData input = {-125, 340, 87};
    uint8_t frame[FRAME_SIZE] = {0};

    std::cout << "=== [1/3] Binary Frame Encoder / Decoder Test ===" << std::endl;
    encode_frame(input, frame);

    TelemetryData decoded = {0, 0, 0};
    if (!decode_frame(frame, decoded) || decoded.roll_decideg != -125 || decoded.pitch_decideg != 340 || decoded.battery_pct != 87) {
        std::cout << "[FAIL] Valid frame decode test failed." << std::endl;
        return 1;
    }

    frame[4] ^= 0x01; // Inject bit error in payload
    if (decode_frame(frame, decoded)) {
        std::cout << "[FAIL] Corrupted frame was not rejected by XOR checksum." << std::endl;
        return 1;
    }
    std::cout << "[PASS] Binary protocol encode/decode & XOR checksum verified." << std::endl;

    std::cout << "\n=== [2/3] UART Stream State Machine (FrameParser) Test ===" << std::endl;
    uint8_t frame1[FRAME_SIZE];
    uint8_t frame2[FRAME_SIZE];
    encode_frame({-50, 120, 95}, frame1);
    encode_frame({210, -85, 42}, frame2);

    std::vector<uint8_t> uart_stream = {
        0x00, 0xFF, 0x42,
        0xAA, 0x12, 0x99,
    };
    for (int i = 0; i < FRAME_SIZE; ++i) uart_stream.push_back(frame1[i]);
    uart_stream.push_back(0x77);
    uart_stream.push_back(0x88);
    for (int i = 0; i < FRAME_SIZE; ++i) uart_stream.push_back(frame2[i]);

    FrameParser parser;
    int valid_frames_count = 0;
    TelemetryData rx_data = {0, 0, 0};

    for (size_t i = 0; i < uart_stream.size(); ++i) {
        if (parser.push_byte(uart_stream[i], rx_data)) {
            valid_frames_count++;
        }
    }

    if (valid_frames_count != 2 || rx_data.roll_decideg != 210 || rx_data.pitch_decideg != -85 || rx_data.battery_pct != 42) {
        std::cout << "[FAIL] State machine parser test failed." << std::endl;
        return 1;
    }
    std::cout << "[PASS] Noisy UART byte-stream state machine verified (2/2 frames extracted)." << std::endl;

    std::cout << "\n=== [3/3] Moving Average Filter Test ===" << std::endl;
    MovingAverageFilter roll_filter(4);
    std::vector<int16_t> noisy_samples = {100, 120, 80, 100, 140};
    int16_t last_filtered = 0;

    for (size_t i = 0; i < noisy_samples.size(); ++i) {
        last_filtered = roll_filter.update(noisy_samples[i]);
        std::cout << "  Sample #" << (i + 1) << " = " << std::setw(4) << noisy_samples[i]
                  << "  ->  Filtered (N=4) = " << last_filtered << std::endl;
    }

    if (last_filtered == 110) {
        roll_filter.reset();
        if (roll_filter.update(50) == 50) {
            std::cout << "[PASS] MovingAverageFilter verified." << std::endl;
            std::cout << "\nAll AeroTelem test suites passed successfully." << std::endl;
            return 0;
        }
    }

    std::cout << "[FAIL] MovingAverageFilter test failed (got " << last_filtered << ", expected 110)." << std::endl;
    return 1;
}
