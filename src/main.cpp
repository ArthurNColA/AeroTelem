#include <iostream>
#include <iomanip>
#include <vector>
#include "telemetry.hpp"

using namespace aerotelem;

int main() {
    TelemetryData input = {-125, 340, 87};
    uint8_t frame[FRAME_SIZE] = {0};

    std::cout << "=== [1/4] Binary Frame Encoder / Decoder Test ===" << std::endl;
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

    std::cout << "\n=== [2/4] UART Stream State Machine (FrameParser) Test ===" << std::endl;
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

    std::cout << "\n=== [3/4] Moving Average Filter Test ===" << std::endl;
    MovingAverageFilter roll_filter(4);
    std::vector<int16_t> noisy_samples = {100, 120, 80, 100, 140};
    int16_t last_filtered = 0;

    for (size_t i = 0; i < noisy_samples.size(); ++i) {
        last_filtered = roll_filter.update(noisy_samples[i]);
    }

    if (last_filtered != 110) {
        std::cout << "[FAIL] MovingAverageFilter test failed (got " << last_filtered << ", expected 110)." << std::endl;
        return 1;
    }
    roll_filter.reset();
    if (roll_filter.update(50) != 50) {
        std::cout << "[FAIL] MovingAverageFilter reset test failed." << std::endl;
        return 1;
    }
    std::cout << "[PASS] MovingAverageFilter verified." << std::endl;

    std::cout << "\n=== [4/4] TelemetryDriver Integration & Flight Safety Test ===" << std::endl;
    TelemetryDriver driver(2); // Sliding window of 2 samples

    auto send_telemetry = [&](const TelemetryData& d) {
        uint8_t raw[FRAME_SIZE];
        encode_frame(d, raw);
        for (int i = 0; i < FRAME_SIZE; ++i) {
            driver.process_byte(raw[i]);
        }
    };

    // Frame 1: Roll = 100, Pitch = -200, Batt = 80% -> Nominal
    send_telemetry({100, -200, 80});
    if (driver.valid_frames() != 1 || driver.evaluate_safety() != SafetyStatus::NOMINAL) {
        std::cout << "[FAIL] Frame 1 nominal state check failed." << std::endl;
        return 1;
    }

    // Frame 2: Roll = 140 (avg with 100 = 120), Pitch = -100 (avg with -200 = -150), Batt = 15% -> Low Battery
    send_telemetry({140, -100, 15});
    if (driver.latest_data().roll_decideg != 120 ||
        driver.latest_data().pitch_decideg != -150 ||
        driver.evaluate_safety() != SafetyStatus::LOW_BATTERY) {
        std::cout << "[FAIL] Frame 2 filtering or LOW_BATTERY check failed." << std::endl;
        return 1;
    }

    // Frame 3: Roll = -900 (avg with 140 = -380), Pitch = 1100 (avg with -100 = +500 > 450), Batt = 15% -> Critical Angle
    send_telemetry({-900, 1100, 15});
    if (driver.evaluate_safety() != SafetyStatus::CRITICAL_ANGLE) {
        std::cout << "[FAIL] Frame 3 CRITICAL_ANGLE priority check failed." << std::endl;
        return 1;
    }

    std::cout << "[PASS] TelemetryDriver pipeline & safety monitor verified (" << driver.valid_frames() << " frames processed)." << std::endl;
    std::cout << "\nAll AeroTelem test suites passed successfully." << std::endl;
    return 0;
}
