#include <iostream>
#include <iomanip>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include "telemetry.hpp"

using namespace aerotelem;

int main() {
    TelemetryData input = {-125, 340, 87};
    uint8_t frame[FRAME_SIZE] = {0};

    std::cout << "=== [1/5] Binary Frame Encoder / Decoder Test ===" << std::endl;
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

    std::cout << "\n=== [2/5] UART Stream State Machine (FrameParser) Test ===" << std::endl;
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

    std::cout << "\n=== [3/5] Moving Average Filter Test ===" << std::endl;
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

    std::cout << "\n=== [4/5] TelemetryDriver Integration & Flight Safety Test ===" << std::endl;
    TelemetryDriver driver(2);

    auto send_telemetry = [&](const TelemetryData& d) {
        uint8_t raw[FRAME_SIZE];
        encode_frame(d, raw);
        for (int i = 0; i < FRAME_SIZE; ++i) {
            driver.process_byte(raw[i]);
        }
    };

    send_telemetry({100, -200, 80});
    if (driver.valid_frames() != 1 || driver.evaluate_safety() != SafetyStatus::NOMINAL) {
        std::cout << "[FAIL] Frame 1 nominal state check failed." << std::endl;
        return 1;
    }

    send_telemetry({140, -100, 15});
    if (driver.latest_data().roll_decideg != 120 ||
        driver.latest_data().pitch_decideg != -150 ||
        driver.evaluate_safety() != SafetyStatus::LOW_BATTERY) {
        std::cout << "[FAIL] Frame 2 filtering or LOW_BATTERY check failed." << std::endl;
        return 1;
    }

    send_telemetry({-900, 1100, 15});
    if (driver.evaluate_safety() != SafetyStatus::CRITICAL_ANGLE) {
        std::cout << "[FAIL] Frame 3 CRITICAL_ANGLE priority check failed." << std::endl;
        return 1;
    }
    std::cout << "[PASS] TelemetryDriver pipeline & safety monitor verified (" << driver.valid_frames() << " frames processed)." << std::endl;

    std::cout << "\n=== [5/5] Multi-Threaded UART Acquisition & Consumer Test ===" << std::endl;
    ThreadSafeTelemetry shared_telem(4);
    std::atomic<bool> producer_done{false};

    // Thread 1 (Producer): Simulates high-rate UART reception (20 frames sent sequentially)
    std::thread uart_rx_thread([&]() {
        for (int k = 1; k <= 20; ++k) {
            uint8_t raw[FRAME_SIZE];
            encode_frame({static_cast<int16_t>(k * 10), static_cast<int16_t>(-k * 5), 75}, raw);
            for (int b = 0; b < FRAME_SIZE; ++b) {
                shared_telem.process_byte(raw[b]);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        producer_done = true;
    });

    // Thread 2 (Consumer): Concurrently reads snapshots while Thread 1 is writing
    TelemetrySnapshot last_snap{{0, 0, 0}, SafetyStatus::NOMINAL, 0};
    int snapshots_read = 0;
    while (!producer_done) {
        last_snap = shared_telem.get_snapshot();
        snapshots_read++;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    uart_rx_thread.join(); // Wait for UART RX thread to finish cleanly
    last_snap = shared_telem.get_snapshot();

    // Last 4 frames sent were k=17, 18, 19, 20 -> Roll = 170, 180, 190, 200 -> avg = 185
    if (last_snap.valid_frames != 20 || last_snap.data.roll_decideg != 185 || last_snap.status != SafetyStatus::NOMINAL) {
        std::cout << "[FAIL] Multi-threaded telemetry test failed (valid_frames=" << last_snap.valid_frames
                  << ", roll=" << last_snap.data.roll_decideg << ")." << std::endl;
        return 1;
    }

    std::cout << "[PASS] Multi-threaded acquisition verified (" << last_snap.valid_frames
              << " frames received, " << snapshots_read << " concurrent snapshots read safely)." << std::endl;
    std::cout << "\nAll AeroTelem test suites passed successfully." << std::endl;
    return 0;
}
