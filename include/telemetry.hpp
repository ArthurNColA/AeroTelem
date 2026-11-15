#ifndef TELEMETRY_HPP
#define TELEMETRY_HPP

#include <cstdint>
#include <cstddef>
#include <deque>

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

struct TelemetryData {
    int16_t roll_decideg;   // Angle Roll en dixièmes de degré (ex: -125 = -12.5°)
    int16_t pitch_decideg;  // Angle Pitch en dixièmes de degré (ex: 340 = +34.0°)
    uint8_t battery_pct;    // Pourcentage de batterie (0 à 100 %)
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
    /**
     * Constructeur : définit la taille de la fenêtre glissante (ex: 5 échantillons).
     */
    explicit MovingAverageFilter(size_t window_size);

    /**
     * Ajoute une nouvelle mesure brute dans le filtre et renvoie la moyenne lissée.
     */
    int16_t update(int16_t new_sample);

    /**
     * Réinitialise (vide) l'historique du filtre.
     */
    void reset();

private:
    size_t window_size_;            // Nombre max d'échantillons à garder en mémoire
    std::deque<int16_t> history_;   // File contenant les derniers échantillons reçus
};

} // namespace aerotelem

#endif // TELEMETRY_HPP
