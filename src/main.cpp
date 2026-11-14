#include <iostream>
#include <iomanip>
#include <vector>
#include "telemetry.hpp"

using namespace aerotelem;

int main() {
    TelemetryData input = {-125, 340, 87}; // Roll = -12.5°, Pitch = +34.0°, Batt = 87%
    uint8_t frame[FRAME_SIZE] = {0};

    std::cout << "=== SPRINT 1 : TEST ENCODEUR / DECODEUR ===" << std::endl;
    encode_frame(input, frame);

    TelemetryData decoded = {0, 0, 0};
    if (decode_frame(frame, decoded) && decoded.roll_decideg == -125 && decoded.pitch_decideg == 340 && decoded.battery_pct == 87) {
        std::cout << "[OK] Test 1 & 2 (Sprint 1) : Encodeur/Decodeur OK !" << std::endl;
    } else {
        std::cout << "[ERREUR] Sprint 1 en echec." << std::endl;
        return 1;
    }

    std::cout << "\n=== SPRINT 2 : TEST MACHINE A ETATS (FLUX UART BRUITE) ===" << std::endl;
    // Préparation de 2 vraies trames
    uint8_t frame1[FRAME_SIZE];
    uint8_t frame2[FRAME_SIZE];
    encode_frame({-50, 120, 95}, frame1);   // Trame 1 : Roll=-5.0°, Pitch=+12.0°, Batt=95%
    encode_frame({210, -85, 42}, frame2);   // Trame 2 : Roll=+21.0°, Pitch=-8.5°, Batt=42%

    // Simulation d'un câble série UART avec des octets parasites (bruit) et un faux départ (0xAA suivi de 0x12)
    std::vector<uint8_t> uart_stream = {
        0x00, 0xFF, 0x42,       // Bruit au démarrage du drone
        0xAA, 0x12, 0x99,       // Faux départ ! (0xAA mais pas suivi de 0x55)
    };
    // On insère la vraie Trame 1 octet par octet
    for (int i = 0; i < FRAME_SIZE; ++i) uart_stream.push_back(frame1[i]);
    // Un peu de bruit entre les deux trames
    uart_stream.push_back(0x77);
    uart_stream.push_back(0x88);
    // On insère la vraie Trame 2 octet par octet
    for (int i = 0; i < FRAME_SIZE; ++i) uart_stream.push_back(frame2[i]);

    FrameParser parser;
    int valid_frames_count = 0;
    TelemetryData rx_data = {0, 0, 0};

    // On envoie les octets UN PAR UN au parser, exactement comme une interruption UART sur STM32 !
    for (size_t i = 0; i < uart_stream.size(); ++i) {
        if (parser.push_byte(uart_stream[i], rx_data)) {
            valid_frames_count++;
            std::cout << "  -> Trame #" << valid_frames_count << " detectee a l'octet " << i
                      << " : Roll=" << rx_data.roll_decideg
                      << ", Pitch=" << rx_data.pitch_decideg
                      << ", Batt=" << static_cast<int>(rx_data.battery_pct) << "%" << std::endl;
        }
    }

    if (valid_frames_count == 2 && rx_data.roll_decideg == 210 && rx_data.pitch_decideg == -85 && rx_data.battery_pct == 42) {
        std::cout << "[OK] Test 3 REUSSI : Ta machine a etats a filtre le bruit et extrait les 2 trames !" << std::endl;
        std::cout << "\nBravo Arthur ! Le Sprint 2 (POO & Machine a etats) est 100% valide !" << std::endl;
        return 0;
    } else {
        std::cout << "[ERREUR] Test 3 ECHOUE : Trames valides trouvees = " << valid_frames_count << " (attendu : 2)" << std::endl;
        return 1;
    }
}
