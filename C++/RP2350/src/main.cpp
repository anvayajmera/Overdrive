#include <Arduino.h>
#include "motor.hpp"

// Define motors with your specific pins
Motor motors[4] = {
    Motor(D2, D1),   // Motor 1 (Left Front)
    Motor(D4, D3),   // Motor 2 (Right Front)
    Motor(D8, D7),   // Motor 3 (Left Rear)
    Motor(D10, D9)   // Motor 4 (Right Rear)
};

// Helper to stop everything
void allStop() {
    for(int i = 0; i < 4; i++) motors[i].stop();
}

void setup() {
    for(int i = 0; i < 4; i++) {
        motors[i].setup();
    }
    
    // PHYSICAL CALIBRATION:
    // Flip Motors 1 and 3 because they are mounted on the opposite side.
    // This makes 'setSpeed' move all wheels in the same physical direction.
    motors[0].reverse(); 
    motors[2].reverse();
}

void loop() {
    // // --- PHASE 1: FORWARD FULL SPEED ---
    // for(int i = 0; i < 4; i++) {
    //     motors[i].setSpeed(100);
    // }
    // delay(10000); 

    // // --- PHASE 2: SHORT STOP ---
    // allStop();
    // delay(500);

    // // --- PHASE 3: SPIN (Point Turn) ---
    // // To spin, we want the Left side to go Backward.
    // // Since they are currently in "Forward Mode", we toggle reverse() 
    // // to put them in "Backward Mode".
    // motors[0].reverse(); 
    // motors[2].reverse();

    // for(int i = 0; i < 4; i++) {
    //     motors[i].setSpeed(100);
    // }
    // delay(2000); 

    // // --- PHASE 4: STOP AND RESET ---
    // allStop();
    // delay(500);

    // // Flip 1 and 3 back to "Forward Mode" for the next loop start
    // motors[0].reverse(); 
    // motors[2].reverse();
}