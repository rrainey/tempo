/* 
 * This file is part of the Tempo distribution (https://github.com/rrainey/tempo)
 * Copyright (c) 2024 Riley Rainey
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MORSEBLINKER_H
#define MORSEBLINKER_H

#include <Arduino.h>

/*
 * Flash an LED, repeating a given Morse Code character.
 */
class MorseBlinker {

public:
    MorseBlinker() {
        unitTime = 250;
        reset();
    }

    // Initialize function to set the LED pin and basic time unit
    void initialize(int pin, unsigned long timeUnit = 100) {
        ledPin = pin;
        unitTime = timeUnit;
        pinMode(ledPin, OUTPUT);
        reset();
    }

    // Set the output character and update Morse code sequence
    void setOutputCharacter(char character) {
        reset();
        outputChar = character;
        // Retrieve the Morse code sequence based on character
        morseSequence = getMorseCode(character);
        sequenceLength = morseSequence.length();
    }

    // Function to be called from Arduino loop
    void loop() {
        unsigned long currentMillis = millis();
        
        // If we're between blinks, check if it's time to move to the next element
        if (currentMillis - lastUpdate >= currentDelay) {
            lastUpdate = currentMillis;
            
            // If we're at the end of the sequence, reset
            if (currentIndex >= sequenceLength) {
                reset();
                return;
            }

            // Determine the delay based on Morse code character
            char morseChar = morseSequence[currentIndex++];
            if (morseChar == '.') {
                digitalWrite(ledPin, HIGH);
                currentDelay = unitTime;  // dot duration
            } else if (morseChar == '-') {
                digitalWrite(ledPin, HIGH);
                currentDelay = 3 * unitTime;  // dash duration
            } else { // gap
                digitalWrite(ledPin, LOW);
                currentDelay = unitTime;  // inter-symbol gap
            }

            // Turn off LED after dot or dash
            if (currentIndex < sequenceLength) {
                digitalWrite(ledPin, LOW);
                lastUpdate += currentDelay;
                currentDelay = unitTime;  // gap between elements
            }
        }
    }

protected:
    int ledPin;
    unsigned long unitTime;
    char outputChar = ' ';
    String morseSequence;
    int sequenceLength = 0;
    int currentIndex = 0;
    unsigned long currentDelay = 0;
    unsigned long lastUpdate = 0;

    // Resets the state of the blinker
    void reset() {
        digitalWrite(ledPin, LOW);
        currentIndex = 0;
        currentDelay = unitTime * 3;  // inter-character gap
        lastUpdate = millis();
    }

    // Returns the Morse code sequence for a given character
    String getMorseCode(char character) {
        switch (toupper(character)) {
            case 'A': return ".-";
            case 'B': return "-...";
            case 'C': return "-.-.";
            case 'D': return "-..";
            case 'E': return ".";
            case 'F': return "..-.";
            case 'G': return "--.";
            case 'H': return "....";
            case 'I': return "..";
            case 'J': return ".---";
            case 'K': return "-.-";
            case 'L': return ".-..";
            case 'M': return "--";
            case 'N': return "-.";
            case 'O': return "---";
            case 'P': return ".--.";
            case 'Q': return "--.-";
            case 'R': return ".-.";
            case 'S': return "...";
            case 'T': return "-";
            case 'U': return "..-";
            case 'V': return "...-";
            case 'W': return ".--";
            case 'X': return "-..-";
            case 'Y': return "-.--";
            case 'Z': return "--..";
            case '1': return ".----";
            case '2': return "..---";
            case '3': return "...--";
            case '4': return "....-";
            case '5': return ".....";
            case '6': return "-....";
            case '7': return "--...";
            case '8': return "---..";
            case '9': return "----.";
            case '0': return "-----";
            case ' ': return "";  // space for no output
        }
    }
};

#endif // MORSEBLINKER_H