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

// default time unit driving the output rate (milliseconds)
#define MORSE_DEFAULT_TIME_UNIT_MS 250

/*
 * Flash an LED, repeating a given Morse Code character.
 */
class MorseBlinker {

public:
    MorseBlinker() {
        unitTime_ms = MORSE_DEFAULT_TIME_UNIT_MS;
        reset();
        ledPin = -1;
    }

    // Initialization function to set the LED pin and basic time unit
    void initialize(int pin, unsigned long timeUnit = MORSE_DEFAULT_TIME_UNIT_MS) {
        ledPin = pin;
        unitTime_ms = timeUnit;
        pinMode(ledPin, OUTPUT);
        reset();
    }

    // Set the output character and update Morse code sequence
    void setOutputCharacter(char character) {
        reset();
        outputChar = character;
        // Retrieve the Morse code sequence based on character
        morseSequence = getMorseCode(character);
        // include the trailing NULL character as part of the sequence we consider; see below
        sequenceLength = strlen(morseSequence) + 1;

        updateLEDState();
    }

    // Function to be called from Arduino loop
    void loop() {
        if (ledPin >= 0) {
            unsigned long currentTime_ms = millis();

            // If we're between blinks, check if it's time to move to the next
            // element
            if (currentTime_ms - lastUpdate_ms >= thisInterval_ms) {

                lastUpdate_ms = currentTime_ms;

                if (isOn) {
                    isOn = false;
                    digitalWrite(ledPin, LOW);
                    thisInterval_ms = unitTime_ms;
                } else {
                    updateLEDState();
                }
            }
        }
    }

protected:
    // Arduino pin # for output
    int ledPin;
    // indicates if we are at the beginning or end of a bit's output
    bool isOn = false;
    // lowest level time interval for the Morse Code output.
    // A dot is one interval. A dash is three intervals.
    unsigned long unitTime_ms;
    // ASCII encoding of the output character
    char outputChar = ' ';
    // encoded Morse code sequence for the output character
    const char * morseSequence;
    // number of dot/dash elements in the character's Morse code sequence
    int sequenceLength = 0;
    // currently executing dot/dash bit in the sequence
    int currentIndex = 0;
    // target duration of the current on/off interval
    unsigned long thisInterval_ms = 0;
    // millis() time of last update
    unsigned long lastUpdate_ms = 0;

    // Resets the state of the blinker
    void reset() {
        digitalWrite(ledPin, LOW);
        isOn = false;
        currentIndex = 0;
        thisInterval_ms = 0; 
        lastUpdate_ms = millis();
    }

    void updateLEDState() {
        if (currentIndex >= sequenceLength) {
            reset();
            morseSequence = getMorseCode(outputChar);
            return;
        }

        // Determine the delay based on Morse code character
        char morseChar = morseSequence[currentIndex++];
        if (morseChar == '.') {
            digitalWrite(ledPin, HIGH);
            thisInterval_ms = unitTime_ms;  // Morse DOT
            isOn = true;
        } else if (morseChar == '-') {
            digitalWrite(ledPin, HIGH);
            thisInterval_ms = 3 * unitTime_ms;  // Dash
            isOn = true;
        } else {  // NULL - end of character
            digitalWrite(ledPin, LOW);
            thisInterval_ms = 7 * unitTime_ms;  // inter-character gap
        }
    }

    // Returns the Morse code sequence for a given character
    const char * getMorseCode(char character) {
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
            default: return "";  // space for no output
            case ' ': return "";  // space for no output
        }
    }
};

#endif // MORSEBLINKER_H