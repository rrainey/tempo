import sys
import re

'''
Fix a bug affecting $PTH records in the a text version of 156 Tempo logs.

This application renders a corrected version of two log files tha5 were generated during
two test jumps with the original 156 software. A fix has been committed to the temp-logger 
application.

The $PTH record value was incorrectly being output as millis() rather than the
offset time from the start of the log file. This fix estimates a corrected value.
Two files needed to be corrected. The correction values for A were estimated by visually inspecting
the file for adjacent $PTH and $PIMU records and calculating the difference between the two values.

Value of A for LOG004 : 249277
Value of A for LOG005 : 247504
'''

def calculate_checksum(sentence):
    """Calculate the NMEA checksum for a given sentence."""
    checksum = 0
    for char in sentence:
        checksum ^= ord(char)
    return f"{checksum:02X}"

def process_stream(input_stream, A):
    """
    Process NMEA sentences from standard input.
    
    Args:
        input_stream: The input stream (e.g., sys.stdin).
        A: Integer to subtract from the numeric value in `$PTH` sentences.
    """
    for line in input_stream:
        line = line.strip()

        # Match NMEA sentence pattern
        match = re.match(r"^\$(.*)\*(\w\w)$", line)
        if not match:
            print(line)  # Output non-matching lines unmodified
            continue

        sentence_body, checksum = match.groups()

        if sentence_body.startswith("PTH"):
            fields = sentence_body.split(",")
            try:
                numeric_value = int(fields[1])
                modified_value = numeric_value - A
                fields[1] = str(modified_value)

                # Reconstruct the sentence and calculate new checksum
                modified_body = ",".join(fields)
                new_checksum = calculate_checksum(modified_body)

                # Output the modified sentence
                print(f"${modified_body}*{new_checksum}")
            except ValueError:
                print(line)  # Output the original line if parsing fails
        else:
            # Output non-$PTH sentences unmodified
            print(line)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python fix-156.py <integer_A>")
        sys.exit(1)

    try:
        A = int(sys.argv[1])
    except ValueError:
        print("Error: <integer_A> must be an integer.")
        sys.exit(1)

    process_stream(sys.stdin, A)
