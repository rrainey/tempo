import re
import pyglet
from pyglet.gl import *
from pyrr import Quaternion, matrix44
import time

def parse_nmea_log(file_path):
    """
    Parses a log file containing NMEA sentences. Validates checksums and processes $PTH timestamps.
    
    Args:
        file_path (str): Path to the log file.
    
    Returns:
        tuple: A tuple containing:
            - List of parsed sentences (each a list of fields).
            - Count of rejected sentences due to checksum validation failure.
    """
    def calculate_checksum(nmea_sentence):
        """Calculates the NMEA checksum for a given sentence."""
        checksum = 0
        for char in nmea_sentence:
            checksum ^= ord(char)
        return f"{checksum:02X}"

    def validate_checksum(sentence):
        """Validates the NMEA checksum for a given sentence."""
        match = re.match(r"^\$(.*)\*(\w\w)$", sentence)
        if not match:
            return False
        content, checksum = match.groups()
        return calculate_checksum(content) == checksum

    parsed_sentences = []
    rejected_count = 0
    last_gpgga = None
    last_gpgll = None
    lastGroundTrack_degT = None
    lastBaroAlt_ft = None
    lastGNSSAlt_ft = None


    with open(file_path, "r") as file:
        for line in file:
            line = line.strip()
            if not validate_checksum(line):
                rejected_count += 1
                continue

            # Remove checksum for processing
            sentence_body = line[1:].split("*")[0]
            fields = sentence_body.split(",")

            if fields[0] == "GPGGA":
                last_gpgga = fields
            elif fields[0] == "GPGLL":
                last_gpgll = fields
            elif fields[0] == "PTH":
                if last_gpgga:
                    last_gpgga.append(fields[1])  # Append PTH timestamp
                if last_gpgll:
                    last_gpgll.append(fields[1])  # Append PTH timestamp
            else:
                parsed_sentences.append(fields)

            # Add GPGGA and GPGLL to results after processing PTH
            if fields[0] == "GPGGA" and last_gpgga:
                parsed_sentences.append(last_gpgga)
                last_gpgga = None
            if fields[0] == "GPGLL" and last_gpgll:
                parsed_sentences.append(last_gpgll)
                last_gpgll = None

    return parsed_sentences, rejected_count

# Example usage
#file_path = "your_nmea_log.txt"
#parsed_data, rejected = parse_nmea_log(file_path)
#print(f"Parsed data: {parsed_data}")
#print(f"Rejected sentences due to checksum failure: {rejected}")

# Adjust this path to your GLTF model
GLTF_MODEL_PATH = "./model.gltf"

class NMEA3DRenderer:
    def __init__(self, parsed_data, model_path, playback_speed=1.0):
        self.data = parsed_data
        self.model_path = model_path
        self.playback_speed = playback_speed
        self.start_time = None
        self.current_index = 0
        
        # Load glTF model (stubbed for simplicity, requires proper glTF loader)
        self.gltf_model = pyglet.graphics.Batch()

        # Set up viewer's perspective
        self.view_matrix = matrix44.create_look_at(
            eye=[-20, 0, 0], target=[0, 0, 0], up=[0, 0, 1]
        )
        
        # Display window
        self.window = pyglet.window.Window(800, 600, "NMEA 3D Renderer", resizable=True)
        self.window.on_draw = self.on_draw

    def on_draw(self):
        self.window.clear()
        glLoadIdentity()
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(65, self.window.width / self.window.height, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)
        glLoadMatrixf(self.view_matrix.flatten())
        
        # Render model at the origin with the current quaternion rotation
        if self.start_time is None:
            self.start_time = time.time()

        current_time = (time.time() - self.start_time) * 1000 * self.playback_speed
        while self.current_index < len(self.data):
            record = self.data[self.current_index]
            timestamp = int(record[-1])  # Assuming timestamp is last field
            if timestamp > current_time:
                break
            
            if record[0] == "PIM2":
                quaternion = Quaternion([float(record[2]), float(record[3]), float(record[4]), float(record[1])])
                rotation_matrix = matrix44.create_from_quaternion(quaternion)
                glMultMatrixf(rotation_matrix.flatten())
            
            if record[0] == "GPGLL" or record[0] == "GNGLL":
                altitude = float(record[2]) if record[0] == "PENV" else float(record[9])
                self.draw_altitude_text(altitude)

            if record[0] == "PENV":
                altitude = float(record[2]) if record[0] == "PENV" else float(record[9])
                self.draw_altitude_text(altitude)

            self.current_index += 1

        # Draw the model (stubbed, replace with your glTF rendering logic)
        self.gltf_model.draw()

    def draw_altitude_text(self, altitude, groundTrack_degT):
        """Draw altitude information on the window."""
        label = pyglet.text.Label(
            f"Altitude: {altitude} meters",
            font_name="Arial",
            font_size=12,
            x=10, y=10,
            anchor_x="left", anchor_y="bottom",
            color=(255, 255, 255, 128)  # Semi-transparent white
        )
        label.draw()

    def run(self):
        pyglet.app.run()


# Example usage with parsed NMEA data
if __name__ == "__main__":
    parsed_data = [
        # Example parsed sentences
        ["PIM2", "13925127", "1.0000", "0.0000", "0.0000", "0.0000"],
        ["PENV", "13925040", "984.62", "791.18", "3.79"],
        ["GPGLL", "5248.312,N", "10010.876,W", "1234.56", "A", "13925296"],
    ]
    renderer = NMEA3DRenderer(parsed_data, GLTF_MODEL_PATH, playback_speed=1.0)
    renderer.run()
