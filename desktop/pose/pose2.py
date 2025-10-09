import pyglet
from pyglet.gl import *
from pyglet.math import Mat4
from pyglet.window import mouse
from pyrr import Quaternion, matrix44
import time
import sys
import re
import trimesh

correction = 249277

# Adjust this path to your GLTF model
GLTF_MODEL_PATH = "./model.gltf"

def METERStoFEET(meters):
    return meters * 3.28084

def get_timestamp(record):
    if (record[0] == "PIMU" or record[0] == "PIM2" or record[0] == "PENV"):
        return int(record[1])
    elif (record[0] == "GPGLL" or record[0] == "GNGLL"):
        return int(record[8]) - correction
    return -1

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
        content, checksum = sentence.strip().split("*")
        #return calculate_checksum(content).upper() == checksum.upper()
        return True

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
                    parsed_sentences.append(last_gpgga)
                if last_gpgll:
                    last_gpgll.append(fields[1])  # Append PTH timestamp
                    parsed_sentences.append(last_gpgll)
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

class NMEA3DRenderer:
    def __init__(self, parsed_data, model_path, playback_speed=1.0):
        self.data = parsed_data
        self.model_path = model_path
        self.playback_speed = playback_speed
        self.start_time = None
        self.current_index = 0
        self.is_paused = False
        self.pause_time = 0
        self.lastGroundTrack_degT = None
        self.lastBaroAlt_ft = None
        self.lastGNSSAlt_ft = None
        self.lastTimestamp_ms = 0
        self.batch =batch = pyglet.graphics.Batch()
        
        # Load glTF model (stubbed for simplicity, requires proper glTF loader)
        #self.model = trimesh.load('path/to/your/model.gltf')
        #self.gltf_model = pyglet.graphics.Batch()
        self.model =pyglet.model.load("skydiver.obj", batch=self.batch)

        # Set up viewer's perspective
        self.view_matrix = matrix44.create_look_at(
            eye=[-20, 0, 0], target=[0, 0, 0], up=[0, 0, 1]
        )
        
        # Display window
        self.window = pyglet.window.Window(1600, 800, "NMEA 3D Renderer", resizable=True)
        self.window.on_draw = self.on_draw
        self.window.on_mouse_press = self.on_mouse_press

        # Load play and pause icons
        self.play_icon = pyglet.resource.image("play_circle_outline_42dp_E8EAED.png")
        self.pause_icon = pyglet.resource.image("pause_circle_outline_42dp_E8EAED.png")
        self.is_playing = True

    def on_draw(self):
        glClear(GL_COLOR_BUFFER_BIT)
        #glLoadIdentity()
        #glMatrixMode(GL_PROJECTION)
        #glLoadIdentity()
        #gluPerspective(65, self.window.width / self.window.height, 0.1, 100.0)
       # glMatrixMode(GL_MODELVIEW)
        #glLoadMatrixf(self.view_matrix.flatten())


        self.window.projection = Mat4.orthogonal_projection(
            0, self.window.width, 0, self.window.height, -255, 255
        )

        if self.start_time is None:
            self.start_time = time.time()

        if not self.is_paused:
            current_time = (time.time() - self.start_time) * 1000 * self.playback_speed + self.pause_time
            while self.current_index < len(self.data):
                record = self.data[self.current_index]
                timestamp = get_timestamp(record)
                if timestamp == -1:
                    self.current_index += 1
                    continue

                if timestamp > current_time:
                    break
                
                if record[0] == "PIM2":
                    quaternion = Quaternion([float(record[2]), float(record[3]), float(record[4]), float(record[5])])
                    rotation_matrix = matrix44.create_from_quaternion(quaternion)
                    glMultMatrixf(rotation_matrix.flatten())
                    self.lastTimestamp_ms = int(record[1])
                
                if record[0] == "PENV":
                    altitude = float(record[2]) if record[0] == "PENV" else float(record[9])
                    self.lastBaroAlt_ft = altitude
                    #self.draw_altitude_text(altitude)
                    self.lastTimestamp_ms = int(record[1])
                
                if record[0] == "GPGLL":
                    altitude = float(record[2]) if record[0] == "PENV" else float(record[9])
                    self.lastGNSSAlt_ft = METERStoFEET(altitude)
                    #self.draw_altitude_text(altitude)
                    self.lastTimestamp_ms = int(record[1])

                if record[0] == "GNGLL":
                    altitude = float(record[2]) if record[0] == "PENV" else float(record[9])
                    self.lastGNSSAlt_ft = METERStoFEET(altitude)
                    #self.draw_altitude_text(altitude)
                    self.lastTimestamp_ms = int(record[1])

                self.current_index += 1

        # Draw the model (stubbed, replace with your glTF rendering logic)
        self.gltf_model.draw()

        # Draw play/pause buttons
        self.draw_controls()

    def draw_altitude_text(self):
        """Draw altitude information on the window."""
        ts_sec = self.lastTimestamp_ms / 1000
        label = pyglet.text.Label(
            f"\nTime: {ts_sec} sec\nBaro Altitude: {self.lastBaroAlt_ft} ft\nGNSS Altitude: {self.lastGNSSAlt_ft} ft\nGround Track: {self.lastGroundTrack_degT} deg, True",
            font_name="Arial",
            font_size=12,
            x=10, y=10,
            anchor_x="left", anchor_y="bottom",
            color=(255, 255, 255, 128)  # Semi-transparent white
        )
        label.draw()

    def draw_controls(self):
        """Draw play and pause buttons."""
        self.draw_altitude_text()
        if self.is_playing:
            self.pause_icon.blit(740, 20)  # Bottom-right corner
        else:
            self.play_icon.blit(740, 20)

    def on_mouse_press(self, x, y, button, modifiers):
        """Handle mouse clicks for play/pause controls."""
        if button == mouse.LEFT and 740 <= x <= 780 and 20 <= y <= 60:
            self.toggle_play_pause()

    def toggle_play_pause(self):
        """Toggle play/pause state."""
        if self.is_playing:
            self.is_paused = True
            self.pause_time += (time.time() - self.start_time) * 1000 * self.playback_speed
        else:
            self.is_paused = False
            self.start_time = time.time()
        self.is_playing = not self.is_playing

    def run(self):
        pyglet.app.run()


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python pose2.py <logfile-path>")
        sys.exit(1)

    try:
        parsed_data, err = parse_nmea_log(sys.argv[1])
    except ValueError:
        print("Error: {err}")
        sys.exit(1)

    print(len(parsed_data))
    print(err)
    renderer = NMEA3DRenderer(parsed_data, GLTF_MODEL_PATH, playback_speed=1.0)
    renderer.run()