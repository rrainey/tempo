from PyQt5 import QtWidgets, QtOpenGL, QtCore
from PyQt5.QtCore import Qt
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutBitmapCharacter, GLUT_BITMAP_HELVETICA_18, glutBitmapString
from pyrr import Quaternion, matrix44
import time
import logging
import sys
import numpy as np
import pywavefront
from pywavefront import visualization
import functools

# Early log files had an issue with the PTH timestamp offset which required manual correction.
# Current log files will not have this issue.
correction = 0

# We'll pre-scan the log file and note the timestamps of events that correspond to exit, deployment, and landing.
# Markers are placed on the timeline slider to indicate these events.

gevents = []

parsed_data = None

# Configuration
WINDOW_WIDTH = 1000
WINDOW_HEIGHT = 1000
MODEL_PATH = "freefall.obj"
VIEWER_POSITION = [-20, 0, 0]
LIGHT_DISTANCE = 20
AXIS_LINE_LENGTH = 4
max_time_sec = 0.0


def METERStoFEET(meters):
    return meters * 3.28084

def KNOTStoMPH(knots):
    return knots * 1.15078

# return millis() timestamp of the record
def get_timestamp(record):
    if (record[0] == "PIMU" or record[0] == "PIM2" or record[0] == "PENV"):
        return int(record[1])
    elif (record[0] == "GPGLL" or record[0] == "GNGLL"):
        return int(record[8]) - correction
    return 0

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
    event = False
    max_time_sec = 0.0

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
            if fields[0] == "GNGGA":
                last_gpgga = fields
            elif fields[0] == "GNGLL":
                last_gpgll = fields
            elif fields[0] == "PTH":
                if last_gpgga:
                    last_gpgga.append(fields[1])
                    parsed_sentences.append(last_gpgga)
                    last_gpgga = None
                if last_gpgll:
                    last_gpgll.append(fields[1])
                    parsed_sentences.append(last_gpgll)
                    last_gpgll = None
            else:

                parsed_sentences.append(fields)

                # Monitor acceleration for exit, deployment, and landing events.
                # These will be marked on the timeline slider

                if fields[0] == "PIMU":
                    max_time_sec = int(fields[1]) / 1000
                    norm_g = np.linalg.norm([float(fields[2]), float(fields[3]), float(fields[4])]) / 9.18
                    if norm_g > 1.8 or norm_g < 0.65:
                        if event == False:
                            event = True
                            start_time_ms = int(fields[1])
                    elif event == True:
                        end_time_ms = int(fields[1])
                        event = False
                        gevents.append((start_time_ms, end_time_ms))



    return parsed_sentences, rejected_count, gevents, max_time_sec

def euler_to_matrix(yaw_deg, pitch_deg, roll_deg):
    """Converts Euler angles to a rotation matrix """
    yaw = np.radians(yaw_deg)
    pitch = np.radians(pitch_deg)
    roll = np.radians(roll_deg)

    # Rotation matrix for yaw
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Rotation matrix for pitch
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation matrix for roll
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    return Rz @ Ry @ Rx

# Quaternion to matrix conversion function
def quaternion_to_matrix(w, x, y, z):

    # Extract the values from Q
    q0 = w
    q1 = x
    q2 = y
    q3 = z
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 4x4 rotation matrix
    matrix = [[r00, r10, r20, 0],
              [r01, r11, r21, 0],
              [r02, r12, r22, 0],
              [0,0,0,1]]
    return matrix

'''
template <typename T, precision P>
	GLM_FUNC_QUALIFIER tmat4x4<T, P> lookAtRH
	(
		tvec3<T, P> const & eye,
		tvec3<T, P> const & center,
		tvec3<T, P> const & up
	)
	{
		tvec3<T, P> const f(normalize(center - eye));
		tvec3<T, P> const s(normalize(cross(f, up)));
		tvec3<T, P> const u(cross(s, f));

		tmat4x4<T, P> Result(1);
		Result[0][0] = s.x;
		Result[1][0] = s.y;
		Result[2][0] = s.z;
		Result[0][1] = u.x;
		Result[1][1] = u.y;
		Result[2][1] = u.z;
		Result[0][2] =-f.x;
		Result[1][2] =-f.y;
		Result[2][2] =-f.z;
		Result[3][0] =-dot(s, eye);
		Result[3][1] =-dot(u, eye);
		Result[3][2] = dot(f, eye);
		return Result;
	}
'''

def normalize(v):    
    norm = np.linalg.norm(v)
    if norm == 0: 
       return v
    return v / norm

def look_at(eye, center, up):
    f = normalize((center - eye))
    s = normalize(np.cross(f,up))
    u = np.cross(s,f)

    result = np.identity(4)
    result[0][0] = s[0]
    result[1][0] = s[1]
    result[2][0] = s[2]
    result[0][1] = u[0]
    result[1][1] = u[1]
    result[2][1] = u[2]
    result[0][2] = -f[0]
    result[1][2] = -f[1]
    result[2][2] = -f[2]
    result[3][0] = - np.dot(s,eye)
    result[3][1] = - np.dot(u,eye)
    result[3][2] = np.dot(f, eye)
    return result

class OpenGLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.orientation = np.identity(4)  # B frame orientation wrt NED frame
        self.viewpoint = (20, 0, 0)
        self.up = (0, 0, -1)
        self.look_at = (0, 0, 0)
        self.look_at_matrix = look_at(np.array(self.viewpoint), np.array(self.look_at), np.array(self.up))
        self.data = None          
        self.model_path = MODEL_PATH
        self.playback_speed = 1.0
        self.start_time = None
        self.is_paused = False
        self.pause_time = 0
        self.lastGroundTrack_degT = None
        self.help_label = QtWidgets.QLabel(self)
        self.help_label.setText("Legend")
        self.help_label.setStyleSheet("font-family: Courier; font-Weight: bold; background-color: rgba(20, 20, 20, 150); color: white; font-size: 16px; padding: 20px;")
        self.help_label.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        self.help_label.setFixedWidth(300)
        self.is_playing = True
        self.acceleration = (0, 0, 0)
        self.net_acceleration = 0

        self.model = pywavefront.Wavefront(MODEL_PATH,strict=False)

    def mouseMoveEventX(self, event):
        if event.buttons() & QtCore.Qt.MidButton:
            if self.is_dragging:
                self.opengl_widget.set_viewpoint((event.x(), event.y(), 0), (0, 0, -1))
            else:
                self.is_dragging = True
            print(event.globalPos().x(), event.globalPos().y())

    def set_data(self, data):
        self.data = data

    def set_text(self, text):
        self.help_label.setText( text )

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.1, 0.1, 0.1, 1.0)

    def update_acceleration(self, x, y, z):
        self.acceleration = (x/9.18, y/9.18, z/9.18)
        self.net_acceleration = np.linalg.norm(self.acceleration)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()

        glLight(GL_LIGHT0, GL_POSITION,  (- LIGHT_DISTANCE, 0, 0, 1)) 
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0, 0, 0, 0.3))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1, 1, 1, 0.7))
        
        # Setup camera
        #gluLookAt(self.viewpoint[0], self.viewpoint[1], self.viewpoint[2], 0, 0, 0, *self.up)
        glLoadMatrixf(self.look_at_matrix)

        self.draw_ground_velocity_vector()

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE )

        # Apply the Tempo unit's reported orientation
        glPushMatrix()

        # technically speaking, the Tempo unit will be resting in the jumper's pocket
        # at an orientation that doesn't match the jumper's body frame. We'll need to estimate that 
        # orientation relative to the skydiver's body to get a more accurate rendering

        glMultMatrixf(self.orientation)

        #a = time.time() % 360 * 5

        #glRotatef(a , 1, 0, 0)
        #glRotatef(a, 0, 1, 0)
        #glRotatef(a, 0, 0, 1)

        self.draw_axes()

        glRotatef(180, 1, 0, 0)
        glRotatef(90, 0, 1, 0)
        glRotatef(90, 0, 0, 1)  
         
        glScalef(0.05, 0.05, 0.05)
        glColor3f(0,0.5,0)
        visualization.draw(self.model)

        glPopMatrix()
        
    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def draw_ground_velocity_vector(self):
        if self.lastGroundTrack_degT is not None:
            glDisable(GL_LIGHTING)
            glDisable(GL_LIGHT0)
            glDisable(GL_COLOR_MATERIAL)

            glColor3f(0.8,0.8,0.8)
            glBegin(GL_LINES)
            glVertex3f(0, 0, 0)
            glVertex3f(AXIS_LINE_LENGTH * np.cos(np.radians(self.lastGroundTrack_degT)), AXIS_LINE_LENGTH * np.sin(np.radians(self.lastGroundTrack_degT)), 0)
            glEnd()

            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            glEnable(GL_COLOR_MATERIAL)

    def draw_axes(self):
        axes = [
            ((4, 0, 0), b'X', (1, 0, 0)),
            ((0, 4, 0), b'Y', (0, 1, 0)),
            ((0, 0, 4), b'Z', (0, 0, 1)),
        ]

        glDisable(GL_LIGHTING)
        glDisable(GL_LIGHT0)
        glDisable(GL_COLOR_MATERIAL)
        
        glLineWidth(4.0)
        for (dir_vec, label, color) in axes:
            glColor3f(*color)
            glBegin(GL_LINES)
            glVertex3f(0, 0, 0)
            glVertex3f(*dir_vec)
            glEnd()
            
            glRasterPos3f(*(np.array(dir_vec) * 1.1))
            #for char in label:
            #    glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(char))
            #glutBitmapString(GLUT_BITMAP_HELVETICA_18, label)

        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)

    def update_orientation(self, w, x, y, z):
        self.orientation = quaternion_to_matrix(w, x, y, z) 
        self.update()

    def set_viewpoint(self, point, up):
        self.viewpoint = point
        self.up = up
        self.look_at_matrix = look_at(np.array(self.viewpoint), np.array(self.look_at), np.array(self.up))
        self.update()

    def update_ground_track(self, track):
        self.lastGroundTrack_degT = track
        self.update()   

    def toggle_play_pause(self):
        """Toggle play/pause state."""
        if self.is_playing:
            self.is_paused = True
            self.pause_time += (time.time() - self.start_time) * 1000 * self.playback_speed
        else:
            self.is_paused = False
            self.start_time = time.time()
        self.is_playing = not self.is_playing

class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.opengl_widget = OpenGLWidget(self)

        self.setCentralWidget(self.opengl_widget)
        self.setWindowTitle("Tempo Log Visualizer")
        self.setMouseTracking(True)
        parsed_data, err, gevents, m = parse_nmea_log(sys.argv[1])
        self.timeline = TickOverride(self, maximum=int(m+0.5))
        self.timeline.setGeometry(20, WINDOW_HEIGHT-60, WINDOW_WIDTH-40, 40)
        self.timeline.valueChanged.connect(self.timeline_change)
        lastValue_sec = 0
        for i in gevents:
            value=int(i[0] / 1000)
            if value != lastValue_sec:
                self.timeline.addTick(value=value)
                print(f"Adding tick at {value} sec")
                lastValue_sec = value
        count = len(parsed_data)
        print(f"Loaded {count} records from {sys.argv[1]}; log file spans {m} seconds")
        self.current_index = 0
        self.start_time_sec = None
        self.is_paused = False
        self.playback_speed = 1.0
        self.pause_time = 0
        self.data = parsed_data
        self.lastGroundTrack_degT = None
        self.lastBaroAlt_ft = None
        self.lastGNSSAlt_ft = None
        self.lastGroundTrack_degT = None
        self.lastGroundSpeed_mph = None
        self.lastTimestamp_ms = 0
        self.is_dragging = False
        self.update_animation_frame()

    def timeline_change(self):
        self.scrub_timeline(self.timeline.value() * 1000.0)
        self.is_paused = False
        self.update()

    def keyPressEvent(self, event):
        key = event.text().upper()
        if key == 'Q':
            self.close()
        elif key == 'F':
            self.opengl_widget.set_viewpoint((20, 0, 0), (0, 0, -1))
            print("front")
        elif key == 'R':
            self.opengl_widget.set_viewpoint((0, 20, 0), (0, 0, -1))
            print("right")
        event.accept()

    # Advance the playback to the current visual frame using the system's clock time to
    # maintain a "real time" playback speed.
    def advance_timeline(self):
        if self.start_time_sec is None:
            self.start_time_sec = time.time()

        if not self.is_paused:
            current_time = (time.time() - self.start_time_sec) * 1000 * self.playback_speed + self.pause_time
            while self.current_index < len(self.data):
                record = self.data[self.current_index]
                timestamp = get_timestamp(record)
                if timestamp == -1:
                    self.current_index += 1
                    continue

                if timestamp > current_time:
                    break

                if record[0] == "PIMU":
                    self.opengl_widget.update_acceleration(float(record[2]), float(record[3]), float(record[4]))
                    self.lastTimestamp_ms = int(record[1])
                
                if record[0] == "PIM2":
                    #quaternion = Quaternion([float(record[2]), float(record[3]), float(record[4]), float(record[5])])
                    self.opengl_widget.update_orientation(float(record[2]), float(record[3]), float(record[4]), float(record[5]))
                    self.lastTimestamp_ms = int(record[1])
                
                if record[0] == "PENV":
                    self.lastBaroAlt_ft = float(record[3])
                    self.lastTimestamp_ms = int(record[1])
                
                if record[0] == "GPGGA" or record[0] == "GNGGA":
                    if (len(record[9]) > 0):
                        altitude = float(record[9])
                        self.lastGNSSAlt_ft = METERStoFEET(altitude)
                    else:
                        self.lastGNSSAlt_ft = None

                if record[0] == "GPVTG" or record[0] == "GNVTG":
                    if (len(record[1]) > 0):
                        self.lastGroundTrack_degT = float(record[1])
                    else:
                        self.lastGroundTrack_degT = None
                    if (len(record[5]) > 0):
                        self.lastGroundSpeed_mph = KNOTStoMPH(float(record[5]))
                    else:
                        self.lastGroundSpeed_mph = None

                self.current_index += 1

    # Shift the timeline to a specified new log time, expressed in milliseconds
    def scrub_timeline(self, time_ms):

        # if the desired time is prior to the current time, reset the index
        # otherwise, we can continue from the current index and thus save some effort
        if self.lastTimestamp_ms != None:
            if time_ms < self.lastTimestamp_ms:
                self.current_index = 0
        else:
            self.current_index = 0

        while self.current_index < len(self.data):
            record = self.data[self.current_index]
            timestamp = get_timestamp(record)
            if timestamp == -1:
                self.current_index += 1
                continue

            if timestamp > time_ms:
                break

            self.current_index += 1

        self.start_time_sec = time.time() - time_ms / 1000

    def update_animation_frame(self):
        self.advance_timeline()
        ts_sec = self.lastTimestamp_ms / 1000
        baroAlt_ft = self.lastBaroAlt_ft
        if baroAlt_ft is None:
            baroAlt_ft = -1
        gnssAlt_ft = self.lastGNSSAlt_ft
        if gnssAlt_ft is None:
            gnssAlt_ft = -1
        a = f"Time (sec): {ts_sec:.2f}\nBaro alt (ft,MSL): {baroAlt_ft:.0f}\nGNSS alt (ft,MSL): {gnssAlt_ft:.0f}"
        lastGroundTrack_degT = self.lastGroundTrack_degT
        lastGroundSpeed_mph = self.lastGroundSpeed_mph
        if lastGroundTrack_degT is None:
            lastGroundTrack_degT = -1
        if lastGroundSpeed_mph is None:
            lastGroundSpeed_mph = -1
        b = f"\nGnd track (deg,T): {lastGroundTrack_degT:.0f}\nGnd speed (mph): {lastGroundSpeed_mph:.0f}"
        netAccleration_G = self.opengl_widget.net_acceleration
        c = f"\nAccel (G): {netAccleration_G:.1f}"
        self.opengl_widget.set_text(a + b + c)
        self.opengl_widget.update_ground_track(self.lastGroundTrack_degT)
        self.update()
        QtCore.QTimer.singleShot(16, self.update_animation_frame)

class TickOverride(QtWidgets.QSlider):
    def __init__(self, *args, **kwargs):
        super().__init__(QtCore.Qt.Horizontal, *args, **kwargs)
        self.ticks = set()
        self.setTickPosition(self.TicksAbove)

    def addTick(self, value=None):
        if isinstance(value, bool) or value is None:
            value = self.value()
        if not value in self.ticks and self.minimum() <= value <= self.maximum():
            self.ticks.add(value)
            self.update()

    def removeTick(self, value=None):
        if isinstance(value, bool) or value is None:
            value = self.value()
        if value in self.ticks:
            self.ticks.discard(value)
            self.update()

    def paintEvent(self, event):
        qp = QtWidgets.QStylePainter(self)
        opt = QtWidgets.QStyleOptionSlider()
        style = self.style()
        self.initStyleOption(opt)

        # draw the groove only
        opt.subControls = style.SC_SliderGroove

        qp.drawComplexControl(style.CC_Slider, opt)

        sliderMin = self.minimum()
        sliderMax = self.maximum()
        sliderLength = style.pixelMetric(style.PM_SliderLength, opt, self)
        span = style.pixelMetric(style.PM_SliderSpaceAvailable, opt, self)

        # if the tick option is set and ticks actually exist, draw them
        if self.ticks and self.tickPosition():
            qp.save()
            qp.translate(opt.rect.x() + sliderLength / 2, 0)
            grooveRect = style.subControlRect(
                style.CC_Slider, opt, style.SC_SliderGroove)
            grooveTop = grooveRect.top() - 1
            grooveBottom = grooveRect.bottom() + 1
            ticks = self.tickPosition()
            bottom = self.height()
            for value in sorted(self.ticks):
                x = style.sliderPositionFromValue(
                    sliderMin, sliderMax, value, span)
                if ticks & self.TicksAbove:
                    qp.drawLine(x, 0, x, grooveTop)
                if ticks & self.TicksBelow:
                    qp.drawLine(x, grooveBottom, x, bottom)
            qp.restore()

        opt.subControls = style.SC_SliderHandle
        opt.activeSubControls = style.SC_SliderHandle
        if self.isSliderDown():
            opt.state |= style.State_Sunken
        qp.drawComplexControl(style.CC_Slider, opt)
        qp.end()

if __name__ == "__main__":

    logging.basicConfig(filename='pose-visualizer.log', level=logging.INFO)

    if len(sys.argv) != 2:
        print("Usage: python pose-visualizer.py <logfile-path>")
        sys.exit(1)

    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.resize(WINDOW_WIDTH, WINDOW_HEIGHT)
    window.show()
    sys.exit(app.exec_())