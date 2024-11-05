import sys
import serial
import numpy as np
from PyQt5 import QtWidgets, QtOpenGL
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutBitmapCharacter, GLUT_BITMAP_HELVETICA_18, glutInit
from PyQt5.QtCore import Qt
from PyQt5 import QtCore
import re
import pywavefront
from pywavefront import visualization

import numpy as np
import quaternion

obj = pywavefront.Wavefront("objects/tempo-cover-off.obj")

glutInit(sys.argv)

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

# NMEA 0183 checksum calculation
def calculate_checksum(data):
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    return checksum

class OpenGLWidget(QtOpenGL.QGLWidget):
    def __init__(self, parent=None):
        super(OpenGLWidget, self).__init__(parent)
        self.orientation = np.identity(4)
        self.viewpoint = (20, 0, 0)
        self.up = (0, 0, -1)

        #self.centralWidget = QtWidgets.QLabel("F - Front View\nR - Right View\nQ - Quit")
        #self.centralWidget.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        #self.setCentralWidget(self.centralWidget)

        self.help_label = QtWidgets.QLabel(self)
        self.help_label.setText("Press F for forward\n      R for Right\n      Q to quit")
        self.help_label.setStyleSheet("font-family: Courier; font-Weight: bold; background-color: rgba(20, 20, 20, 150); color: white; font-size: 16px; padding: 35px;")
        self.help_label.setAlignment(Qt.AlignLeft | Qt.AlignBottom)
        self.help_label.setFixedWidth(300)

        #mesh.texture("objects/tempo-cover-off.mtl", scale=0.1)

    def initializeGL(self):
        glEnable(GL_DEPTH_TEST)
        glClearColor(0.0, 0.0, 0.0, 1.0)

    def paintGL(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
        
        # Setup camera
        gluLookAt(self.viewpoint[0], self.viewpoint[1], self.viewpoint[2], 0, 0, 0, *self.up)

        # Apply the object's orientation
        glPushMatrix()
        
        glMultMatrixf(self.orientation)
        
        self.draw_axes()
        visualization.draw(obj)

        glPopMatrix()

    def resizeGL(self, w, h):
        glViewport(0, 0, w, h)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, w / h, 0.1, 100.0)
        glMatrixMode(GL_MODELVIEW)

    def draw_axes(self):
        axes = [
            ((4, 0, 0), 'X', (1, 0, 0)),
            ((0, 4, 0), 'Y', (0, 1, 0)),
            ((0, 0, 4), 'Z', (0, 0, 1)),
        ]
        
        glLineWidth(4.0)
        for (dir_vec, label, color) in axes:
            glColor3f(*color)
            glBegin(GL_LINES)
            glVertex3f(0, 0, 0)
            glVertex3f(*dir_vec)
            glEnd()
            
            glRasterPos3f(*(np.array(dir_vec) * 1.1))
            for char in label:
                glutBitmapCharacter(GLUT_BITMAP_HELVETICA_18, ord(char))

    def update_orientation(self, w, x, y, z):
        self.orientation = quaternion_to_matrix(w, x, y, z) 
        self.update()

    def set_viewpoint(self, point, up):
        self.viewpoint = point
        self.up = up
        self.update()

class Window(QtWidgets.QMainWindow):
    def __init__(self):
        super(Window, self).__init__()
        self.opengl_widget = OpenGLWidget(self)
        self.setCentralWidget(self.opengl_widget)
        self.setWindowTitle("Tempo Visualizer")
        self.serial_port = serial.Serial('COM8', 115200, timeout=30)  
        self.read_data()

    def read_data(self):
        while self.serial_port.in_waiting:
            line = self.serial_port.readline().decode('ascii', errors='ignore').strip()
            if line.startswith("Q,"):
                self.process_line(line)
        self.update()
        QtCore.QTimer.singleShot(10, self.read_data)

    def process_line(self, line):
        match = re.match(r"Q,\s*([\d\.\-]+),\s*([\d\.\-]+),\s*([\d\.\-]+),\s*([\d\.\-]+)\*([0-9A-Fa-f]{2})", line)
        if match:
            w, x, y, z = map(float, match.groups()[:4])
            checksum = match.groups()[4]
            
            # Validate checksum
            data = line.split('*')[0]
            if int(checksum, 16) == calculate_checksum(data):
                self.opengl_widget.update_orientation(w, x, y, z)
                #print(checksum)
            else:
                print( "Invalid checksum")

    def keyPressEvent(self, event):
        key = event.text().upper()
        if key == 'Q':
            self.close()
        elif key == 'F':
            self.opengl_widget.set_viewpoint((20, 0, 0), (0, 0, -1))
        elif key == 'R':
            self.opengl_widget.set_viewpoint((0, 20, 0), (0, 0, -1))
        event.accept()

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = Window()
    window.resize(1200, 1200)
    window.show()
    sys.exit(app.exec_())
