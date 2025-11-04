import sys, struct, math, threading, time
import numpy as np
import serial
import pyqtgraph as pg
import pyqtgraph.opengl as gl
from PyQt6.QtWidgets import QApplication, QMainWindow
from PyQt6.QtCore import QTimer


# ------------------------------
# UART Thread
# ------------------------------
class SerialReader(threading.Thread):
    def __init__(self, port="COM5", baud=115200):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.last_angles = (0.0, 0.0, 0.0)
        self.running = True
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            print(f"[INFO] Connected {self.port}")
        except Exception as e:
            print(f"[ERROR] {e}")
            self.ser = None

    def run(self):
        fmt = "<fff"
        while self.running and self.ser:
            data = self.ser.read(12)
            if len(data) == 12:
                yaw, pitch, roll = struct.unpack(fmt, data)
                self.last_angles = (yaw, pitch, roll)
            time.sleep(0.01)

    def get_angles(self):
        return self.last_angles


# ------------------------------
# 3D Visualization Widget
# ------------------------------
class IMU3DWidget(gl.GLViewWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU 3D Viewer (PyQtGraph)")
        self.setCameraPosition(distance=8, elevation=20)
        self.setBackgroundColor(pg.mkColor(25, 25, 35))

        # Add grid
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.addItem(grid)

        # Create 3D axis lines
        self.axis = gl.GLAxisItem()
        self.axis.setSize(2, 2, 2)
        self.addItem(self.axis)

        # Create cube (8 vertices)
        verts = np.array([
            [1, 1, 1], [-1, 1, 1], [-1, -1, 1], [1, -1, 1],
            [1, 1, -1], [-1, 1, -1], [-1, -1, -1], [1, -1, -1]
        ])

        faces = np.array([
            [0, 1, 2], [0, 2, 3],
            [4, 5, 6], [4, 6, 7],
            [0, 1, 5], [0, 5, 4],
            [2, 3, 7], [2, 7, 6],
            [1, 2, 6], [1, 6, 5],
            [0, 3, 7], [0, 7, 4]
        ])

        colors = np.ones((len(faces), 4)) * [0.1, 0.8, 0.9, 0.9]

        self.mesh = gl.GLMeshItem(vertexes=verts, faces=faces, faceColors=colors, smooth=False, drawEdges=True)
        self.mesh.translate(0, 0, 0)
        self.addItem(self.mesh)

        self.yaw, self.pitch, self.roll = 0, 0, 0

    def update_angles(self, yaw, pitch, roll):
        self.yaw, self.pitch, self.roll = yaw, pitch, roll
        self.update_orientation()

    def update_orientation(self):
        m = self.euler_to_matrix(self.yaw, self.pitch, self.roll)
        self.mesh.resetTransform()
        self.mesh.rotate(math.degrees(self.roll), 1, 0, 0)
        self.mesh.rotate(math.degrees(self.pitch), 0, 1, 0)
        self.mesh.rotate(math.degrees(self.yaw), 0, 0, 1)

    @staticmethod
    def euler_to_matrix(yaw, pitch, roll):
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        return Rz @ Ry @ Rx


# ------------------------------
# Main App
# ------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU 3D Viewer - PyQtGraph")
        self.resize(800, 600)
        self.viewer = IMU3DWidget()
        self.setCentralWidget(self.viewer)

        # UART
        self.serial_thread = SerialReader("COM5", 115200)
        self.serial_thread.start()

        # Update Timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_view)
        self.timer.start(30)

    def update_view(self):
        yaw, pitch, roll = self.serial_thread.get_angles()
        self.viewer.update_angles(yaw, pitch, roll)

    def closeEvent(self, event):
        self.serial_thread.running = False
        super().closeEvent(event)


if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
