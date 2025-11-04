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
        fmt = "<fff"  # 3 floats: yaw, pitch, roll
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
        self.setCameraPosition(distance=8, elevation=25)
        self.setBackgroundColor(pg.mkColor(20, 20, 30))

        # Grid
        grid = gl.GLGridItem()
        grid.scale(2, 2, 1)
        self.addItem(grid)

        # World axis (fixed)
        self.world_axis = gl.GLAxisItem()
        self.world_axis.setSize(3, 3, 3)
        self.addItem(self.world_axis)

        # IMU axis (dynamic)
        self.x_line = self._make_line([0, 0, 0], [1.5, 0, 0], color=(1, 0, 0, 1))   # Red X
        self.y_line = self._make_line([0, 0, 0], [0, 1.5, 0], color=(0, 1, 0, 1))   # Green Y
        self.z_line = self._make_line([0, 0, 0], [0, 0, 1.5], color=(0, 0, 1, 1))   # Blue Z
        self.addItem(self.x_line)
        self.addItem(self.y_line)
        self.addItem(self.z_line)

        self.yaw, self.pitch, self.roll = 0, 0, 0

    def _make_line(self, start, end, color):
        pts = np.array([start, end])
        plt = gl.GLLinePlotItem(pos=pts, color=color, width=3, antialias=True)
        return plt

    def update_angles(self, yaw, pitch, roll):
        self.yaw, self.pitch, self.roll = yaw, pitch, roll
        self.update_orientation()

    def update_orientation(self):
        R = self.euler_to_matrix(self.yaw, self.pitch, self.roll)

        origin = np.array([0, 0, 0])
        x_axis = R @ np.array([1.5, 0, 0])
        y_axis = R @ np.array([0, 1.5, 0])
        z_axis = R @ np.array([0, 0, 1.5])

        self.x_line.setData(pos=np.array([origin, x_axis]), color=(1, 0, 0, 1))
        self.y_line.setData(pos=np.array([origin, y_axis]), color=(0, 1, 0, 1))
        self.z_line.setData(pos=np.array([origin, z_axis]), color=(0, 0, 1, 1))

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
# Main Window
# ------------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("IMU Axes Visualizer (PyQtGraph)")
        self.resize(800, 600)
        self.viewer = IMU3DWidget()
        self.setCentralWidget(self.viewer)

        # Serial thread
        self.serial_thread = SerialReader("COM5", 115200)
        self.serial_thread.start()

        # Timer to update view
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_view)
        self.timer.start(30)

    def update_view(self):
        yaw, pitch, roll = self.serial_thread.get_angles()
        self.viewer.update_angles(yaw, pitch, roll)

    def closeEvent(self, event):
        self.serial_thread.running = False
        super().closeEvent(event)


# ------------------------------
# Run app
# ------------------------------
if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
