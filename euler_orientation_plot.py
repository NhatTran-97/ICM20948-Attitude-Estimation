import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import numpy as np
import math as m
import serial
import struct
from threading import Thread
from matplotlib.animation import FuncAnimation

data_format_uart = fmt = "%df"%3

def Rx(theta):
  return np.matrix([[ 1, 0           , 0           ],
                   [ 0, m.cos(theta),-m.sin(theta)],
                   [ 0, m.sin(theta), m.cos(theta)]])
  
def Ry(theta):
  return np.matrix([[ m.cos(theta), 0, m.sin(theta)],
                   [ 0           , 1, 0           ],
                   [-m.sin(theta), 0, m.cos(theta)]])
  
def Rz(theta):
  return np.matrix([[ m.cos(theta), -m.sin(theta), 0 ],
                   [ m.sin(theta), m.cos(theta) , 0 ],
                   [ 0           , 0            , 1 ]])

def plot_euler(euler_angles, color):
    
    v = np.array([[ 1, 0, 0], [ -0.2, -1, -0.5],
                   [ -0.2, 1, -0.5],  [ 0,0,0], [ -0.2, 0, 0.3]]).transpose()
    R_total =  Rz(euler_angles[0]) * Ry(euler_angles[1])  * Rx(euler_angles[2])
    v_rot = np.zeros((3,5))
    v_rot = R_total * v     
    v_rot = np.array(v_rot)
    v_rot = v_rot.transpose()    
    verts = [ [v_rot[0],v_rot[1],v_rot[3]], [v_rot[0],v_rot[2],v_rot[3]], [v_rot[0],v_rot[3],v_rot[4]]]
    ax.add_collection3d(Poly3DCollection(verts, facecolors= color, linewidths=1, edgecolors='r', alpha=1))
    return


def receive_euler():
    ''' Receive quaternion from Serial connection '''
    euler_values = [0, 0, 0]
    # read euler angles, each euler angle is 4bytes-float numbers
    euler_bytes = uart_mcu.read(12)
    # convert to float
    if euler_bytes.__len__() == 12:
        euler_values= struct.unpack(data_format_uart, np.flip(euler_bytes))
    return euler_values


def plot_animation(i):
    ''' Animation function callback to plot the cube '''
    global euler_angles
    # clearing the figure
    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.invert_zaxis()
    ax.invert_yaxis()
    ax.set_xlabel("x label")
    ax.set_ylabel("y label")
    ax.set_zlabel("z label")
    # plot the cube
    plot_euler(euler_angles, 'cyan')


def receive_data():
    ''' Thread function to receive data from Serial connection indefinitely '''
    global euler_angles
    while True:
        euler_angles = receive_euler()


if __name__ == "__main__":
    # Create figure and 3D axes
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect("equal")
    # set axis limits
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    euler_angles = [0,0,0]
    # # Create animation callback
    ani = FuncAnimation(fig, plot_animation, frames=100,
                        interval=10, blit=False)
    
    # # uart comport
    uart_mcu = serial.Serial('COM5', 115200, parity=serial.PARITY_NONE,
                             stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
    uart_thread = Thread(target=receive_data, daemon=True)
    uart_thread.start()
    
    plt.show()