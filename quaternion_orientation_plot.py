# quat_airframe_uart.py
# pip install pyquaternion pyserial matplotlib
import time, struct, binascii
import numpy as np
import serial
from threading import Thread
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from pyquaternion import Quaternion

# ================== CẤU HÌNH UART/DỮ LIỆU ==================
PORT = "COM5"           # đổi cho đúng
BAUD = 115200
TIMEOUT = 0.20

MODE   = "binary"       # "binary" = 4*float32; "csv" = "w,x,y,z\n"
ENDIAN = "<"            # "<" little-endian, ">" big-endian
LAYOUT = "wxyz"         # dữ liệu MCU là "wxyz" hay "xyzw"
CSV_SEPARATOR = ","     # nếu MODE="csv"

# Hiệu chỉnh hệ trục nếu hình bị lật (tùy IMU)
AXIS_FIX = Quaternion()  # ví dụ: Quaternion(axis=[1,0,0], angle=np.pi)

# ================== HÌNH “CŨ” ==================
def base_shape():
    # 5 đỉnh như code Euler trước đây của bạn
    #   0: mũi   1: cánh trái   2: cánh phải   3: gốc thân   4: đuôi
    return np.array([
        [ 1.0,  0.0,  0.0],   # nose
        [-0.2, -1.0, -0.5],   # left wing
        [-0.2,  1.0, -0.5],   # right wing
        [ 0.0,  0.0,  0.0],   # origin (body root)
        [-0.2,  0.0,  0.3],   # tail
    ], dtype=np.float32)      # (5,3)

def plot_airframe_quaternion(ax, q: Quaternion, color="cyan"):
    v = base_shape()                 # (5,3)
    v_rot = np.empty_like(v)
    for i in range(v.shape[0]):
        v_rot[i] = q.rotate(v[i])

    # 3 mặt như bạn vẽ trước đây
    verts = [
        [v_rot[0], v_rot[1], v_rot[3]],
        [v_rot[0], v_rot[2], v_rot[3]],
        [v_rot[0], v_rot[3], v_rot[4]],
    ]
    ax.add_collection3d(
        Poly3DCollection(verts, facecolors=color, linewidths=1, edgecolors="r", alpha=1.0)
    )

# ================== UART HELPERS ==================
def read_exact(ser, n, timeout_s=TIMEOUT):
    buf = bytearray()
    t0 = time.time()
    while len(buf) < n and (time.time() - t0) < timeout_s:
        chunk = ser.read(n - len(buf))
        if chunk:
            buf.extend(chunk)
        else:
            time.sleep(0.001)
    return bytes(buf)

# ================== PARSERS ==================
def parse_quat_binary(raw16):
    try:
        a, b, c, d = struct.unpack(ENDIAN + "4f", raw16)
    except struct.error:
        return None
    if LAYOUT.lower() == "wxyz":
        w, x, y, z = a, b, c, d
    else:
        w, x, y, z = d, a, b, c
    q = Quaternion(w, x, y, z)
    return q.unit if np.isfinite(q.norm) and q.norm > 1e-8 else None

def parse_quat_csv(line_bytes):
    try:
        parts = [p.strip() for p in line_bytes.decode(errors="ignore").split(CSV_SEPARATOR)]
        if len(parts) < 4: return None
        w, x, y, z = map(float, parts[:4])  # nếu MCU gửi xyzw thì đảo lại ở đây
        q = Quaternion(w, x, y, z)
        return q.unit
    except Exception:
        return None

# ================== RX THREAD ==================
g_q = Quaternion(1, 0, 0, 0)

def rx_loop(ser):
    global g_q
    if MODE == "csv":
        buff = bytearray()
        while True:
            chunk = ser.read(64)
            if not chunk: continue
            buff.extend(chunk)
            while b"\n" in buff:
                line, _, rest = buff.partition(b"\n")
                buff = bytearray(rest)
                q = parse_quat_csv(line)
                if q is not None:
                    g_q = (AXIS_FIX * q).unit
    else:
        # binary
        while True:
            raw = read_exact(ser, 16, TIMEOUT)
            if len(raw) != 16:
                # debug nếu cần:
                # print(f"[BIN] short: {len(raw)} bytes, in_waiting={ser.in_waiting}")
                continue
            q = parse_quat_binary(raw)
            if q is not None:
                g_q = (AXIS_FIX * q).unit
            # else: print("[BIN] bad/unpack fail:", binascii.hexlify(raw))

# ================== ANIMATION ==================
def plot_animation(_):
    ax.clear()
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.set_zlim(-1, 1)
    ax.invert_yaxis()
    ax.invert_zaxis()
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    plot_airframe_quaternion(ax, g_q, color="cyan")

# ================== MAIN ==================
if __name__ == "__main__":
    fig = plt.figure()
    ax  = fig.add_subplot(111, projection="3d")
    ax.set_aspect("auto")

    uart = serial.Serial(
        PORT, BAUD,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=TIMEOUT
    )
    uart.reset_input_buffer()

    Thread(target=rx_loop, args=(uart,), daemon=True).start()

    ani = FuncAnimation(fig, plot_animation, interval=15, blit=False, cache_frame_data=False)
    plt.show()
