import socket
import cv2
import numpy as np
import platform
import threading
import time


class CameraSource:
    def __init__(self, udp_ip="0.0.0.0", udp_port=8000):
        # 仍然保留 is_arm 标记
        self.is_arm = platform.machine().startswith("arm") or platform.machine().startswith("aarch64")

        self.frame = None
        self.running = True
        self.last_return_time = 0  # 上一次 get_frame 的时间戳
        self.min_interval = 0.03   # 最小间隔 30ms

        # --- ARM/PC 都用 UDP ---
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((udp_ip, udp_port))
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8*1024*1024)
        self.buffer = b''

        # 开启 UDP 接收线程
        self.thread = threading.Thread(target=self._udp_loop, daemon=True)
        self.thread.start()

    def _udp_loop(self):
        """UDP 接收循环（ARM/PC 通用）"""
        while self.running:
            try:
                data, _ = self.sock.recvfrom(65536)
                self.buffer += data
                start = self.buffer.find(b'\xff\xd8')
                end = self.buffer.find(b'\xff\xd9')
                if start != -1 and end != -1 and end > start:
                    jpg_data = self.buffer[start:end+2]
                    self.buffer = self.buffer[end+2:]
                    frame = cv2.imdecode(np.frombuffer(jpg_data, np.uint8), cv2.IMREAD_COLOR)
                    if frame is not None:
                        self.frame = frame
            except Exception as e:
                print("UDP receive error:", e)

    def get_frame(self):
        """返回最新帧，保证调用间隔 ≥30ms"""
        now = time.time()
        elapsed = now - self.last_return_time
        if elapsed < self.min_interval:
            time.sleep(self.min_interval - elapsed)
        self.last_return_time = time.time()
        return self.frame

    def close(self):
        self.running = False
        self.sock.close()
