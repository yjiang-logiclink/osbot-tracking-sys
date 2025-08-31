# tcp_sender.py
import socket
import struct
import binascii


class TCPSender:
    def __init__(self, host="127.0.0.1", port=12341):
        """
        初始化 TCP 客户端
        """
        self.host = host
        self.port = port
        self.sock = None
        self.connect()

    def connect(self):
        """建立 TCP 连接"""
        if self.sock:
            self.sock.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # 关闭 Nagle 提高实时性
        self.sock.connect((self.host, self.port))
        print(f"[INFO ] Connected to {self.host}:{self.port}")

    def calculate_crc(self, data):
        """CRC32 校验"""
        crc = binascii.crc32(data)
        return struct.pack('!I', crc)

    def pack_message(self, device_id, coordinates, rotations,extra_data):
        fixed = 0x0F0F
        msg_length = 50
        
        x, y, z = coordinates
        xr, yr, zr = rotations
        reserved_bytes = b'0' * 12
        
        message_without_crc = struct.pack('!HHB7f12s', fixed, msg_length, device_id, x, y, z, xr, yr, zr, extra_data, reserved_bytes)
        crc = binascii.crc32(message_without_crc)
        crc_bytes = struct.pack('!I', crc)
        return message_without_crc + crc_bytes

    def send_message(self, device_id, coordinates, rotations):
        """发送消息"""
        try:
            msg = self.pack_message(device_id, coordinates, rotations,0)
            self.sock.sendall(msg)
            # print(f"[SEND ] Device {device_id}: {coordinates}, {rotations}")
        except (BrokenPipeError, ConnectionResetError):
            print("[WARN ] Connection lost, reconnecting...")
            self.connect()
            self.send_message(device_id, coordinates, rotations)  # 重试一次

    def close(self):
        """关闭连接"""
        if self.sock:
            self.sock.close()
            self.sock = None
