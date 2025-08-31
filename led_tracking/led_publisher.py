import socket
import json

class LedPublisher:
    def __init__(self, ip="127.0.0.1", port=9000):
        """
        LED Publisher (UDP only)
        :param ip: Target server IP
        :param port: Target server port
        """
        self.server_ip = ip
        self.server_port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def package(self, cam1_leds, cam2_leds, cam3_leds):
        """
        Package all camera LED data into one dictionary.
        - cam1_leds: list of 4 (x,y)
        - cam2_leds: list of 2 (x,y)
        - cam3_leds: list of 2 (x,y)
        """
        data = {
            "Cam_1": cam1_leds,  # 4 LEDs
            "Cam_2": cam2_leds,  # 2 LEDs
            "Cam_3": cam3_leds   # 2 LEDs
        }
        return data

    def send(self, data):
        """
        Send LED data (dict) as JSON over UDP.
        """
        if data is None:
            return
        message = json.dumps(data).encode("utf-8")
        self.sock.sendto(message, (self.server_ip, self.server_port))

    def close(self):
        if self.sock:
            self.sock.close()
            self.sock = None
