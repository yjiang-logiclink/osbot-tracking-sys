import socket
import json

def format_leds(data):
    """Format LEDs into aligned one-line string"""
    cam1 = " ".join(f"({x:4d},{y:4d})" for x, y in data["Cam_1"])
    cam2 = " ".join(f"({x:4d},{y:4d})" for x, y in data["Cam_2"])
    cam3 = " ".join(f"({x:4d},{y:4d})" for x, y in data["Cam_3"])
    return f"Cam_1: {cam1} | Cam_2: {cam2} | Cam_3: {cam3}"

def udp_receiver(ip="0.0.0.0", port=9000):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))

    print(f"[INFO ] Listening on {ip}:{port} (UDP)...")

    while True:
        data, addr = sock.recvfrom(4096)
        try:
            packet = json.loads(data.decode("utf-8"))
            formatted = format_leds(packet)
            print(f"[RECV ] From {addr} | {formatted}")
        except json.JSONDecodeError:
            print(f"[WARN ] Invalid JSON from {addr}: {data}")

if __name__ == "__main__":
    udp_receiver(port=9000)
