import os, json
from led_system import LedTrackingSystem

BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # main.py 所在目录
CONFIG_PATH = os.path.join(BASE_DIR, "config.json")    # 拼接路径

if __name__ == "__main__":
    # Load config
    with open(CONFIG_PATH) as f:
        cfg = json.load(f)

    receiver = LedTrackingSystem(
        udp_port=cfg["udp_port"],
        show_windows=cfg["show_windows"],
        resize_scale=cfg["resize_scale"],
        publish_ip=cfg["publish_ip"],
        publish_port=cfg["publish_port"],
        debug=cfg["debug"]
    )
    receiver.run()
