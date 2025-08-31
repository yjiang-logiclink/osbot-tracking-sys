from led_system import LedTrackingSystem

if __name__ == "__main__":
    receiver = LedTrackingSystem(udp_port=8000, show_windows=True, resize_scale=0.33)
    receiver.run()
