import os
import time
import psutil
import subprocess

def get_temp():
    """获取树莓派 CPU 温度"""
    try:
        out = subprocess.check_output(["vcgencmd", "measure_temp"]).decode()
        return float(out.replace("temp=", "").replace("'C\n", ""))
    except Exception:
        try:
            with open("/sys/class/thermal/thermal_zone0/temp", "r") as f:
                return int(f.read().strip()) / 1000.0
        except:
            return None

def get_throttled():
    """检查是否降频/过热"""
    try:
        out = subprocess.check_output(["vcgencmd", "get_throttled"]).decode().strip()
        return out
    except Exception:
        return "N/A"

def get_fan_speed():
    """读取风扇当前状态"""
    try:
        with open("/sys/class/thermal/cooling_device0/cur_state") as f:
            cur = int(f.read().strip())
        with open("/sys/class/thermal/cooling_device0/max_state") as f:
            max_s = int(f.read().strip())
        percent = cur / max_s * 100.0 if max_s > 0 else 0
        return cur, max_s, percent
    except FileNotFoundError:
        return None, None, None

def main():
    while True:
        temp = get_temp()
        throttled = get_throttled()
        cur, max_s, percent = get_fan_speed()

        cpu_usage = psutil.cpu_percent(interval=1)
        per_cpu = psutil.cpu_percent(interval=None, percpu=True)
        mem = psutil.virtual_memory()

        os.system("clear")
        print("=== Raspberry Pi Monitor ===")
        print(f"CPU Temp      : {temp:.1f} °C" if temp else "CPU Temp : N/A")
        print(f"Throttled     : {throttled}")
        print(f"CPU Usage     : {cpu_usage:.1f}%")
        print(f"Per-CPU Usage : {per_cpu}")
        print(f"Memory        : {mem.used/1024/1024:.1f} MB / {mem.total/1024/1024:.1f} MB ({mem.percent}%)")

        if cur is not None:
            print(f"Fan Speed     : state {cur}/{max_s} ({percent:.0f}%)")
        else:
            print("Fan Speed     : N/A")

        time.sleep(1)

if __name__ == "__main__":
    main()
