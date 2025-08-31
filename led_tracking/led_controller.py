import platform
import subprocess
import paramiko

class LedController:
    def __init__(self, username="cam", password=None, hostname="osbot-pi5-1"):
        self.is_arm = platform.machine().startswith("arm") or platform.machine().startswith("aarch64")
        self.gpio_map = {0: 22, 1: 23, 4: 17, 5: 27}

        if not self.is_arm:
            self.hostname = hostname
            self.username = username
            self.password = password
            self.ssh = None
            self._connect()
        else:
            self.ssh = None

    def _connect(self):
        if self.ssh:
            self.ssh.close()
        self.ssh = paramiko.SSHClient()
        self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        self.ssh.connect(self.hostname, username=self.username, password=self.password)

    def _exec(self, cmd):
        if self.is_arm:
            subprocess.run(cmd, shell=True, check=False)
        else:
            try:
                _, _, stderr = self.ssh.exec_command(cmd)
                err = stderr.read().decode().strip()
                if err:
                    print(f"Error executing: {cmd}, stderr: {err}")
            except Exception:
                print("SSH connection lost, reconnecting...")
                self._connect()
                self._exec(cmd)

    def control_led(self, states):
        if len(states) != 6:
            raise ValueError("states must be a list of length 6")

        for idx in [0, 1, 4, 5]:
            pin = self.gpio_map[idx]
            state = "dh" if states[idx] else "dl"
            self._exec(f"pinctrl set {pin} op {state}")

        val = 0
        if states[2]:
            val |= 0x02
        if states[3]:
            val |= 0x01
        self._exec(f"/usr/sbin/i2cset -y 1 0x20 {val}")

    def close(self):
        if self.ssh and not self.is_arm:
            self.ssh.close()
            self.ssh = None
