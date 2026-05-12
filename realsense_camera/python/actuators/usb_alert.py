import os
import subprocess


CH341_START = 0xFF
CH341_END = 0xAA


def build_ch341_payload(light_mode, buzzer_mode, flash_frequency):
    return bytes((CH341_START, light_mode, buzzer_mode, flash_frequency, CH341_END))


CH341_LIGHT_OFF = 0x01
CH341_GREEN = 0x02
CH341_BLUE = 0x03
CH341_RED = 0x04
CH341_CYAN = 0x05
CH341_YELLOW = 0x06
CH341_MAGENTA = 0x07
CH341_WHITE = 0x08

CH341_BUZZER_OFF = 0x01
CH341_BUZZER_ON = 0x02

CH341_FLASH_NONE = 0x01
CH341_FLASH_FAST = 0x02
CH341_FLASH_MEDIUM = 0x03
CH341_FLASH_SLOW = 0x04


class UsbAlertOutput:
    """Send risk level updates to a USB serial buzzer/light controller."""

    def __init__(self, port, baudrate=115200, command_map=None):
        self.port = port
        self.baudrate = int(baudrate)
        self.command_map = dict(command_map or {})
        self._stream = None
        self._last_level = None
        self._disabled = not bool(port)

    def _configure_port(self):
        subprocess.run(
            ["stty", "-F", self.port, str(self.baudrate), "raw", "-echo"],
            check=True,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

    def _ensure_connected(self):
        if self._disabled:
            return False
        if self._stream is not None:
            return True

        try:
            self._configure_port()
            fd = os.open(self.port, os.O_RDWR | os.O_NOCTTY | os.O_SYNC)
            self._stream = os.fdopen(fd, "wb", buffering=0)
            return True
        except OSError as exc:
            print(f"USB alert disabled on {self.port}: {exc}")
            self._disabled = True
            return False
        except subprocess.CalledProcessError as exc:
            print(f"USB alert disabled: failed to configure {self.port}: {exc}. Check /dev/ttyUSB permissions.")
            self._disabled = True
            return False

    def send_level(self, level):
        if level == self._last_level:
            return
        if not self._ensure_connected():
            return

        payload = self.command_map.get(level)
        if payload is None:
            payload = f"{level}\n"

        self.send_payload(payload)
        self._last_level = level

    def send_payload(self, payload):
        if not self._ensure_connected():
            return

        if isinstance(payload, str):
            payload = payload.encode("utf-8")

        try:
            self._stream.write(payload)
        except OSError as exc:
            print(f"USB alert write failed: {exc}")
            self.close()
            self._disabled = True

    def close(self):
        if self._stream is not None:
            self._stream.close()
            self._stream = None
