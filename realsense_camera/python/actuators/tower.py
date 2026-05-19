import serial
import time


class SignalTowerController:
    """Control a CH341-based USB signal tower (LED + buzzer) at 9600 baud."""

    def __init__(self, port="/dev/ttyUSB0", baudrate=9600):
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(0.1)

        # Track states internally so changes don't overwrite each other
        self._current_light = "01"  # Default: Off
        self._current_buzzer = "01"  # Default: Off
        self._current_flash = "01"  # Default: Continuous

    def _send(self, hex_string: str):
        """Send a hex command string to the device."""
        payload = bytes.fromhex(hex_string.replace(" ", ""))
        self.ser.write(payload)

    def _update_tower(self):
        """Assemble and send the full hardware command state."""
        full_command = f"FF {self._current_light} {self._current_buzzer} {self._current_flash} AA"
        self._send(full_command)

    # --- LIGHTS ---
    def red(self):
        """Set light to red."""
        self._current_light = "04"
        self._update_tower()

    def green(self):
        """Set light to green."""
        self._current_light = "02"
        self._update_tower()

    def yellow(self):
        """Set light to yellow."""
        self._current_light = "06"
        self._update_tower()

    # --- BUZZER ---
    def buzzer(self, state: bool):
        """Enable or disable buzzer. True = On (0x02), False = Off (0x01)."""
        self._current_buzzer = "02" if state else "01"
        self._update_tower()

    # --- FLASH FREQUENCY ---
    def flash(self, mode: int = 1):
        """
        Set flash frequency.
        1 = Continuous (Solid)
        2 = Faster flash (0.85 s/time)
        3 = Fast flash (1.7 s/time)
        4 = Slow flash (2.5 s/time)
        """
        modes = {1: "01", 2: "02", 3: "03", 4: "04"}
        self._current_flash = modes.get(mode, "01")
        self._update_tower()

    # --- STOP ---
    def stop(self):
        """Reset everything to off and solid (no flash)."""
        self._current_light = "01"
        self._current_buzzer = "01"
        self._current_flash = "01"
        self._update_tower()

    def close(self):
        """Safely close the connection."""
        self.stop()
        self.ser.close()
