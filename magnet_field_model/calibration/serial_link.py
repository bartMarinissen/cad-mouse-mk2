"""Serial transport for the bundle calibration link.

Deliberately thin: opening/closing the port, reading a line, writing a
line, and listing available ports. All protocol/state-machine logic lives
in protocol.py / session.py so it can be tested without a real port.
"""

from __future__ import annotations

import serial
import serial.tools.list_ports
from serial.tools.list_ports_common import ListPortInfo


def list_available_ports() -> list[ListPortInfo]:
    return sorted(serial.tools.list_ports.comports(), key=lambda p: p.device)


class SerialLink:
    port: str
    baudrate: int
    _ser: serial.Serial | None

    def __init__(self, port: str, baudrate: int = 921600):
        self.port = port
        self.baudrate = baudrate
        self._ser = None

    def open(self) -> None:
        # Timeout bounds how long readline() can block with no data, so the
        # caller's display loop keeps ticking (e.g. for a live countdown)
        # even when the knob isn't sending anything.
        self._ser = serial.Serial(self.port, self.baudrate, timeout=0.1)

    def close(self) -> None:
        if self._ser is not None:
            self._ser.close()
            self._ser = None

    def readline(self) -> str | None:
        """Read one line, blocking up to the port timeout. Returns None on timeout."""
        assert self._ser is not None, "SerialLink.open() must be called first"
        raw = self._ser.readline()
        if not raw:
            return None
        return raw.decode("utf-8", errors="replace")

    def send(self, line: str) -> None:
        assert self._ser is not None, "SerialLink.open() must be called first"
        self._ser.write(f"{line}\n".encode("utf-8"))
