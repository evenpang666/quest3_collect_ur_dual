"""Minimal Robotiq 2F socket client for the URCap server on TCP port 63352."""
from __future__ import annotations

import socket
import threading


class Robotiq2FClient:
    def __init__(self, host: str, port: int = 63352, timeout: float = 0.25):
        self.socket = socket.create_connection((host, port), timeout=timeout)
        self.socket.settimeout(timeout)
        self._lock = threading.Lock()

    def _command(self, command: str) -> str:
        with self._lock:
            self.socket.sendall((command.strip() + "\n").encode("ascii"))
            return self.socket.recv(1024).decode("ascii", errors="replace").strip()

    def activate(self, speed: int = 128, force: int = 100) -> None:
        self._command("SET ACT 1")
        self._command("SET GTO 1")
        self._command(f"SET SPE {max(0, min(255, speed))}")
        self._command(f"SET FOR {max(0, min(255, force))}")

    def move_normalized(self, closed: float) -> None:
        self._command(f"SET POS {round(max(0.0, min(1.0, closed)) * 255)}")

    def position_normalized(self) -> float:
        response = self._command("GET POS")
        fields = response.split()
        if len(fields) != 2 or fields[0] != "POS":
            raise RuntimeError(f"unexpected Robotiq response: {response!r}")
        return int(fields[1]) / 255.0

    def close(self) -> None:
        self.socket.close()
