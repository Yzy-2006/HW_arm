"""Small dependency-light protocol helpers shared by the PI05 processes."""

from __future__ import annotations

import base64
import json
import socket
import struct
from typing import Any


MAX_PACKET_BYTES = 32 * 1024 * 1024


def send_packet(sock: socket.socket, payload: dict[str, Any]) -> None:
    body = json.dumps(payload, separators=(",", ":"), allow_nan=False).encode("utf-8")
    if len(body) > MAX_PACKET_BYTES:
        raise ValueError(f"Packet is too large: {len(body)} bytes")
    sock.sendall(struct.pack("!I", len(body)) + body)


def recv_exact(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Peer closed the inference connection")
        chunks.extend(chunk)
    return bytes(chunks)


def recv_packet(sock: socket.socket) -> dict[str, Any]:
    header = recv_exact(sock, 4)
    (size,) = struct.unpack("!I", header)
    if size <= 0 or size > MAX_PACKET_BYTES:
        raise ValueError(f"Invalid packet size: {size}")
    payload = json.loads(recv_exact(sock, size).decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("Packet payload must be a JSON object")
    return payload


def encode_bytes(data: bytes) -> str:
    return base64.b64encode(data).decode("ascii")


def decode_bytes(data: str) -> bytes:
    if not isinstance(data, str):
        raise ValueError("Encoded data must be a string")
    return base64.b64decode(data.encode("ascii"), validate=True)
