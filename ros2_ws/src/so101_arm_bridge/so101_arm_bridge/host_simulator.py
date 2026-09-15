"""PC-only serial-shaped port for the C v2 parser and mobile motor test plant.

C state is process-global; use one instance per loaded library. Arm GET_STATE is
synthetic. Mobile commands run the real endpoint, supervisor, router and packet
code, with ideal synthetic feedback. This never discovers or opens a device.
"""

import ctypes
from pathlib import Path


class NativeHostSerial:
    def __init__(self, library: Path, clock_ms, boot_id=1, *, chunk_size=7):
        if type(chunk_size) is not int or not 1 <= chunk_size <= 1024:
            raise ValueError("bounded chunk size required")
        self.library = ctypes.CDLL(str(library.resolve()))
        self.now, self.chunk_size = clock_ms, chunk_size
        self.library.pc_mobile_reset.argtypes = [ctypes.c_uint32]
        self.library.pc_host_set_attached.argtypes = [ctypes.c_int]
        self.library.pc_host_feed.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint8),
            ctypes.c_int,
        ]
        self.library.pc_host_feed.restype = ctypes.c_int
        self.library.pc_mobile_reset(boot_id)
        self.pending = bytearray()
        self.writes = []

    def write(self, packet):
        self.writes.append(bytes(packet))
        output = (ctypes.c_uint8 * 2048)()
        for start in range(0, len(packet), self.chunk_size):
            chunk = packet[start : start + self.chunk_size]
            count = self.library.pc_host_feed(
                chunk, len(chunk), self.now() & 0xFFFFFFFF, output, len(output)
            )
            if count < 0:
                raise RuntimeError("native host framing/router failure")
            self.pending.extend(bytes(output[:count]))
        return len(packet)

    def read_until(self, delimiter):
        if delimiter != b"\x00":
            raise ValueError("binary delimiter required")
        count = min(self.chunk_size, len(self.pending))
        result = bytes(self.pending[:count])
        del self.pending[:count]
        return result
