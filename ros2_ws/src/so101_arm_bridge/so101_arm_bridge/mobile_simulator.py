"""Explicit PC-only ctypes transport for the native firmware test plant."""

import ctypes
from pathlib import Path


class NativeMobileSimulator:
    def __init__(self, library: Path, clock_ms, boot_id=1):
        self.library = ctypes.CDLL(str(library.resolve()))
        self.now = clock_ms
        self.library.pc_mobile_reset.argtypes = [ctypes.c_uint32]
        self.library.pc_mobile_exchange.argtypes = [
            ctypes.c_char_p,
            ctypes.c_int,
            ctypes.c_uint32,
            ctypes.POINTER(ctypes.c_uint8),
        ]
        self.library.pc_mobile_exchange.restype = ctypes.c_int
        self.library.pc_mobile_transmissions.restype = ctypes.c_uint32
        self.library.pc_mobile_reset(boot_id)

    def exchange(self, packet):
        output = (ctypes.c_uint8 * 64)()
        if (
            self.library.pc_mobile_exchange(
                packet, len(packet), self.now() & 0xFFFFFFFF, output
            )
            != 1
        ):
            raise RuntimeError("native simulator framing/router failure")
        return bytes(output)
