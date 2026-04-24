# olympus_hlc/csp.py — CSP v1.x encapsulation over UDP/IP (SRS-001, RF-006)

import struct
import zlib


class CSPPacket:
    """
    Encapsulamiento CSP v1.x (CubeSat Space Protocol) sobre UDP/IP (SRS-001).

    Cubre SRS-001, RF-006 y SyRS-016 usando stdlib Python (struct + zlib),
    sin dependencias externas ni libcsp.

    Formato (4B header + payload + 4B CRC-32 big-endian):
      bits 31-30: priority  (2=NORM)
      bits 29-25: src addr  (5 bits, 0–31)
      bits 24-20: dst addr  (5 bits, 0–31)
      bits 19-14: dst port  (6 bits, 0–63)
      bits 13- 8: src port  (6 bits, 0–63)
      bits  7- 2: reserved
      bit      1: CRC32 flag (siempre 1)
      bit      0: reserved
    """

    PRIO_NORM  = 2
    FLAG_CRC32 = 0b01  # CSP_FCRC32 = bit 0 in libcsp (NOT bit 1 which is RDP)
    MIN_SIZE   = 8   # 4B header + 0B payload + 4B CRC

    @staticmethod
    def pack(src: int, dst: int, dport: int, sport: int,
             payload: bytes, prio: int = 2) -> bytes:
        """Construye un paquete CSP con CRC-32."""
        header = (
            ((prio  & 0x03) << 30) |
            ((src   & 0x1F) << 25) |
            ((dst   & 0x1F) << 20) |
            ((dport & 0x3F) << 14) |
            ((sport & 0x3F) <<  8) |
            CSPPacket.FLAG_CRC32
        )
        raw = struct.pack(">I", header) + payload
        crc = struct.pack(">I", zlib.crc32(raw) & 0xFFFFFFFF)
        return raw + crc

    @staticmethod
    def unpack(data: bytes) -> "tuple[int | None, bytes | None]":
        """
        Valida y decapsula un paquete CSP.
        Retorna (header, payload) o (None, None) si el CRC falla (RF-006).
        """
        if len(data) < CSPPacket.MIN_SIZE:
            return None, None

        raw, crc_recv = data[:-4], data[-4:]
        crc_calc = struct.pack(">I", zlib.crc32(raw) & 0xFFFFFFFF)
        if crc_calc != crc_recv:
            return None, None

        header  = struct.unpack(">I", raw[:4])[0]
        payload = raw[4:]
        return header, payload

    @staticmethod
    def dst_port(header: int) -> int:
        return (header >> 14) & 0x3F

    @staticmethod
    def src_port(header: int) -> int:
        return (header >> 8) & 0x3F

    @staticmethod
    def src_addr(header: int) -> int:
        return (header >> 25) & 0x1F
