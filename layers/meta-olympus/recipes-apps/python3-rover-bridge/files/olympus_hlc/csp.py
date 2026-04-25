# olympus_hlc/csp.py — CSP v1.x encapsulation over UDP/IP (SRS-001, RF-006)

import struct


def _crc32c(data: bytes) -> int:
    """CRC-32C (Castagnoli, poly 0x82F63B78) — matches libcsp 4.x csp_crc32_append."""
    crc = 0xFFFFFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0x82F63B78 if crc & 1 else crc >> 1
    return crc ^ 0xFFFFFFFF


class CSPPacket:
    """
    Encapsulamiento CSP v1.x (CubeSat Space Protocol) sobre UDP/IP (SRS-001).

    Cubre SRS-001, RF-006 y SyRS-016 usando stdlib Python,
    sin dependencias externas ni libcsp.

    Wire format: 4B header (BE) + payload + 4B CRC-32C (BE, over payload-only).
      bits 31-30: priority  (2=NORM)
      bits 29-25: src addr  (5 bits, 0–31)
      bits 24-20: dst addr  (5 bits, 0–31)
      bits 19-14: dst port  (6 bits, 0–63)
      bits 13- 8: src port  (6 bits, 0–63)
      bit      0: FCRC32 flag (0x01) — CRC-32C over payload-only
    """

    PRIO_NORM  = 2
    FLAG_CRC32 = 0x01  # CSP_FCRC32 bit in CSPv1 (libcsp 4.x)
    MIN_SIZE   = 8     # 4B header + 0B payload + 4B CRC

    @staticmethod
    def pack(src: int, dst: int, dport: int, sport: int,
             payload: bytes, prio: int = 2) -> bytes:
        """Construye un paquete CSP con CRC-32C sobre payload."""
        header = (
            ((prio  & 0x03) << 30) |
            ((src   & 0x1F) << 25) |
            ((dst   & 0x1F) << 20) |
            ((dport & 0x3F) << 14) |
            ((sport & 0x3F) <<  8) |
            CSPPacket.FLAG_CRC32
        )
        hdr_bytes = struct.pack(">I", header)
        crc = struct.pack(">I", _crc32c(payload))
        return hdr_bytes + payload + crc

    @staticmethod
    def unpack(data: bytes) -> "tuple[int | None, bytes | None]":
        """
        Valida y decapsula un paquete CSP.
        Retorna (header, payload) o (None, None) si el CRC falla (RF-006).
        CRC-32C se verifica sobre payload-only (sin header), igual que libcsp.
        """
        if len(data) < CSPPacket.MIN_SIZE:
            return None, None

        header_bytes = data[:4]
        payload      = data[4:-4]
        crc_recv     = data[-4:]
        crc_calc = struct.pack(">I", _crc32c(payload))
        if crc_calc != crc_recv:
            return None, None

        header = struct.unpack(">I", header_bytes)[0]
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
