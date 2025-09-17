#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# (Change to your preferred license and update year/company as needed.)
# Copyright (c) 2025 Simplex Wireless LLC
"""
Simplex eUICC Profile Lister — SGP.32 / ES10c

Lists profiles stored on an eUICC (eSIM) by sending APDUs over AT+CSIM to the ISD-R.
Implements the ES10c GetProfilesInfo flow mirrored from LPAC behavior.

Key steps
- Report terminal capabilities: 80 AA 00 00 05 A9 03 83 01 07
- MANAGE CHANNEL (open/close)
- SELECT ISD-R (AID: A0000005591010FFFFFFFF8900000100)
  * SELECT with CLA = 0x0N (logical channel in low nibble)
  * GET RESPONSE and subsequent commands with CLA = 0x8N
- Send ES10c ProfileInfoListRequest
- Handle SW=61xx via GET RESPONSE, parse TLVs (BF 2D / A0)
- Extract: ICCID (5A), AID (4F), status (9F70), name (91), provider (92), class (95)

Output
- JSON: {"profiles": [{"iccid","status","status_code","aid","name","provider","class"}, ...]}

Requirements
- Python 3.8+  •  pyserial  (`pip install pyserial`)
- Modem/reader exposing an AT port with AT+CSIM support (e.g., Quectel, Sierra Wireless, Telit)

Usage
    python3 list_profiles.py /dev/ttyUSB2 [baud]
Examples
    python3 list_profiles.py /dev/ttyUSB2
    python3 list_profiles.py /dev/ttyUSB2 115200

Notes
- ICCID decoding assumes BCD in tag 0x5A (ignore 0xF nibbles).
- Behavior aligned with LPAC 1.6.15 trace; adjust retries/timeout as needed per modem.
- Use in test environments first; productionize with error handling and port discovery.

References
- GSMA SGP.32 v1.2 (ES10c.GetProfilesInfo)
- ETSI TS 102 221 (APDU transport), ETSI TS 102 223 (CAT)

Maintainer
- Simplex Wireless • https://simplexwireless.com • jan.lattunen@simplexwireless.com
"""

import sys
import time
import json
import re
from typing import Tuple, List, Dict, Any
try:
    import serial  # pip install pyserial
except ImportError:
    print("This script requires pyserial. Run: pip install pyserial", file=sys.stderr)
    sys.exit(1)

ISDR_AID_HEX = "A0000005591010FFFFFFFF8900000100"  # SGP.32 ISD-R AID

# ---- Utilities ----

def hexstr(b: bytes) -> str:
    return b.hex().upper()

def unhex(s: str) -> bytes:
    return bytes.fromhex(s.replace(" ", "").replace("\n", ""))

def bcd_iccid_from_5A(b: bytes) -> str:
    """Decode ICCID from 5A value (BCD, low nibble first, F is filler)."""
    digits = []
    for bt in b:
        lo = bt & 0x0F
        hi = (bt >> 4) & 0x0F
        for nib in (lo, hi):
            if nib == 0x0F:
                continue
            digits.append(str(nib))
    return "".join(digits)

def parse_csim_payload(payload_hex: str) -> Tuple[bytes, int, int]:
    """
    +CSIM returns hex like "039000" or "6F1F...9000" etc.
    The last two bytes are SW1SW2; everything before is data.
    """
    if len(payload_hex) < 4:
        # just SW?
        sw = unhex(payload_hex.rjust(4, '0'))
        return b"", sw[0], sw[1]
    raw = unhex(payload_hex)
    data, sw1, sw2 = raw[:-2], raw[-2], raw[-1]
    return data, sw1, sw2

# ---- TLV parsing (supports multi-byte tags and lengths) ----

class TLV:
    def __init__(self, tag: bytes, value: bytes, children: List['TLV'] = None):
        self.tag = tag
        self.value = value
        self.children = children or []

    @property
    def is_constructed(self) -> bool:
        return (self.tag[0] & 0x20) != 0

    def find_all(self, tag_bytes: bytes) -> List['TLV']:
        out = []
        if self.tag == tag_bytes:
            out.append(self)
        for ch in self.children:
            out.extend(ch.find_all(tag_bytes))
        return out

def _parse_tag(buf: bytes, i: int) -> Tuple[bytes, int]:
    first = buf[i]
    tag = bytes([first])
    i += 1
    if (first & 0x1F) == 0x1F:  # long-form tag
        while i < len(buf):
            b = buf[i]
            tag += bytes([b])
            i += 1
            if (b & 0x80) == 0:  # last tag byte
                break
    return tag, i

def _parse_len(buf: bytes, i: int) -> Tuple[int, int]:
    l = buf[i]
    i += 1
    if l < 0x80:
        return l, i
    n = l & 0x7F
    if n == 0 or n > 2:
        raise ValueError("Unsupported TLV length (only up to 2 len-bytes handled)")
    val = 0
    for _ in range(n):
        val = (val << 8) | buf[i]
        i += 1
    return val, i

def parse_tlv(buf: bytes, start: int = 0, end: int = None) -> List[TLV]:
    out = []
    if end is None:
        end = len(buf)
    i = start
    while i < end:
        tag, i = _parse_tag(buf, i)
        length, i = _parse_len(buf, i)
        value = buf[i:i+length]
        i += length
        node = TLV(tag, value)
        if node.is_constructed:
            node.children = parse_tlv(node.value, 0, len(node.value))
        out.append(node)
    return out

# ---- Modem wrapper ----

class AtCsimPort:
    def __init__(self, port: str, baud: int = 9600, timeout: float = 5.0):
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.ser = None

    def open(self):
        self.ser = serial.Serial(self.port_name, self.baud, timeout=self.timeout,
                                 xonxoff=False, rtscts=False, dsrdtr=False,
                                 bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
        time.sleep(0.2)
        self.send_at("ATZ")          # reset parser (best-effort)
        self.send_at("AT+CMEE=2")    # verbose errors

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def _read_until(self, deadline: float) -> str:
        buf = b""
        while time.time() < deadline:
            chunk = self.ser.read(4096)
            if chunk:
                buf += chunk
                # stop once we see OK/ERROR
                if b"\r\nOK\r\n" in buf or b"\r\nERROR\r\n" in buf:
                    break
            else:
                # tiny sleep to avoid busy loop
                time.sleep(0.02)
        return buf.decode(errors="ignore")

    def send_at(self, cmd: str) -> str:
        if not cmd.endswith("\r"):
            cmd += "\r"
        self.ser.write(cmd.encode("ascii"))
        return self._read_until(time.time() + self.timeout)

    def send_csim(self, apdu: bytes) -> Tuple[bytes, int, int, str]:
        payload = hexstr(apdu)
        cmd = f'AT+CSIM={len(payload)},"{payload}"'
        resp = self.send_at(cmd)
        m = re.search(r'\+CSIM:\s*\d+,"([0-9A-Fa-f]+)"', resp)
        if not m:
            # Some modems echo the command; include raw for debugging
            raise RuntimeError(f"CSIM no payload. Raw:\n{resp}")
        data_hex = m.group(1)
        data, sw1, sw2 = parse_csim_payload(data_hex)
        return data, sw1, sw2, resp

# ---- APDU helpers ----

def apdu_manage_channel_open() -> bytes:
    # 00 70 00 00 01  (MANAGE CHANNEL: open)
    return unhex("00 70 00 00 01")

def apdu_manage_channel_close(ch: int) -> bytes:
    # 00 70 80 <ch> 00 (close logical channel)
    return bytes([0x00, 0x70, 0x80, ch & 0xFF, 0x00])

def apdu_select_by_aid(cla: int, aid_hex: str) -> bytes:
    aid = unhex(aid_hex)
    return bytes([cla, 0xA4, 0x04, 0x00, len(aid)]) + aid

def apdu_get_response(cla: int, le: int) -> bytes:
    # GET RESPONSE: CLA (with channel), INS C0, P1 00, P2 00, Le
    return bytes([cla, 0xC0, 0x00, 0x00, le & 0xFF])

def apdu_device_caps() -> bytes:
    return unhex("80 AA 00 00 05 A9 03 83 01 07")

def apdu_profile_info_list(cla: int) -> bytes:
    # Build with dynamic CLA = 0x80 | channel 
    body = unhex("E2 91 00 17 BF 2D 14 A0 03 95 01 02 5C 0D 5A 4F 9F 70 90 91 92 93 95 B6 B7 B8 99 00")
    return bytes([cla]) + body

def get_response_if_needed(port: AtCsimPort, cla: int, sw1: int, sw2: int) -> bytes:
    if sw1 == 0x61:
        # Exactly sw2 bytes available
        data, s1, s2, _ = port.send_csim(apdu_get_response(cla, sw2))
        if s1 != 0x90 or s2 != 0x00:
            raise RuntimeError(f"GET RESPONSE failed SW={s1:02X}{s2:02X}")
        return data
    elif sw1 == 0x90 and sw2 == 0x00:
        return b""
    else:
        raise RuntimeError(f"Unexpected SW={sw1:02X}{sw2:02X}")

# ---- High-level flow ----

def select_isdr_and_get_channel(port: AtCsimPort) -> Tuple[int, int, int]:
    # Report terminal capabilities 
    _, s1, s2, _ = port.send_csim(apdu_device_caps())
    if not (s1 == 0x90 and s2 == 0x00):
        raise RuntimeError(f"Device caps failed SW={s1:02X}{s2:02X}")

    # Open channel
    data, s1, s2, _ = port.send_csim(apdu_manage_channel_open())
    if s1 != 0x90 or s2 != 0x00 or len(data) < 1:
        raise RuntimeError(f"MANAGE CHANNEL open failed SW={s1:02X}{s2:02X} data={hexstr(data)}")
    ch = data[0]

    # SELECT ISD-R on that channel
    cla_select = ch & 0x0F  # used '03' not '83' for SELECT(04) step
    data, s1, s2, _ = port.send_csim(apdu_select_by_aid(cla_select, ISDR_AID_HEX))
    # If 61xx, GET RESPONSE with CLA = 0x80 | channel 
    cla_resp = 0x80 | (ch & 0x0F)
    sel_resp = get_response_if_needed(port, cla_resp, s1, s2)  # ensure SW=9000
    # We don't need to use 'sel_resp' further, just required by spec
    return ch, cla_select, cla_resp

def close_channel_quiet(port: AtCsimPort, ch: int):
    try:
        _, s1, s2, _ = port.send_csim(apdu_manage_channel_close(ch))
        # Either 9000 or already closed—ignore errors in cleanup
    except Exception:
        pass

def extract_profiles_from_tlv(root_nodes: List[TLV]) -> List[Dict[str, Any]]:
    """
    Expect outer BF2D (ProfileInfoList). Inside are A0(...) entries.
    Each profile contains (possibly nested) tags:
      5A (ICCID), 4F (AID), 9F70 (state), 91 (name), 92 (provider), 95 (class)
    """
    profiles: List[Dict[str, Any]] = []

    # Find BF2D anywhere
    def flatten(nodes: List[TLV]) -> List[TLV]:
        out = []
        for n in nodes:
            out.append(n)
            out.extend(flatten(n.children))
        return out

    all_nodes = flatten(root_nodes)
    bf2d_nodes = [n for n in all_nodes if n.tag == b'\xBF\x2D']
    if not bf2d_nodes:
        # Sometimes the GET RESPONSE already returns the inside of BF2D — try parsing everything
        containers = [n for n in all_nodes if n.tag == b'\xA0']
    else:
        # Within BF2D, find A0 containers
        containers = []
        for bf in bf2d_nodes:
            containers.extend([n for n in flatten([bf]) if n.tag == b'\xA0'])

    for a0 in containers:
        # Search inside A0 for fields of interest (recursively)
        a0_all = all_nodes = []
        def collect(n: TLV):
            nonlocal a0_all
            a0_all.append(n)
            for c in n.children:
                collect(c)
        a0_all = []
        collect(a0)

        def first_val(tag: bytes) -> bytes:
            for n in a0_all:
                if n.tag == tag:
                    return n.value
            return b""

        iccid_raw = first_val(b'\x5A')
        aid_raw   = first_val(b'\x4F')
        state_raw = first_val(b'\x9F\x70')
        name_raw  = first_val(b'\x91')
        prov_raw  = first_val(b'\x92')
        cls_raw   = first_val(b'\x95')

        iccid = bcd_iccid_from_5A(iccid_raw) if iccid_raw else ""
        aid   = hexstr(aid_raw) if aid_raw else ""
        state = int(state_raw[0]) if state_raw else None
        name  = name_raw.decode('utf-8', errors='ignore') if name_raw else ""
        prov  = prov_raw.decode('utf-8', errors='ignore') if prov_raw else ""
        cls   = int(cls_raw[0]) if cls_raw else None

        status_map = {0: "disabled", 1: "enabled", 2: "installed-not-activated"}
        status_str = status_map.get(state, f"unknown({state})" if state is not None else "")

        profiles.append({
            "status_code": state,
            "status": status_str,
            "iccid": iccid,
            "aid": aid,
            "name": name,
            "provider": prov,
            "class": cls,
        })

    return profiles

def list_profiles(port_name: str, baud: int = 9600, timeout: float = 6.0) -> List[Dict[str, Any]]:
    port = AtCsimPort(port_name, baud, timeout)
    try:
        port.open()
        ch, cla_select, cla_resp = select_isdr_and_get_channel(port)

        # Send ES10c ProfileInfoListRequest (with channel CLA = 0x80|ch)
        data, s1, s2, _ = port.send_csim(apdu_profile_info_list(cla_resp))
        # Expect 61xx, then GET RESPONSE to fetch the TLV
        resp = get_response_if_needed(port, cla_resp, s1, s2)
        # Parse TLV and extract profiles
        nodes = parse_tlv(resp)
        profiles = extract_profiles_from_tlv(nodes)
        return profiles
    finally:
        try:
            # Close the channel if we opened one successfully
            if 'ch' in locals():
                close_channel_quiet(port, ch)
        finally:
            port.close()

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 list_profiles.py /dev/ttyUSB2 [baud]", file=sys.stderr)
        sys.exit(2)
    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 9600
    profiles = list_profiles(port, baud)
    print(json.dumps({"profiles": profiles}, indent=2))

if __name__ == "__main__":
    main()
