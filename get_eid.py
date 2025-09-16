#!/usr/bin/env python3
# SPDX-License-Identifier: MIT
# Copyright (c) 2025 Simplex Wireless.
#
# get_eid_euicc.py — Retrieve eUICC EID over an AT serial port using AT+CSIM
#
# Overview
#   Opens a modem AT port (e.g., /dev/ttyUSB2), reports terminal capability,
#   allocates a logical channel via MANAGE CHANNEL, SELECTs the eUICC ISD-R
#   (AID: A0000005591010FFFFFFFF8900000100 with 17-byte fallback), and
#   retrieves the EID using the ES10c GetEuiccDataRequest (BF3E / tag 5A).
#   Falls back to classic GET DATA 9F7F if needed. Handles 61xx/6Cxx flows.
#
# Features
#   • Works with modems exposing 3GPP AT+CSIM (e.g., Quectel, Sierra, Telit).
#   • MANAGE CHANNEL open/close, CLA channel encoding, GET RESPONSE chaining.
#   • Robust TLV parsing (nested), BCD decoding (ignores 0xF nibbles).
#   • Prints the 32-digit EID to stdout; exit code 0 on success, 2 on not found.
#
# Requirements
#   Python 3.8+ and pyserial:  pip install pyserial
#
# Quick start
#   sudo python3 get_eid_euicc.py --port /dev/ttyUSB2 --baud 9600 --verbose
#
# Notes & Caveats
#   • Ensure you’re using the modem’s AT command port (often /dev/ttyUSB2).
#   • Some firmware gates AT+CSIM or requires specific baud rates (9600 is common).
#   • Sending APDUs varies by vendor/firmware; use at your own risk in production.
#
# Contributing
#   Issues and PRs welcome! Please include modem model, firmware, and a verbose
#   run (--verbose) snippet when reporting problems.
#
# Maintainers / Contact
#   Simplex Wireless eSIM team — jan.lattunen@simplexwireless.com
#
# License
#   MIT License. See the repository’s LICENSE file for full text.

import argparse
import binascii
import re
import sys
import time
from typing import List, Optional, Tuple

try:
    import serial  # pyserial
except ImportError:
    print("Missing dependency: pyserial. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)

# AID candidates 
AID_CANDIDATES = [
    "A0000005591010FFFFFFFF8900000100", 
    "A0000005591010FFFFFFFF890000010000", # 17 bytes (fallback)
]

OK_RE = re.compile(rb"(^|\r|\n)OK(\r|\n|$)")
ERR_RE = re.compile(rb"(^|\r|\n)(ERROR|CME ERROR: .+|CMS ERROR: .+)(\r|\n|$)")
CSIM_RE = re.compile(rb'\+CSIM:\s*(\d+),"?([0-9A-Fa-f]*)"?')

def hexstr(b: bytes) -> str:
    return binascii.hexlify(b).decode("ascii").upper()

def unhex(s: str) -> bytes:
    s = s.strip().replace(" ", "")
    if s == "":
        return b""
    return binascii.unhexlify(s)

def encode_csim(apdu_hex: str) -> str:
    return f'AT+CSIM={len(apdu_hex)},"{apdu_hex}"'

def read_until(port: serial.Serial, timeout: float = 5.0) -> bytes:
    deadline = time.time() + timeout
    buf = bytearray()
    while time.time() < deadline:
        n = port.in_waiting
        if n:
            buf.extend(port.read(n))
            if OK_RE.search(buf) or ERR_RE.search(buf):
                break
        else:
            time.sleep(0.02)
    return bytes(buf)

def send_at(port: serial.Serial, cmd: str, verbose: bool = False, timeout: float = 5.0) -> Tuple[bytes, bool]:
    if verbose:
        print(f">>> {cmd}")
    port.reset_input_buffer()
    port.write((cmd + "\r").encode("ascii"))
    rsp = read_until(port, timeout=timeout)
    if verbose:
        print(rsp.decode("utf-8", errors="ignore").rstrip())
    ok = OK_RE.search(rsp) is not None and ERR_RE.search(rsp) is None
    return rsp, ok

def parse_csim_response(rsp: bytes) -> Tuple[bytes, int]:
    matches = CSIM_RE.findall(rsp)
    if not matches:
        return b"", -1
    _, hexdata = matches[-1]
    raw = unhex(hexdata.decode("ascii"))
    if len(raw) < 2:
        return b"", -1
    return raw[:-2], (raw[-2] << 8) | raw[-1]

def cla_for_channel(channel: int, base: int) -> int:
    if channel < 0 or channel > 19:
        raise ValueError("Channel must be 0..19")
    if channel <= 3:
        return base | channel
    return base | 0x40 | (channel - 4)

def build_select_aid_apdu(cla: int, aid_hex: str, p2: int = 0x00) -> str:
    aid = unhex(aid_hex)
    header = bytes([cla, 0xA4, 0x04, p2, len(aid) & 0xFF])
    return hexstr(header + aid)

def build_get_response_apdu(cla: int, le: int) -> str:
    return hexstr(bytes([cla, 0xC0, 0x00, 0x00, le & 0xFF]))

def build_manage_channel_open_apdu() -> str:
    return "0070000001"

def build_manage_channel_close_apdu_p2(channel: int) -> str:
        return hexstr(bytes([0x00, 0x70, 0x80, channel & 0xFF, 0x00]))

def build_get_data_9f7f_apdu(cla: int) -> str:
    return hexstr(bytes([cla, 0xCA, 0x9F, 0x7F, 0x00]))

def build_es10c_get_eid_apdu(cla: int) -> str:
    data = bytes([0xBF, 0x3E, 0x03, 0x5C, 0x01, 0x5A])
    apdu = bytes([cla, 0xE2, 0x91, 0x00, len(data)]) + data + b"\x00"
    return hexstr(apdu)

def apdu_xfer_with_chaining(port: serial.Serial, apdu_hex: str, cla: int, verbose: bool=False) -> Tuple[bytes, int]:
    rsp, ok = send_at(port, encode_csim(apdu_hex), verbose=verbose, timeout=5.0)
    data, sw = parse_csim_response(rsp)
    if sw == -1:
        return b"", sw

    # 6Cxx -> retry with suggested Le
    if (sw >> 8) == 0x6C:
        le = sw & 0xFF
        raw = bytearray(unhex(apdu_hex))
        if len(raw) >= 5:
            raw[-1] = le & 0xFF
            if verbose:
                print(f"Retrying with Le={le:02X} due to 6Cxx")
            return apdu_xfer_with_chaining(port, hexstr(raw), cla, verbose)

    full = bytearray(data)
    while (sw >> 8) == 0x61:
        le = sw & 0xFF
        getresp_hex = build_get_response_apdu(cla, le)
        rsp2, ok2 = send_at(port, encode_csim(getresp_hex), verbose=verbose, timeout=5.0)
        data2, sw2 = parse_csim_response(rsp2)
        full.extend(data2)
        sw = sw2
        if (sw2 >> 8) == 0x6C:
            le2 = sw2 & 0xFF
            getresp_hex = build_get_response_apdu(cla, le2)
            if verbose:
                print(f"GET RESPONSE wrong length; retrying with Le={le2:02X}")
            rsp3, ok3 = send_at(port, encode_csim(getresp_hex), verbose=verbose, timeout=5.0)
            data3, sw3 = parse_csim_response(rsp3)
            full.extend(data3)
            sw = sw3
            break
    return bytes(full), sw

def tlv_parse_find(tag_hex: str, data: bytes) -> Optional[bytes]:
    tag = unhex(tag_hex)
    i = 0
    L = len(data)
    while i < L:
        if i >= L:
            break
        t = data[i:i+1]
        if not t:
            break
        i += 1
        if (t[0] & 0x1F) == 0x1F and i < L:
            t += data[i:i+1]
            i += 1
        if i >= L:
            break
        length_byte = data[i]
        i += 1
        if length_byte & 0x80:
            num = length_byte & 0x7F
            if i + num > L:
                break
            length = int.from_bytes(data[i:i+num], "big")
            i += num
        else:
            length = length_byte
        if i + length > L:
            break
        value = data[i:i+length]
        i += length
        if t == tag:
            return value
        # constructed TLV recursion
        if (t[0] & 0x20) and length > 0:
            nested = tlv_parse_find(tag_hex, value)
            if nested is not None:
                return nested
    return None

def bcd_to_digits_ignore_f(b: bytes) -> str:
    out = []
    for byte in b:
        hi = (byte >> 4) & 0xF
        lo = byte & 0xF
        if hi != 0xF:
            out.append(str(hi))
        if lo != 0xF:
            out.append(str(lo))
    return "".join(out)

def send_terminal_capability(port: serial.Serial, verbose: bool=False) -> bool:
    apdu = "80AA000005A903830107"
    rsp, ok = send_at(port, encode_csim(apdu), verbose=verbose, timeout=5.0)
    _, sw = parse_csim_response(rsp)
    if verbose:
        print(f"Terminal capability SW={sw:04X}" if sw != -1 else "Terminal capability: no +CSIM response")
    return sw == 0x9000

def select_isdr(port: serial.Serial, ch: int, verbose: bool=False) -> Tuple[bool, int]:
    """Try both CLA bases and both AID sizes; return (selected, cla_used)."""
    for base in (0x00, 0x80):
        cla = cla_for_channel(ch, base)
        for aid_hex in AID_CANDIDATES:
            sel_hex = build_select_aid_apdu(cla, aid_hex, p2=0x00)
            _, sw = apdu_xfer_with_chaining(port, sel_hex, cla, verbose)
            if sw == 0x9000:
                return True, cla
            # 61xx handled internally
    return False, 0

def try_es10c_get_eid(port: serial.Serial, ch: int, cla_hint: int, verbose: bool=False) -> Optional[str]:
    bases = (0x80, 0x00) if (cla_hint & 0xC0) == 0x80 else (0x00, 0x80)
    for base in bases:
        cla = cla_for_channel(ch, base)
        apdu_hex = build_es10c_get_eid_apdu(cla)
        data, sw = apdu_xfer_with_chaining(port, apdu_hex, cla, verbose)
        if sw == 0x9000 and data:
            val = tlv_parse_find("BF3E", data) or data
            eid_tlv = tlv_parse_find("5A", val)
            if eid_tlv is None and len(val) == 16:
                eid_tlv = val
            if eid_tlv:
                eid_digits = (
                    eid_tlv.decode() if all(0x30 <= c <= 0x39 for c in eid_tlv)
                    else bcd_to_digits_ignore_f(eid_tlv)
                )
                if len(eid_digits) >= 16:
                    return eid_digits
    return None

def try_getdata_9f7f(port: serial.Serial, ch: int, cla_hint: int, verbose: bool=False) -> Optional[str]:
    bases = (0x00, 0x80) if (cla_hint & 0xC0) == 0x00 else (0x80, 0x00)
    for base in bases:
        cla = cla_for_channel(ch, base)
        apdu_hex = build_get_data_9f7f_apdu(cla)
        data, sw = apdu_xfer_with_chaining(port, apdu_hex, cla, verbose)
        if sw == 0x9000 and data:
            val = tlv_parse_find("9F7F", data)
            if val is None and len(data) == 16:
                val = data
            if val:
                eid_digits = (
                    val.decode() if all(0x30 <= c <= 0x39 for c in val)
                    else bcd_to_digits_ignore_f(val)
                )
                if len(eid_digits) >= 16:
                    return eid_digits
    return None

def open_one_channel(port: serial.Serial, verbose: bool=False) -> int:
    data, sw = apdu_xfer_with_chaining(port, build_manage_channel_open_apdu(), 0x00, verbose)
    if sw == 0x9000 and len(data) == 1:
        ch = data[0]
        if verbose:
            print(f"Opened logical channel {ch}")
        return ch
    return -1

def close_channel_p2(port: serial.Serial, channel: int, verbose: bool=False) -> None:
    apdu_hex = build_manage_channel_close_apdu_p2(channel)
    _, sw = apdu_xfer_with_chaining(port, apdu_hex, 0x00, verbose)
    if verbose:
        print(f"Close channel {channel} SW={sw:04X}")

def find_eid(port_path: str, baud: int, target_channel: Optional[int], verbose: bool=False) -> Optional[str]:
    with serial.Serial(port_path, baudrate=baud, timeout=1, dsrdtr=False, rtscts=False) as ser:
        # Basic init
        send_at(ser, "AT", verbose=verbose)
        send_at(ser, "ATE0", verbose=verbose)
        send_at(ser, "AT+CMEE=2", verbose=verbose)
        time.sleep(0.1)

        send_terminal_capability(ser, verbose=verbose)

        # Open one channel (optionally keep opening until target_channel)
        ch = -1
        if target_channel is None:
            ch = open_one_channel(ser, verbose=verbose)
        else:
            for _ in range(1, 20):
                ch = open_one_channel(ser, verbose=verbose)
                if ch == target_channel:
                    break
            if ch != target_channel and verbose:
                print(f"Did not obtain requested channel {target_channel}, using {ch}")

        if ch <= 0:
            if verbose:
                print("Failed to open a logical channel")
            return None

        try:
            selected, cla_used = select_isdr(ser, ch, verbose=verbose)
            if not selected:
                if verbose:
                    print("Failed to SELECT ISD-R on the opened channel")
                return None

            eid = try_es10c_get_eid(ser, ch, cla_used, verbose=verbose)
            if eid:
                return eid

            # Fallback: classic GET DATA 9F7F
            eid = try_getdata_9f7f(ser, ch, cla_used, verbose=verbose)
            if eid:
                return eid

            return None
        finally:
            close_channel_p2(ser, ch, verbose=verbose)

def main():
    parser = argparse.ArgumentParser(description="Fetch eUICC EID via AT+CSIM over a serial AT port.")
    parser.add_argument("--port", default="/dev/ttyUSB2", help="Serial AT port path (default: /dev/ttyUSB2)")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate (default: 9600)")
    parser.add_argument("--target-channel", type=int, default=None, help="Open channels until this one is allocated, if possible.")
    parser.add_argument("--verbose", action="store_true", help="Enable verbose logging")
    args = parser.parse_args()

    eid = find_eid(args.port, args.baud, args.target_channel, verbose=args.verbose)
    if eid:
        print(eid)
        sys.exit(0)
    else:
        print("ERROR: EID not found. Try --verbose; verify this is the modem's AT port; some modems gate +CSIM.", file=sys.stderr)
        sys.exit(2)

if __name__ == "__main__":
    main()
