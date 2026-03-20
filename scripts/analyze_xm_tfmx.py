#!/usr/bin/env python3
"""
Analysiert eine von tfmxplay/convert2xm erzeugte XM-Datei:
- Instrument-Header + Sample-Header (Länge, Loop, Vol, RelNote, Finetune)
- Pattern-Zellen (Note / Instrument / Volume-Spalte / Effekt)
- Volume-Spaltenwerte für ein gewähltes Instrument über Pattern-Bereiche

XM Volume-Spalte: Werte 0x10–0x50 entsprechen Linearlautstärke 0–64.
"""

from __future__ import annotations

import argparse
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterator


@dataclass
class XmMainHeader:
    song_name: str
    tracker_name: str
    version: int
    header_size: int
    song_length: int
    restart: int
    num_channels: int
    num_patterns: int
    num_instruments: int
    flags: int
    tempo: int
    bpm: int
    order: list[int]


@dataclass
class XmPattern:
    index: int
    packing: int
    num_rows: int
    packed_size: int
    raw_packed: bytes


@dataclass
class XmCell:
    note: int
    instrument: int
    volume: int
    effect: int
    param: int

    @property
    def empty(self) -> bool:
        return (
            self.note == 0
            and self.instrument == 0
            and self.volume == 0
            and self.effect == 0
            and self.param == 0
        )


def _read_c_string(data: bytes, maxlen: int) -> str:
    return data[:maxlen].split(b"\x00", 1)[0].decode("latin-1", errors="replace")


def parse_xm(path: Path) -> tuple[XmMainHeader, list[XmPattern], int]:
    data = path.read_bytes()
    if data[:17] != b"Extended Module: ":
        raise ValueError("Kein gültiger XM-Header (fehlt 'Extended Module: ')")

    song_name = _read_c_string(data[17:37], 20)
    if data[37] != 0x1A:
        raise ValueError("Erwartetes 0x1A nach Modulnamen fehlt")
    tracker = _read_c_string(data[38:58], 20)
    version = struct.unpack_from("<H", data, 58)[0]
    header_size = struct.unpack_from("<I", data, 60)[0]
    if header_size < 4 + 16 + 256:
        raise ValueError(f"Header zu klein: {header_size}")

    base = 64
    song_length, restart, num_ch, num_pat, num_inst, flags, tempo, bpm = struct.unpack_from(
        "<8H", data, base
    )
    base += 16
    order = list(data[base : base + 256])
    base += 256

    hdr_end = 60 + header_size
    if base != hdr_end:
        # Einige XMs haben zusätzliche Felder; wir lesen nur den Standardblock.
        base = hdr_end

    mh = XmMainHeader(
        song_name=song_name,
        tracker_name=tracker,
        version=version,
        header_size=header_size,
        song_length=song_length,
        restart=restart,
        num_channels=num_ch,
        num_patterns=num_pat,
        num_instruments=num_inst,
        flags=flags,
        tempo=tempo,
        bpm=bpm,
        order=order,
    )

    patterns: list[XmPattern] = []
    pos = hdr_end
    for pi in range(num_pat):
        if pos + 9 > len(data):
            raise ValueError(f"Pattern {pi}: Datei zu kurz")
        ph_len, pack, rows, psz = struct.unpack_from("<IBHH", data, pos)
        pos += 9
        if ph_len != 9:
            # Nicht-Standard: Header kann länger sein
            extra = ph_len - 9
            if extra < 0 or pos + extra > len(data):
                raise ValueError(f"Pattern {pi}: ungültige Header-Länge {ph_len}")
            pos += extra
        if pos + psz > len(data):
            raise ValueError(f"Pattern {pi}: packed data über Dateiende")
        raw = data[pos : pos + psz]
        pos += psz
        patterns.append(
            XmPattern(index=pi, packing=pack, num_rows=rows, packed_size=psz, raw_packed=raw)
        )

    return mh, patterns, pos


def decode_tfmxplay_cell_packed(buf: memoryview, i: int) -> tuple[XmCell, int]:
    """Liest eine Zelle im von convert2xm geschriebenen Format (0x80 / 0x9F)."""
    b = buf[i]
    if b == 0x80:
        return XmCell(0, 0, 0, 0, 0), i + 1
    if b == 0x9F:
        if i + 6 > len(buf):
            raise ValueError("Unvollständige 0x9F-Zelle")
        note, inst, vol, fx, par = struct.unpack_from("<5B", buf, i + 1)
        return XmCell(note, inst, vol, fx, par), i + 6
    # FT2-Maskenformat (Fallback)
    if b & 0x80:
        mask = b
        j = i + 1
        note = inst = vol = fx = par = 0
        if mask & 0x01:
            note = buf[j]
            j += 1
        if mask & 0x02:
            inst = buf[j]
            j += 1
        if mask & 0x04:
            vol = buf[j]
            j += 1
        if mask & 0x08:
            fx = buf[j]
            j += 1
        if mask & 0x10:
            par = buf[j]
            j += 1
        return XmCell(note, inst, vol, fx, par), j
    # Unbekannt: ein Byte überspringen
    return XmCell(0, 0, 0, 0, 0), i + 1


def pattern_grid(pat: XmPattern, num_ch: int) -> list[list[XmCell]]:
    rows: list[list[XmCell]] = []
    buf = memoryview(pat.raw_packed)
    pos = 0
    for _ in range(pat.num_rows):
        line: list[XmCell] = []
        for _ch in range(num_ch):
            cell, pos = decode_tfmxplay_cell_packed(buf, pos)
            line.append(cell)
        rows.append(line)
    if pos != len(buf):
        # Nur warnen — manche Dateien haben Padding
        pass
    return rows


NOTE_NAMES = ["C-", "C#", "D-", "D#", "E-", "F-", "F#", "G-", "G#", "A-", "A#", "B-"]


def note_str(n: int) -> str:
    if n == 0:
        return "---"
    if n == 97:
        return "^^^"
    if 1 <= n <= 96:
        n -= 1
        octv = n // 12
        return f"{NOTE_NAMES[n % 12]}{octv}"
    return f"0x{n:02X}"


def volume_col_str(v: int) -> str:
    if v == 0:
        return "--"
    if 0x10 <= v <= 0x50:
        return f"{v - 0x10:2d} (0x{v:02X})"
    if 0x60 <= v <= 0x6F:
        return f"slideDn-{v & 0x0F} (0x{v:02X})"
    if 0x70 <= v <= 0x7F:
        return f"slideUp-{v & 0x0F} (0x{v:02X})"
    if 0x80 <= v <= 0x8F:
        return f"vibrSpd (0x{v:02X})"
    if 0x90 <= v <= 0x9F:
        return f"vibrDpth (0x{v:02X})"
    return f"? (0x{v:02X})"


def effect_str(ty: int, par: int) -> str:
    if ty == 0 and par == 0:
        return "---"
    hi, lo = (ty >> 4) & 0xF, ty & 0xF
    return f"{hi:X}{lo:X}{par:02X}"


def iter_instruments(data: bytes, first_inst_offset: int) -> Iterator[tuple[int, dict]]:
    """Instrumente 1…128 (FT2/XM wie von tfmxplay geschrieben).

    Auf dem Datei-Offset ``pos`` steht die DWORD „instrument size“ (typ. 263): Länge des
    Instrument-Metablocks **einschließlich** dieser DWORD, d.h. die ersten Sample-Header
    beginnen bei ``pos + instrument_size``, nicht davor. Anschließend folgen die
    Sample-Rohdaten (Delta-PCM).
    """
    pos = first_inst_offset
    for idx in range(1, 129):
        if pos + 4 > len(data):
            break
        inst_size = struct.unpack_from("<I", data, pos)[0]
        if inst_size < 29:
            break
        name = _read_c_string(data[pos + 4 : pos + 26], 22)
        typ = data[pos + 26]
        num_smp = struct.unpack_from("<H", data, pos + 27)[0]
        inst_info = {
            "name": name,
            "type": typ,
            "num_samples": num_smp,
            "instrument_size_field": inst_size,
        }
        sample_infos: list[dict] = []
        if num_smp > 0:
            smp0 = pos + inst_size
            for s in range(num_smp):
                o = smp0 + 40 * s
                if o + 40 > len(data):
                    break
                slen, lstart, llen = struct.unpack_from("<III", data, o)
                vol = data[o + 12]
                ft = struct.unpack_from("<b", data, o + 13)[0]
                ltype = data[o + 14]
                pan = data[o + 15]
                rel = struct.unpack_from("<b", data, o + 16)[0]
                _ = data[o + 17]
                sname = _read_c_string(data[o + 18 : o + 40], 22)
                sample_infos.append(
                    {
                        "length": slen,
                        "loop_start": lstart,
                        "loop_len": llen,
                        "volume": vol,
                        "finetune": ft,
                        "loop_type": ltype,
                        "panning": pan,
                        "relative_note": rel,
                        "sample_name": sname,
                    }
                )
        inst_info["samples"] = sample_infos
        sample_data_total = sum(s["length"] for s in sample_infos)
        yield idx, inst_info
        pos = pos + inst_size + 40 * num_smp + sample_data_total


def main() -> int:
    ap = argparse.ArgumentParser(description="XM-Analyse (tfmxplay-Export)")
    ap.add_argument("xm_path", type=Path, help="Pfad zur .xm-Datei")
    ap.add_argument(
        "--instruments",
        default="1-5,6,14",
        help="Kommagetrennte Liste und/oder Bereiche, 1-basiert (z.B. 1-5,6,14)",
    )
    ap.add_argument("--vol-inst", type=int, default=6, help="Instrument für Volume-Scan (1-basiert)")
    ap.add_argument("--vol-pat-from", type=int, default=14)
    ap.add_argument("--vol-pat-to", type=int, default=20)
    args = ap.parse_args()

    def parse_inst_spec(s: str) -> list[int]:
        out: list[int] = []
        for part in s.split(","):
            part = part.strip()
            if not part:
                continue
            if "-" in part:
                a, b = part.split("-", 1)
                out.extend(range(int(a), int(b) + 1))
            else:
                out.append(int(part))
        return sorted(set(out))

    inst_list = parse_inst_spec(args.instruments)

    mh, patterns, inst_off = parse_xm(args.xm_path)

    print("=" * 72)
    print("HAUPTHEADER")
    print("=" * 72)
    print(f"  Song:        {mh.song_name!r}")
    print(f"  Tracker:     {mh.tracker_name!r}")
    print(f"  Version:     0x{mh.version:04X}")
    print(f"  Header-Size: {mh.header_size}")
    print(f"  Songlänge:   {mh.song_length} (Positionen in Order-Tabelle)")
    print(f"  Kanäle:      {mh.num_channels}")
    print(f"  Patterns:    {mh.num_patterns}")
    print(f"  Instrumente: {mh.num_instruments}")
    print(f"  Tempo/BPM:   speed={mh.tempo}  bpm={mh.bpm}")
    print()

    raw = args.xm_path.read_bytes()
    inst_by_idx: dict[int, dict] = {}
    for idx, info in iter_instruments(raw, inst_off):
        inst_by_idx[idx] = info

    print("=" * 72)
    print("INSTRUMENT-HEADER + SAMPLE-HEADER")
    print("=" * 72)
    for idx in inst_list:
        if idx not in inst_by_idx:
            print(f"  Instrument {idx:3d}: (nicht gelesen / fehlt)")
            continue
        inf = inst_by_idx[idx]
        print(f"  Instrument {idx:3d}  Name: {inf['name']!r}  samples={inf['num_samples']}")
        for si, s in enumerate(inf.get("samples", [])):
            rn = s["relative_note"]
            ft = s["finetune"]
            lt_n = s["loop_type"] & 3
            lt = ("none", "forward", "pingpong", "?")[lt_n]
            if s["loop_type"] > 3:
                lt = f"{lt}/raw={s['loop_type']}"
            print(
                f"    Sample[{si}]: len={s['length']:6d}  "
                f"loop={s['loop_start']}+{s['loop_len']} ({lt})  "
                f"defVol={s['volume']:3d}  finetune={ft:+4d}  relNote={rn:+4d}  "
                f"name={s['sample_name']!r}"
            )
    print()

    def dump_pat_channels(pat_idx: int, channels: list[int], max_rows: int) -> None:
        if pat_idx >= len(patterns):
            print(f"Pattern {pat_idx} existiert nicht (nur {len(patterns)} Patterns).")
            return
        pat = patterns[pat_idx]
        grid = pattern_grid(pat, mh.num_channels)
        print("-" * 72)
        print(
            f"Pattern {pat_idx}: rows={pat.num_rows} packed={pat.packed_size} bytes  "
            f"Kanäle im Dump: {channels}"
        )
        print("-" * 72)
        for ch in channels:
            print(f"  === Kanal {ch} (0-basiert) ===")
            print(
                f"  {'row':>4}  {'note':^6}  {'ins':>3}  {'volCol':^12}  "
                f"{'eff':^5}  (note raw inst vol fx par)"
            )
            for row in range(min(max_rows, len(grid))):
                c = grid[row][ch]
                print(
                    f"  {row:4d}  {note_str(c.note):^6}  {c.instrument:3d}  "
                    f"{volume_col_str(c.volume):^12}  {effect_str(c.effect, c.param):^5}  "
                    f"({c.note:3d} {c.instrument:3d} {c.volume:3d} {c.effect:3d} {c.param:3d})"
                )
            print()

    print("=" * 72)
    print("PATTERN-DUMPS (erste 32 Zeilen)")
    print("=" * 72)
    dump_pat_channels(14, [2], 32)
    dump_pat_channels(21, [0, 1, 4, 5], 32)

    print("=" * 72)
    print(
        f"VOLUME-SPALTEN für Instrument {args.vol_inst} in Patterns "
        f"{args.vol_pat_from}–{args.vol_pat_to} (nur Zeilen mit instrument=={args.vol_inst})"
    )
    print("=" * 72)
    vi = args.vol_inst
    collected: list[tuple[int, int, int, int]] = []  # pat, row, ch, vol_raw
    for pi in range(args.vol_pat_from, args.vol_pat_to + 1):
        if pi >= len(patterns):
            break
        grid = pattern_grid(patterns[pi], mh.num_channels)
        for row, line in enumerate(grid):
            for ch, c in enumerate(line):
                if c.instrument == vi and c.volume != 0:
                    collected.append((pi, row, ch, c.volume))

    if not collected:
        print("  (keine Einträge mit gesetzter Volume-Spalte und diesem Instrument)")
        # Zusatz: kommt das Instrument in diesen Patterns überhaupt vor?
        hits: list[tuple[int, int, int, XmCell]] = []
        for pi in range(args.vol_pat_from, args.vol_pat_to + 1):
            if pi >= len(patterns):
                break
            grid = pattern_grid(patterns[pi], mh.num_channels)
            for row, line in enumerate(grid):
                for ch, c in enumerate(line):
                    if c.instrument == vi:
                        hits.append((pi, row, ch, c))
        if hits:
            print(f"  Hinweis: Instrument {vi} kommt vor (ohne Volume-Spalte oder vol=0), z.B.:")
            for pti, rw, ch, c in hits[:24]:
                print(
                    f"    pat {pti} row {rw:2d} ch {ch}  {note_str(c.note):>4}  "
                    f"volCol={volume_col_str(c.volume):>14}  eff={effect_str(c.effect, c.param)}"
                )
            if len(hits) > 24:
                print(f"    … insgesamt {len(hits)} Zeilen mit diesem Instrument")
    else:
        by_val: dict[int, int] = {}
        for _p, _r, _ch, v in collected:
            by_val[v] = by_val.get(v, 0) + 1
        print("  Rohwert -> Häufigkeit (0x10–0x50 => Linear 0–64):")
        for v in sorted(by_val.keys()):
            lin = f" => linear {v - 0x10}" if 0x10 <= v <= 0x50 else ""
            print(f"    0x{v:02X} ({v:3d}){lin}: {by_val[v]}×")
        print("  Detail (pattern, row, ch, vol_raw, linear_if_mapped):")
        for p, r, ch, v in collected[:80]:
            lin = v - 0x10 if 0x10 <= v <= 0x50 else None
            extra = f" linear={lin}" if lin is not None else ""
            print(f"    pat {p} row {r:2d} ch {ch}  vol 0x{v:02X}{extra}")
        if len(collected) > 80:
            print(f"    ... und {len(collected) - 80} weitere Zeilen")

    return 0


if __name__ == "__main__":
    sys.exit(main())
