# tfmxplay XM

<p align="center"><img src="img/logo.png" alt="TFMXPlay Logo" width="400"></p>

TFMX (The Final Musicsystem eXtended) player and converter for Chris Huelsbeck's Amiga music format. Converts TFMX modules (mdat + smpl) into high-quality, editable XM (FastTracker II) files — playable in Schism Tracker, MilkyTracker, OpenMPT and others.

The **XM converter** is the main focus of this project and produces remarkably accurate results, especially for **Turrican 2** — volume envelopes, vibrato, sweep effects, multi-phase macros and stereo panning are carefully translated to XM equivalents. Other TFMX modules (Turrican 1, Turrican 3, etc.) also work but may need further tuning.

The built-in **player** is functional but still work in progress — for the best listening experience, convert to XM and use a dedicated tracker like [Schism Tracker](http://schismtracker.org/) or [OpenMPT](https://openmpt.org/).

Unlike other TFMX converters, tfmxplay produces clean, human-readable XM files where macros are mapped to dedicated tracks using XM note parameters. The result is easy to edit and remix.

### Listen

This MP3 was rendered directly from the XM file that tfmxplay's converter produced — TFMX mdat → XM → WAV → MP3, fully automated, no manual editing.

[**Turrican 2 — Title (Subsong 00)**](https://cdn.jsdelivr.net/gh/ray77/tfmxplayxm@master/audio/T2_Title_Sub00_XM_Convert.mp3) — click to play in browser

[**Turrican 2 — Title (Subsong 02)**](https://cdn.jsdelivr.net/gh/ray77/tfmxplayxm@master/audio/T2_Title_Sub02_XM_Convert.mp3) — click to play in browser

[**Turrican 1 — World 1 (Subsong 02)**](https://cdn.jsdelivr.net/gh/ray77/tfmxplayxm@master/audio/T2_World1_Sub02_XM_Convert.mp3) — click to play in browser

## Downloads

Pre-built static binaries (no dependencies required):

| Platform | Binary |
|----------|--------|
| macOS (Apple Silicon + Intel) | [Universal Binary](https://github.com/ray77/tfmxplayxm/releases/latest) |
| Linux x86_64 | [Static Binary](https://github.com/ray77/tfmxplayxm/releases/latest) |
| Linux ARM64 (Raspberry Pi) | [Static Binary](https://github.com/ray77/tfmxplayxm/releases/latest) |
| Windows x64 | [Static .exe](https://github.com/ray77/tfmxplayxm/releases/latest) |

## XM Conversion

```bash
# Basic conversion (default panning: Soft)
tfmxplay mdat.T2_World1 -convert2xm=t2world1.xm

# With subsong selection
tfmxplay mdat.T2_World1 -subsong 3 -convert2xm=t2w1_sub03.xm

# With panning preset
tfmxplay mdat.T2_Title -subsong 0 -convert2xm=t2title.xm -pan=ExperimentalBass
```

### Panning Presets

| Preset | Description |
|--------|-------------|
| **Soft** (default) | Gentle stereo separation, good for headphones and speakers |
| **Amiga** | Original hard left/right Amiga panning |
| **Headphone** | Narrow stereo image, comfortable for long listening |
| **NearMono** | Almost mono, minimal stereo separation |
| **ExperimentalBass** | Bass and drums centered, rest with narrowed stereo — best for modern mixes |

### Conversion Log

Each conversion appends diagnostics to `conversion.log` in the output directory — instrument mapping, volume scaling, panning decisions, and sweep baking details. Useful for debugging and tuning. Logs auto-rotate at ~1 MB (keeps 9 old files).

## Playback

```bash
# Play a TFMX module
tfmxplay mdat.T2_World1

# Play specific subsong
tfmxplay mdat.T2_World1 -subsong 4

# Dump to WAV
tfmxplay mdat.T2_Title -dump
```

### Runtime Keys

Keys are read from the terminal while the player is running.

![Console](img/console01.png)

| Key | Action |
|-----|--------|
| **Tab** | Next subsong |
| **Shift+Tab** | Previous subsong |
| **Enter** | Toggle frame trace |
| **\\** | Set tempo to 6x, show CIA value |
| **~** | Toggle register trace |
| **Backspace** | Reset tempo to default (PAL/NTSC) |
| **[ / ]** | Increase / decrease tempo (~9%) |
| **{ / }** | Double / halve tempo |
| **`** | Toggle NTSC/PAL |
| **1-4** | Show channel 0-3 mute status |
| **5-8** | Toggle channel 0-3 macro trace |
| **A-Z** | Lock channel 3, play macro 0-25 |
| **Ctrl+C** | Quit |

## Command-Line Options

```
General:
  -h, --help              Display help
  -v, --version           Show version
  -i, --info              Show TFMX module info (header, subsongs, format)

Playback:
  -s, --subsong (num)     Select subsong
  -n, --ntsc              Use NTSC rate
  -l, --hle               High-level emulation (lower quality, faster)
  -d, --dump              Dump 16-bit stereo output to tfmx.wav
  -S, --speed             Set speed in clock/2 cycles
  -M, --mute (channels)   Mute channels (e.g. -M 123 mutes ch 1,2,3)

XM Conversion:
  -c, --convert2xm[=file] Convert TFMX to XM file
  -p, --pan (preset)      Panning: Amiga, Soft, Headphone, NearMono, ExperimentalBass
```

## Building from Source

Requires CMake and SDL2.

```bash
# macOS / Linux
cmake -B build
cmake --build build

# Static build (no external dependencies)
cmake -B build -DSTATIC_BUILD=ON
cmake --build build
```

## How It Works

1. **Load** the mdat (music data) and smpl (sample data) files
2. **Simulate** TFMX playback tick-by-tick, writing note/volume events into a linear XM row grid
3. **Post-process**: apply effective volumes, vibrato, sustain/release envelopes, stereo panning, and song-specific patches
4. **Write** the XM file with delta-encoded 8-bit samples, packed pattern data, and instrument headers

TFMX macros (mSetBegin, mOn, mAddVol, mVibrato, mEnv, mWaitUp, etc.) are translated into XM equivalents — volume column effects, effect column commands, and instrument parameters. Multi-phase macros (attack + sustain), sweep baking (mAddBegin), and DMA retrigger loops are handled automatically.

## Tested With

- Turrican 1 (mdat.title)
- Turrican 2 (T2_Title, T2_World1, T2_World2, T2_World5, and others)
- Various other TFMX modules

## Credits

- **Chris Huelsbeck** — TFMX music system and Turrican soundtracks
- **ray77** — tfmxplay XM converter
- **tildearrow** — tfmxplay core engine

![hippo](img/v001.gif)
