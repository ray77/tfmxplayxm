# tfmxplay XM
![TFMXPlay Logo](img/logo.png)

The Player / Converter strives to meet Turrican 2 standards.
The new XM Converter generates an easily editable, readable XM file. (No data mush like with similar converters.) Macros are mapped to additional tracks using XM note parameters. As a result, the converted songs may not sound 100% like the original, but they are easy to edit.

## Runtime keys

Keys are read from the terminal while the player is running.

![Console01](img/console01.png)

| Key | Action |
|-----|--------|
| **Tab** | Next subsong |
| **Shift+Tab** | Previous subsong |
| **Enter** / **Return** | Toggle frame trace |
| **\\** (backslash) | Set tempo to 6× (70937×6), show CIA value |
| **~** (tilde) | Toggle register trace |
| **Backspace** / **Delete** | Reset tempo to default (PAL 70937 / NTSC 59659) |
| **[** | Increase tempo (~9%) |
| **]** | Decrease tempo (~9%) |
| **{** | Double tempo |
| **}** | Halve tempo |
| **`** (backtick) | Toggle NTSC/PAL |
| **1** | Show channel 0 mute status |
| **2** | Show channel 1 mute status |
| **3** | Show channel 2 mute status |
| **4** | Show channel 3 mute status |
| **5** | Toggle channel 0 macro trace |
| **6** | Toggle channel 1 macro trace |
| **7** | Toggle channel 2 macro trace |
| **8** | Toggle channel 3 macro trace |
| **A–Z** | Lock channel 3, play macro 0–25 (volume 20, 15) |
| **Ctrl+C** | Quit |


EXAMPLE:
./tfmxplay -convert2xm=T2w1.xm mdat.T2_World1

*** Try with Turrican 2 World 01 *** ENJOY!

![hippo](img/v001.gif)
