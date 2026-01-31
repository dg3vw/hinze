# Hinze Project Context

## Project Overview
Interactive robot companion with ESP32-S3, featuring voice interaction via host PC (Whisper/Claude/Piper), emotional OLED eyes, head movement, and LED feedback.

## Current State (2026-01-31)

### Completed
- [x] Functional Specification Document (hinze_FSD.md)
- [x] Hardware pin assignments finalized
- [x] PlatformIO configuration for ESP32-S3 SuperMini
- [x] GitHub repository created: https://github.com/dg3vw/hinze
- [x] OLED eye animations - 10 unique emotion styles
- [x] WS2812 LED - Rainbow cycle idle + emotion colors
- [x] Button interrupt with debounce (GPIO 1)
- [x] Idle behavior: random look-around + natural blink

### Hardware Verified
| Component | Status | Notes |
|-----------|--------|-------|
| ESP32-S3 SuperMini | Working | USB CDC on /dev/ttyACM0 or ACM1 |
| SSD1306 OLED | Working | I2C 0x3C, animated eyes displayed |
| WS2812 LED | Working | Rainbow cycle + emotion colors |
| Button | Working | GPIO 10, interrupt-based, cycles emotions |
| INMP441 Mic | Configured | I2S port 0, not yet tested audio |
| MAX98357A Amp | Configured | I2S port 1, not yet tested audio |
| SG90 Servos | Pins set | GPIO 1 (pan), GPIO 2 (tilt), not yet integrated |

### Firmware Version
**v0.3** - Full emotion display system

## Pin Reference (Quick)
```
I2S:    WS=4, BCK=5, DIN=6 (mic), DOUT=7 (amp)
I2C:    SDA=8, SCL=9
Servo:  Pan=1, Tilt=2
LED:    WS2812=48
Button: GPIO 10 (active low)
```

## Implemented Emotions
| # | Emotion | Eyes | LED |
|---|---------|------|-----|
| 0 | Idle | Look around + blink | Rainbow cycle |
| 1 | Listening | Wide, pulsing | Blue pulse |
| 2 | Thinking | Squinted, dots "..." | Yellow pulse |
| 3 | Happy | Curved ^ ^ | Green |
| 4 | Sad | Droopy + tear | Blue |
| 5 | Angry | Narrow + brows | Red |
| 6 | Surprised | Wide, tiny pupils | White |
| 7 | Confused | Asymmetric + "?" | Purple |
| 8 | Sleepy | Half-closed + "Zzz" | Dim orange |
| 9 | Excited | Bouncy + sparkles | Fast rainbow |

## Next Steps
1. ~~Add OLED display library and test eye graphics~~ (done)
2. ~~Add WS2812 LED library and test colors~~ (done)
3. Add servo library and test head movement
4. Implement serial command parser (JSON)
5. Test I2S microphone audio capture
6. Test I2S amplifier audio playback
7. Create Python host application skeleton

## Build Commands
```bash
pio run                    # Build
pio run -t upload          # Flash
pio device monitor         # Serial monitor (needs terminal)
pio run -t upload -t monitor  # All-in-one
```

## Repository
- GitHub: https://github.com/dg3vw/hinze
- Branch: master

## Key Files
- `hinze_FSD.md` - Full specification document
- `src/main.cpp` - ESP32 firmware entry point
- `platformio.ini` - Build configuration
