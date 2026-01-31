# Hinze Project Context

## Project Overview
Interactive robot companion with ESP32-S3, featuring voice interaction via host PC (Whisper/Claude/Piper), emotional OLED eyes, head movement, and LED feedback.

## Current State (2026-01-31)

### Completed
- [x] Functional Specification Document (hinze_FSD.md)
- [x] Hardware pin assignments finalized
- [x] Basic main.cpp with initialization
- [x] PlatformIO configuration for ESP32-S3 SuperMini
- [x] GitHub repository created: https://github.com/dg3vw/hinze
- [x] Initial flash and boot test successful

### Hardware Verified
| Component | Status | Notes |
|-----------|--------|-------|
| ESP32-S3 SuperMini | Working | USB CDC on /dev/ttyACM0 |
| SSD1306 OLED | Detected | I2C address 0x3C |
| INMP441 Mic | Configured | I2S port 0, not yet tested audio |
| MAX98357A Amp | Configured | I2S port 1, not yet tested audio |
| SG90 Servos | Pins set | Library not yet integrated |
| WS2812 LED | Pin set | Library not yet integrated |
| Button | Pin set | Interrupt not yet attached |

### Boot Output (Verified Working)
```
=================================
  Hinze Robot Companion v0.1
  ESP32-S3 SuperMini
=================================
[INIT] Setting up I2C...
[I2C] Scanning...
[I2C] Device found at 0x3C (OLED)
[I2C] 1 device(s) found
[INIT] Setting up I2S audio...
[I2S] Microphone configured
[I2S] Amplifier configured
[INIT] Setting up servos...
[SERVO] Pan on GPIO 1, Tilt on GPIO 2
[INIT] Setting up LED...
[LED] WS2812 on GPIO 48
[INIT] Setting up button...
[BUTTON] Input on GPIO 0 (active low)
[INIT] Setup complete!
```

## Pin Reference (Quick)
```
I2S:    WS=4, BCK=5, DIN=6 (mic), DOUT=7 (amp)
I2C:    SDA=8, SCL=9
Servo:  Pan=1, Tilt=2
LED:    WS2812=48
Button: GPIO 0 (active low)
```

## Next Steps
1. Add OLED display library and test eye graphics
2. Add WS2812 LED library and test colors
3. Add servo library and test movement
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
