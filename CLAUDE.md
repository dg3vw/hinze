# Hinze Project Context

## Project Overview
Interactive robot companion with ESP32-S3, featuring voice interaction via host PC (Whisper/Claude/Piper), emotional OLED eyes, head movement, and LED feedback.

## Current State (2026-02-02)

### Completed
- [x] Functional Specification Document (hinze_FSD.md)
- [x] Hardware pin assignments finalized
- [x] PlatformIO configuration for ESP32-S3 SuperMini
- [x] GitHub repository created: https://github.com/dg3vw/hinze
- [x] OLED eye animations - 10 unique emotion styles
- [x] WS2812 LED - Rainbow cycle idle + emotion colors
- [x] TTP223 touch sensor with debounce (GPIO 10)
- [x] Idle behavior: random look-around + natural blink
- [x] I2S microphone input (INMP441)
- [x] Serial JSON command parser
- [x] Audio streaming protocol (binary packets)
- [x] Python host application skeleton
- [x] Python virtual environment created (host/venv)
- [x] All Python dependencies installed (whisper, anthropic, piper-tts)
- [x] Whisper "base" model downloaded (~140MB)
- [x] Host connects to ESP32 serial successfully

### Hardware Verified
| Component | Status | Notes |
|-----------|--------|-------|
| ESP32-S3 SuperMini | Working | USB CDC on /dev/ttyACM0 or ACM1 |
| SSD1306 OLED | Working | I2C 0x3C, animated eyes displayed |
| WS2812 LED | Working | Rainbow cycle + emotion colors |
| TTP223 Touch | Working | GPIO 10, active HIGH, triggers recording |
| INMP441 Mic | Implemented | I2S port 0, 16kHz 16-bit, needs testing |
| MAX98357A Amp | Configured | I2S output, not yet implemented |
| SG90 Servos | Pins set | GPIO 1 (pan), GPIO 2 (tilt), not yet integrated |

### Firmware Version
**v0.4** - I2S Microphone + Serial Commands

## Pin Reference (Quick)
```
I2S:    WS=4, BCK=5, DIN=6 (mic), DOUT=7 (amp)
I2C:    SDA=8, SCL=9
Servo:  Pan=1, Tilt=2
LED:    WS2812=48
Touch:  TTP223=GPIO 10 (active HIGH)
```

## Serial Protocol

### Host -> ESP32 (JSON commands)
```json
{"cmd":"emotion","state":"happy"}     // Set emotion
{"cmd":"status"}                       // Request status
{"cmd":"record_start"}                 // Start recording
{"cmd":"record_stop"}                  // Stop recording
```

### ESP32 -> Host (JSON events)
```json
{"event":"ready"}                      // Firmware initialized
{"event":"button","action":"press"}    // Button pressed
{"event":"audio_start"}                // Recording started
{"event":"audio_end"}                  // Recording stopped
```

### Audio Packets (ESP32 -> Host)
Binary format: `[0xAA][0x55][len_high][len_low][pcm_data...]`
- 16-bit signed PCM, little-endian
- 16kHz sample rate

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
3. ~~Implement serial command parser (JSON)~~ (done)
4. ~~Add I2S microphone capture~~ (done)
5. ~~Create Python host application skeleton~~ (done)
6. ~~Set up Python venv and install dependencies~~ (done)
7. Test I2S microphone audio capture end-to-end
8. Test Python host with Whisper transcription
9. Test Claude API integration
10. Add I2S amplifier audio playback
11. Add servo library and test head movement

## Build Commands
```bash
pio run                    # Build firmware
pio run -t upload          # Flash to ESP32
pio device monitor         # Serial monitor (needs terminal)
pio run -t upload -t monitor  # All-in-one
```

## Host Application
```bash
cd host
pip install -r requirements.txt
cp config.py config_local.py  # Edit with your API key
python hinze_host.py          # Run host application
```

## Repository
- GitHub: https://github.com/dg3vw/hinze
- Branch: master

## Key Files
- `hinze_FSD.md` - Full specification document
- `src/main.cpp` - ESP32 firmware entry point
- `platformio.ini` - Build configuration
- `host/hinze_host.py` - Python host application
- `host/config.py` - Host configuration template
- `host/requirements.txt` - Python dependencies
