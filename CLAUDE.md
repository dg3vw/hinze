# Hinze Project Context

## Project Overview
Interactive robot companion with ESP32-S3, featuring voice interaction via host PC (Whisper/Claude/Piper), emotional OLED eyes, head movement, and LED feedback. Supports WiFi TCP streaming for unlimited speech duration.

## Current State (2026-02-04)

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
- [x] WiFi STA mode with mDNS (hinze.local)
- [x] TCP server on port 8266 (FreeRTOS task on core 0)
- [x] Ring buffer streaming playback (no duration limit)
- [x] Transport abstraction (serial + TCP)
- [x] NVS-stored WiFi credentials (wifi_config command)
- [x] Python TCP client with auto/serial/tcp modes
- [x] Streaming TTS pipeline (chunk-by-chunk)

### Hardware Verified
| Component | Status | Notes |
|-----------|--------|-------|
| ESP32-S3 SuperMini | Working | USB CDC on /dev/ttyACM0 or ACM1 |
| SSD1306 OLED | Working | I2C 0x3C, animated eyes displayed |
| WS2812 LED | Working | Rainbow cycle + emotion colors |
| TTP223 Touch | Working | GPIO 10, active HIGH, triggers recording |
| INMP441 Mic | **Working** | I2S 32-bit mode, 16kHz, levels 400-4500 |
| MAX98357A Amp | **Working** | I2S 32-bit, 8kHz, streaming playback |
| SG90 Servos | Pins set | GPIO 1 (pan), GPIO 2 (tilt), not yet integrated |

### End-to-End Test (2026-02-03)
- ✅ Touch sensor triggers recording
- ✅ Audio captured and streamed to host
- ✅ Whisper transcription working (German)
- ✅ Emotion display updates on ESP32
- ✅ Multi-backend LLM support (Ollama, OpenRouter, Anthropic)
- ✅ Ollama with llama3.2 tested and working (free, local)
- ✅ Short conversation history (3 turns for follow-ups)
- ✅ Piper TTS audio playback working (German voice)
- ✅ Full voice interaction loop complete!

### Firmware Version
**v0.7** - WiFi Streaming Audio

## Architecture
```
Host PC (Python)                      ESP32-S3
┌──────────────┐                ┌─────────────────────┐
│ TCPHandler   │── TCP:8266 ──> │ Core 0: wifiNetTask │
│ (or Serial   │                │   TCP server        │
│  fallback)   │                │   → ring buffer     │
└──────────────┘                │                     │
                                │ Core 1: loop()      │
                                │   eyes/LED/buttons  │
                                │   ring buf → I2S    │
                                └─────────────────────┘
```

## Pin Reference (Quick)
```
I2S:    DIN=4 (mic), WS=5, BCK=6, DOUT=7 (amp)
I2C:    SDA=8, SCL=9
Servo:  Pan=1, Tilt=2
LED:    WS2812=48
Touch:  TTP223=GPIO 10 (active HIGH)
```

## Protocol (Serial + TCP)

Commands and events use the same JSON + binary protocol over both transports.

### Host -> ESP32 (JSON commands)
```json
{"cmd":"emotion","state":"happy"}     // Set emotion
{"cmd":"status"}                       // Request status
{"cmd":"record_start"}                 // Start recording
{"cmd":"record_stop"}                  // Stop recording
{"cmd":"play_start"}                   // Start streaming playback mode
{"cmd":"play_stop"}                    // Signal end of audio stream
{"cmd":"wifi_config","ssid":"...","pass":"..."}  // Store WiFi credentials (NVS)
{"cmd":"wifi_status"}                  // Get WiFi/TCP connection info
```

### ESP32 -> Host (JSON events)
```json
{"event":"ready"}                      // Firmware initialized
{"event":"button","action":"press"}    // Button pressed
{"event":"audio_start"}                // Recording started
{"event":"audio_end"}                  // Recording stopped
```

### Audio Packets (both directions)
Binary format: `[0xAA][0x55][len_high][len_low][pcm_data...]`
- 16-bit signed PCM, little-endian
- Mic → Host: 16kHz
- Host → Speaker: 8kHz

## Streaming Audio Architecture

### Ring Buffer (replaces old 90KB static buffer)
- 8,000 samples (16 KB) = 1 second at 8kHz
- Prefill: 0.5s (4,000 samples) before playback starts
- Underrun recovery: rebuffer 0.25s then resume
- Non-blocking: 256 samples per loop iteration

### Audio States
| State | Description |
|-------|-------------|
| AUDIO_IDLE | No audio activity |
| AUDIO_LISTENING | Touch triggered, preparing to record |
| AUDIO_STREAMING | Recording mic audio, streaming to host |
| AUDIO_BUFFERING | Receiving TTS, prefilling ring buffer |
| AUDIO_PLAYING | Streaming playback from ring buffer |
| AUDIO_REBUFFERING | Underrun recovery, waiting for refill |

### Playback Flow
1. Host sends `play_start` → ESP32 switches to speaker I2S, enters BUFFERING
2. Host streams audio packets → ESP32 writes to ring buffer
3. Ring buffer reaches 0.5s prefill → transitions to PLAYING
4. ESP32 feeds I2S from ring buffer each loop iteration (non-blocking)
5. Eyes/LED/buttons keep animating during playback (30 FPS)
6. Host sends `play_stop` → sets stream-end marker
7. ESP32 plays until ring buffer drains, then switches back to mic

### Memory Budget
- Old: 180 KB (play buffer) + no WiFi = ~200 KB
- New: 16 KB (ring buffer) + ~60 KB (WiFi stack) = ~76 KB
- Net savings: ~124 KB freed

## WiFi Setup
```bash
# 1. Flash firmware
pio run -t upload

# 2. Set WiFi credentials via serial (stored in NVS, survives reboot)
echo '{"cmd":"wifi_config","ssid":"MyNetwork","pass":"MyPassword"}' > /dev/ttyACM0

# 3. Reboot ESP32 - it will connect and print IP
# [WIFI] Connected! IP: 192.168.1.42
# [WIFI] mDNS: hinze.local:8266

# 4. Run host with TCP
python hinze_host.py --tcp
# or auto-detect:
python hinze_host.py
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
3. ~~Implement serial command parser (JSON)~~ (done)
4. ~~Add I2S microphone capture~~ (done)
5. ~~Create Python host application skeleton~~ (done)
6. ~~Set up Python venv and install dependencies~~ (done)
7. ~~Test I2S microphone audio capture end-to-end~~ (done)
8. ~~Test Python host with Whisper transcription~~ (done - German working)
9. ~~Test LLM integration~~ (done - Ollama/OpenRouter/Anthropic)
10. ~~Add I2S amplifier audio playback~~ (done - Piper TTS to ESP32)
11. ~~WiFi streaming audio~~ (done - TCP, ring buffer, unlimited duration)
12. Add servo library and test head movement

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
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
cp config.py config_local.py  # Edit settings
python hinze_host.py          # Run (auto mode: TCP then serial)
python hinze_host.py --tcp    # Force TCP
python hinze_host.py --serial # Force serial
python hinze_host.py --host 192.168.1.42  # Specific IP
```

## LLM Backends
Configure in `host/config_local.py`:

| Backend | Config | Notes |
|---------|--------|-------|
| **Ollama** | `LLM_BACKEND = "ollama"` | Free, local, requires `ollama pull llama3.2` |
| **OpenRouter** | `LLM_BACKEND = "openrouter"` | Free tier available, needs API key |
| **DeepSeek** | `LLM_BACKEND = "deepseek"` | Very affordable, fast responses |
| **Anthropic** | `LLM_BACKEND = "anthropic"` | Claude API, needs credits |

```python
# Example config_local.py for DeepSeek
LLM_BACKEND = "deepseek"
DEEPSEEK_API_KEY = "sk-..."
DEEPSEEK_MODEL = "deepseek-chat"  # or "deepseek-reasoner"
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
