# Hinze Project Context

## Project Overview
Interactive robot companion with ESP32-S3, featuring voice interaction via host PC (Whisper/Claude/Piper), emotional OLED eyes, head movement, and LED feedback.

## Current State (2026-02-03)

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
| INMP441 Mic | **Working** | I2S 32-bit mode, 16kHz, levels 400-4500 |
| MAX98357A Amp | **Working** | I2S 32-bit, 8kHz, ~11s buffer, German TTS |
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
**v0.6** - I2S Microphone + Speaker Output

## Pin Reference (Quick)
```
I2S:    DIN=4 (mic), WS=5, BCK=6, DOUT=7 (amp)
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
{"cmd":"play_start"}                   // Start audio playback mode
{"cmd":"play_stop"}                    // Stop audio playback mode
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

### Audio Packets (Host -> ESP32)
Same binary format, sent after `play_start` command:
- 16-bit signed PCM, little-endian
- 8kHz sample rate (resampled from Piper's 22.05kHz)
- Buffer-then-play: all audio buffered before playback (~11 sec max)

## Audio Implementation Notes

### I2S Port Switching
The mic (INMP441) and speaker (MAX98357A) share the I2S bus (BCK/WS pins). The firmware switches dynamically:
1. Normal state: Mic driver installed (I2S_NUM_0)
2. For playback: Uninstall mic → Install speaker driver → Play → Uninstall speaker → Reinstall mic

### Speaker Configuration
- I2S format: 32-bit stereo (`I2S_BITS_PER_SAMPLE_32BIT`)
- 16-bit audio samples shifted left by 16 bits for 32-bit frame
- DMA: 16 buffers × 1024 samples
- Buffer: 90000 samples (int16_t) = ~11 seconds at 8kHz

### TTS Pipeline
1. Piper synthesizes at 22050Hz
2. Host resamples to 8000Hz (scipy.signal.resample)
3. Audio sent via serial in binary packets
4. ESP32 buffers all audio, then plays

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
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
cp config.py config_local.py  # Edit settings
python hinze_host.py          # Run host application
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
