# Hinze - Functional Specification Document

## 1. Overview

Hinze is an interactive robot companion based on ESP32-S3, featuring voice interaction, emotional expressions, and head movement capabilities. The system uses a split architecture with the ESP32 handling hardware I/O and a host PC running AI processing (Whisper for speech recognition, multi-backend LLM for conversation, and Piper TTS for speech synthesis). Audio streaming uses WiFi TCP for unlimited speech duration.

## 2. Hardware Components

### 2.1 Main Controller
- **ESP32-S3 SPK Board** - Custom board with 16MB flash, 8MB PSRAM, onboard mics and speaker amp, UART bridge (CH340/CP2102)

### 2.2 Audio System
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Microphone (x2) | MSM261D3526H1CPM | PDM (I2S_NUM_0) | Voice command input (dual onboard) |
| Amplifier | NS4168 | I2S (I2S_NUM_1) | Audio output/speech (onboard) |

### 2.3 Visual Display
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| OLED Display | 0.96" 128x64 SSD1306 | I2C | Emotion display (animated eyes) |
| RGB LED | SK6812 (1x) | GPIO data | Emotion color indication |

### 2.4 Motion System
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Servo Pan | SG90 | PWM | Head horizontal movement (not yet integrated) |
| Servo Tilt | SG90 | PWM | Head vertical movement (not yet integrated) |

### 2.5 User Input
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Touch Sensor | TTP223 | GPIO 1 (active HIGH) | Push-to-talk activation |

## 3. Pin Assignment (ESP32-S3 SPK Board)

### 3.1 I2S Microphone (PDM, I2S_NUM_0)
| Function | GPIO | Component |
|----------|------|-----------|
| PDM CLK (Clock) | 39 | MSM261 SCK |
| PDM DATA (Data In) | 38 | MSM261 SD |
| WS (unused for PDM) | 40 | — |

### 3.2 I2S Speaker (I2S_NUM_1)
| Function | GPIO | Component |
|----------|------|-----------|
| I2S_SPK_BCK (Bit Clock) | 10 | NS4168 BCK |
| I2S_SPK_WS (Word Select) | 45 | NS4168 WS |
| I2S_SPK_DOUT (Data Out) | 9 | NS4168 DIN |
| I2S_SPK_CTRL (Amp Enable) | 46 | NS4168 CTRL (HIGH = on) |

### 3.3 I2C Display
| Function | GPIO | Component |
|----------|------|-----------|
| I2C_SDA | 13 | SSD1306 OLED |
| I2C_SCL | 14 | SSD1306 OLED |

### 3.4 Control Outputs
| Function | GPIO | Component |
|----------|------|-----------|
| SK6812_DATA | 21 | RGB LED |

### 3.5 User Input
| Function | GPIO | Component |
|----------|------|-----------|
| TOUCH_PIN | 1 | TTP223 capacitive touch sensor (active HIGH) |

## 4. Communication

### 4.1 Host Connection
- **WiFi TCP** - Primary connection for audio streaming (port 8266, mDNS as `hinze.local`)
- **Serial (UART)** - Debug output and JSON commands only (115200 baud, /dev/ttyUSB0 via UART bridge)
- **Auto mode** - Host tries TCP first, falls back to serial

### 4.2 Protocol
- **Commands**: JSON format for control messages (same protocol over serial and TCP)
- **Audio Capture**: Raw PCM data (16kHz, 16-bit, mono) - ESP32 → Host (TCP only)
- **Audio Playback**: Raw PCM data (8kHz, 16-bit, mono) - Host → ESP32 (TCP only, streaming via ring buffer)
- **Status Updates**: JSON status messages from ESP32
- **WiFi Config**: Credentials stored in NVS via `wifi_config` command

## 5. Software Architecture

### 5.1 System Overview
```
┌─────────────────────────────────────────────────────────┐
│                    Host PC (Python)                      │
├─────────────────┬─────────────────┬─────────────────────┤
│ Whisper (local) │ LLM (multi-     │   Piper TTS (local) │
│ Speech-to-Text  │ backend)        │   Text-to-Speech    │
└────────┬────────┴────────┬────────┴──────────┬──────────┘
         │                 │                    │
         └────────────────┬┴───────────────────┘
                          │ WiFi TCP / Serial (debug)
┌─────────────────────────┴───────────────────────────────┐
│              ESP32-S3 SPK Board (Arduino)               │
├───────────┬───────────┬───────────┬─────────────────────┤
│ MSM261    │ NS4168    │ SSD1306   │ SK6812              │
│ PDM Mics  │ Speaker   │ Eyes      │ LED                 │
│ (I2S_0)   │ (I2S_1)   │ (I2C)    │ (GPIO)              │
└───────────┴───────────┴───────────┴─────────────────────┘
```

### 5.2 Dual-Core Architecture
```
Core 0: wifiNetTask (FreeRTOS, priority 1)
  - TCP server, accept/disconnect clients
  - Receive JSON commands + audio packets
  - Audio packets → ring buffer (with back-pressure)

Core 1: playbackTask (FreeRTOS, priority 2)
  - Ring buffer → I2S_NUM_1 via blocking i2s_write()
  - Paces naturally at 8kHz sample rate

Core 1: loop() (Arduino, priority 1)
  - RoboEyes animation (30 FPS)
  - LED emotion colors
  - Touch sensor polling
  - Mic capture (when recording)
  - Serial debug/command input
```

### 5.3 ESP32 Firmware (Arduino/PlatformIO)
- **Framework**: Arduino with PlatformIO build system
- **Audio Input**: PDM microphone via I2S_NUM_0, 16kHz, 16-bit
- **Audio Output**: I2S speaker via I2S_NUM_1, 8kHz, 32-bit stereo (streaming via ring buffer in PSRAM)
- **Dual I2S**: Mic and speaker on separate I2S peripherals — no switching needed
- **Amp Control**: NS4168 enable pin (GPIO 46) — amp on only during playback
- **WiFi**: STA mode, TCP server on port 8266, FreeRTOS task on core 0
- **Serial**: UART0 via bridge chip, 115200 baud, debug output + JSON commands only

### 5.4 Host Application (Python)
| Component | Technology | Purpose |
|-----------|------------|---------|
| Speech-to-Text | Whisper (local) | Convert user speech to text |
| AI Backend | Multi-backend LLM | Process queries, generate responses |
| Text-to-Speech | Piper TTS (local) | Convert response text to audio |
| Communication | TCP sockets / pyserial | WiFi primary, serial fallback |

### 5.5 LLM Backends
| Backend | Config | Notes |
|---------|--------|-------|
| **Ollama** | `LLM_BACKEND = "ollama"` | Free, local, requires `ollama pull llama3.2` |
| **OpenRouter** | `LLM_BACKEND = "openrouter"` | Free tier available, needs API key |
| **DeepSeek** | `LLM_BACKEND = "deepseek"` | Very affordable, fast responses |
| **Anthropic** | `LLM_BACKEND = "anthropic"` | Claude API, needs credits |

### 5.6 Data Flow
1. User touches sensor → ESP32 starts recording
2. MSM261 PDM mic captures audio → ESP32 streams 16kHz PCM to Host via TCP
3. Whisper transcribes speech to text (German/English)
4. LLM processes query, returns response + emotion tag (e.g., `[happy]`)
5. Piper TTS generates audio chunks (22kHz) → resampled to 8kHz in real-time
6. Host streams: emotion command + audio packets to ESP32 via TCP
7. ESP32 fills ring buffer (PSRAM), starts playback after 0.5s prefill
8. NS4168 amp enabled, I2S_NUM_1 plays audio while eyes/LED continue animating

## 6. Activation Modes

| Mode | Description | Status |
|------|-------------|--------|
| Button Press | Push-to-talk with silence detection | **Implemented** |
| Wake Word | Local detection of "Hey Hinze" | Planned |
| Always Listening | Continuous speech detection | Planned |

### 6.1 Touch Sensor Behavior (Push-to-Talk)
- **Sensor**: TTP223 capacitive touch sensor
- **GPIO**: 1 (active HIGH)
- **Debounce**: Software debounce (200ms)
- **Detection**: Interrupt on RISING edge
- **Mode**: Push-to-talk with automatic silence detection

#### Recording Flow
1. **Touch sensor** → Recording starts (Idle → Listening → Streaming)
2. **Speak** → "Speech detected" when audio exceeds threshold
3. **Stop speaking** → Auto-stops after 1.5s of silence
4. **Timeout** → Auto-stops after 10 seconds max
5. **Manual stop** → Touch again (after 500ms minimum)

#### Silence Detection Parameters
| Parameter | Value | Description |
|-----------|-------|-------------|
| `SILENCE_THRESHOLD` | 500 | Audio level threshold (0-32767) |
| `SILENCE_DURATION_MS` | 1500 | Stop after this much silence |
| `MIN_SPEECH_MS` | 300 | Minimum speech before silence detection |
| `MAX_RECORDING_MS` | 10000 | Maximum recording time (10 sec) |

## 7. Functional Requirements

### 7.1 Voice Interaction
- [x] Capture audio via MSM261 PDM microphone (I2S_NUM_0)
- [x] Stream raw PCM audio to host (16kHz, 16-bit) via TCP
- [x] Silence detection for automatic recording stop
- [x] Receive audio responses from host (8kHz, 16-bit) via TCP
- [x] Play audio through NS4168 I2S amplifier (I2S_NUM_1, 32-bit stereo)

### 7.2 Emotion Display
- [x] Display animated eye graphics on OLED (RoboEyes library)
- [x] Support 10 emotion states with unique eye styles
- [x] LED color/pattern mapping to emotions
- [x] Frame-based animation at ~30 FPS
- [x] Button input triggers recording (GPIO 1)

### 7.3 Head Movement
- [ ] Pan servo control (left/right, 0-180°)
- [ ] Tilt servo control (up/down, 0-180°)
- [ ] Synchronized movement with emotion states
- [ ] Smooth motion interpolation

### 7.4 Host Communication
- [x] Serial debug/command interface (JSON, 115200 baud)
- [x] WiFi TCP audio streaming (port 8266)
- [x] Transport abstraction (serial/TCP auto-detection)
- [x] NVS-stored WiFi credentials
- [x] mDNS service discovery (hinze.local)
- [x] Status reporting
- [x] Event notifications (button, audio start/end)
- [x] Streaming playback via ring buffer (PSRAM)

## 8. OLED Eye Graphics

### 8.1 Display Specifications
- **Resolution**: 128x64 pixels, monochrome
- **Library**: FluxGarage RoboEyes (hardware-accelerated eye animations)
- **Animation**: Built-in moods, autoblink, look-around, smooth transitions

### 8.2 Implemented Emotions (RoboEyes)
| # | Emotion | Eyes (RoboEyes) | LED |
|---|---------|-----------------|-----|
| 0 | Idle | Default + autoblink + idle look-around | Rainbow cycle |
| 1 | Listening | Large (40x40), round | Blue pulse |
| 2 | Thinking | Small squinted (30x24), looking NW | Yellow pulse |
| 3 | Happy | HAPPY mood + autoblink | Green |
| 4 | Sad | TIRED mood, shorter (28h), looking S | Blue |
| 5 | Angry | ANGRY mood, wide+narrow (40x24) | Red |
| 6 | Surprised | Very large round (44x44) | White |
| 7 | Confused | Asymmetric sizes + anim_confused() | Purple |
| 8 | Sleepy | TIRED mood, very short (16h) + autoblink | Dim orange |
| 9 | Excited | HAPPY mood, large (40x40) + periodic anim_laugh() | Fast rainbow |

## 9. SK6812 LED Behavior

### 9.1 LED Modes
| Mode | Description | Speed |
|------|-------------|-------|
| Solid | Static single color | - |
| Pulse | Fade in/out | 1-2 Hz |
| Rainbow Cycle | Smooth hue rotation through full spectrum | ~5 sec/cycle |
| Rainbow Fast | Quick hue rotation | ~1 sec/cycle |
| Flash | Quick on/off burst | 100ms |

### 9.2 Emotion-to-LED Mapping
| Emotion | LED Mode | Color/Hue |
|---------|----------|-----------|
| Idle | Rainbow Cycle | Full spectrum |
| Listening | Pulse | Blue (240°) |
| Thinking | Pulse | Yellow (60°) |
| Happy | Solid | Green (120°) |
| Sad | Solid | Blue (240°) |
| Angry | Solid | Red (0°) |
| Surprised | Flash | White |
| Confused | Solid | Purple (280°) |
| Sleepy | Solid | Dim Orange (30°) |
| Excited | Rainbow Fast | Full spectrum |

## 10. Communication Protocol

### 10.1 JSON Commands (Host → ESP32)
```json
{"cmd": "emotion", "state": "happy"}     // Set emotion state
{"cmd": "status"}                         // Request status
{"cmd": "record_start"}                   // Start recording (manual)
{"cmd": "record_stop"}                    // Stop recording (manual)
{"cmd": "play_start"}                     // Start streaming playback mode
{"cmd": "play_stop"}                      // Signal end of audio stream
{"cmd": "wifi_config", "ssid": "...", "pass": "..."}  // Store WiFi credentials (NVS)
{"cmd": "wifi_status"}                    // Get WiFi connection info
```

**Available emotions**: `idle`, `listening`, `thinking`, `happy`, `sad`, `angry`, `surprised`, `confused`, `sleepy`, `excited`

### 10.2 JSON Events (ESP32 → Host)
```json
{"event": "ready"}                        // Firmware initialized
{"event": "button", "action": "press"}    // Button pressed
{"event": "audio_start"}                  // Recording started
{"event": "audio_end"}                    // Recording stopped
```

### 10.3 JSON Responses (ESP32 → Host)
```json
{"status": "ok", "version": "0.9", "emotion": "idle", "audio": "idle"}
{"ok": true, "emotion": "happy"}
{"ok": true, "action": "play_ready"}
```

### 10.4 Audio Streaming Protocol
Binary audio packets (same format in both directions):
```
[0xAA][0x55][len_high][len_low][pcm_data...]
```
- **Header**: `0xAA 0x55` (2 bytes)
- **Length**: Big-endian uint16 (2 bytes) - number of PCM bytes
- **Data**: Raw 16-bit signed PCM, little-endian, mono
- **Mic → Host**: 16kHz (via TCP)
- **Host → Speaker**: 8kHz (via TCP)

### 10.5 Streaming Playback Flow
1. Host sends `{"cmd":"play_start"}` → ESP32 enables amp, enters BUFFERING state
2. Host streams binary audio packets (8kHz) via TCP
3. ESP32 writes incoming audio to ring buffer in PSRAM (16,000 samples = 2 sec)
4. After 0.5s prefill (4,000 samples), playback task starts I2S output (PLAYING state)
5. Dedicated playback task (Core 1, priority 2) feeds I2S with blocking writes
6. Host sends `{"cmd":"play_stop"}` → sets stream-end marker
7. ESP32 plays until ring buffer drains, flushes DMA with silence, disables amp

**Ring Buffer**:
- Size: 16,000 samples (32 KB) in PSRAM
- Prefill threshold: 4,000 samples (0.5 sec)
- Underrun recovery: rebuffer 2,000 samples (0.25 sec)
- Blocking writes: back-pressures TCP naturally (no overflow, no drops)
- No maximum duration limit (streaming)

**I2S Output** (I2S_NUM_1):
- Input: 16-bit signed PCM, little-endian, mono, 8kHz
- I2S Output: 32-bit stereo (samples amplified and shifted to upper 16 bits)
- DMA: 8 buffers × 256 samples

### 10.6 Future Commands (Not Yet Implemented)
```json
{"cmd": "servo", "pan": 90, "tilt": 45}   // Head movement
{"cmd": "led", "r": 0, "g": 255, "b": 0}  // Manual LED control
```

## 11. Development & Debugging

### 11.1 Build Environment
- **IDE**: PlatformIO (VS Code extension or CLI)
- **Framework**: Arduino
- **Board**: `lolin_s3_mini` (generic S3, works with SPK board pin customization)

### 11.2 Serial Connection
The ESP32-S3 SPK board uses a UART bridge chip (CH340/CP2102):
- **Linux**: Appears as `/dev/ttyUSB0`
- **Baud Rate**: 115200
- **Important**: Opening the serial port toggles DTR, which resets the ESP32
  - Fix: Use `--dtr 0 --rts 0` with miniterm, or `stty -hupcl` before opening
  - In Python: Set `serial.Serial(dtr=False, rts=False)` before `open()`

### 11.3 PlatformIO Commands
```bash
pio run                        # Build firmware
pio run -t upload              # Flash to ESP32
pio run -t upload -t monitor   # Build + flash + monitor
pio run -t clean               # Clean build files
```

### 11.4 platformio.ini Configuration
```ini
[env:esp32-s3-spk]
platform = espressif32
board = lolin_s3_mini
framework = arduino

monitor_speed = 115200

; Flash and PSRAM configuration for ESP32-S3 SPK board (16MB flash, 8MB PSRAM)
board_build.partitions = default_16MB.csv
board_upload.flash_size = 16MB
board_build.arduino.memory_type = qio_qspi

; Build flags (UART bridge board, not native USB CDC)
build_flags =
    -DARDUINO_USB_MODE=0
    -DARDUINO_USB_CDC_ON_BOOT=0
    -DBOARD_HAS_PSRAM
    -mfix-esp32-psram-cache-issue

lib_deps =
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
    fastled/FastLED@3.6.0
    bblanchon/ArduinoJson@^7.0.0
    fluxgarage/FluxGarage RoboEyes
```

### 11.5 Debug Output (Boot)
```
=================================
  Hinze Robot Companion v0.9
  ESP32-S3 SPK Board
  Dual I2S + WiFi Streaming
=================================
[INIT] PSRAM found: 8388608 bytes
[INIT] Ring buffer: 16000 samples (32 KB)
[INIT] Free heap: 280000
[INIT] Setting up OLED...
[OLED] Display initialized
[INIT] Setting up LED...
[LED] SK6812 on GPIO 21
[INIT] Setting up touch sensor...
[TOUCH] TTP223 on GPIO 1 (interrupt enabled)
[INIT] Setting up PDM microphone...
[I2S] PDM Microphone configured: 16000Hz, 16-bit
[I2S] PDM pins: CLK=39, DATA=38
[INIT] Setting up I2S speaker...
[I2S] Speaker configured: 8000Hz, 32-bit stereo
[I2S] Spk pins: BCK=10, WS=45, DOUT=9
[AMP] Control pin GPIO 46 (off)
[WIFI] Connecting to 'MyNetwork'...
[WIFI] Connected! IP: 192.168.1.42
[WIFI] mDNS: hinze.local:8266
[WIFI] TCP server listening on port 8266
[WIFI] Network task started on core 0
[INIT] Playback task started on core 1
[INIT] Setup complete!
{"event":"ready"}
```

### 11.6 Troubleshooting

| Issue | Solution |
|-------|----------|
| Device not detected | Check USB cable; board appears as /dev/ttyUSB0 (not ttyACM0) |
| Upload fails | Hold BOOT button, plug USB, release — forces ROM bootloader |
| ESP32 resets on serial connect | Set DTR=false, RTS=false before opening port |
| WiFi not connecting | NVS credentials are per-chip — send `wifi_config` command |
| No mic audio (level 0) | Verify PDM mode, check CLK=39 and DATA=38 not swapped |
| No speaker audio | Check amp CTRL pin (GPIO 46) — must be HIGH for output |
| Crash during playback | Check playback task stack (need 8192+), check free heap |

## 12. Host Application (Python)

### 12.1 Directory Structure
```
host/
├── hinze_host.py      # Main application
├── config.py          # Configuration template
├── config_local.py    # Local config (API keys, not in git)
└── requirements.txt   # Python dependencies
```

### 12.2 Dependencies
```
pyserial>=3.5
openai-whisper>=20231117
anthropic>=0.21.0
ollama>=0.4.0
openai>=1.0.0
piper-tts>=1.2.0
numpy>=1.24.0
sounddevice>=0.4.6
```

### 12.3 Running the Host
```bash
cd host
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
cp config.py config_local.py   # Edit with your settings

# Connection modes:
python hinze_host.py              # Auto: try TCP first, fall back to serial
python hinze_host.py --tcp        # Force TCP connection
python hinze_host.py --serial     # Force serial connection
python hinze_host.py --host 192.168.1.42  # Specific IP address
```

**Note**: On first run, Whisper will download the model (~140MB for "base").

### 12.4 Host Components
| Component | Class | Purpose |
|-----------|-------|---------|
| Serial Handler | `SerialHandler` | Serial connection (debug + commands, DTR-safe) |
| TCP Handler | `TCPHandler` | WiFi TCP connection (primary for audio) |
| Audio Buffer | `AudioBuffer` | Collects incoming PCM samples |
| Speech Recognition | `SpeechRecognizer` | Whisper transcription |
| Conversation | `ConversationEngine` | Multi-backend LLM with emotion extraction |
| Text-to-Speech | `TextToSpeech` | Piper TTS with streaming synthesis |

### 12.5 AI Personality
The LLM is configured with a personality prompt that instructs it to:
- Keep responses brief (1-2 sentences)
- Be friendly and slightly playful
- End every response with an emotion tag: `[happy]`, `[sad]`, etc.

---
*Document Version: 2.0*
*Created: 2026-01-31*
*Last Updated: 2026-02-17*
