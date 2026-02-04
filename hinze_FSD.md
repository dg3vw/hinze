# Hinze - Functional Specification Document

## 1. Overview

Hinze is an interactive robot companion based on ESP32-S3, featuring voice interaction, emotional expressions, and head movement capabilities. The system uses a split architecture with the ESP32 handling hardware I/O and a host PC running AI processing (Whisper for speech recognition, Claude API for conversation, and Piper TTS for speech synthesis).

## 2. Hardware Components

### 2.1 Main Controller
- **ESP32-S3 SuperMini** - Compact development board with WiFi/Bluetooth 5.0

### 2.2 Audio System
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Microphone | INMP441 | I2S | Voice command input |
| Amplifier | MAX98357A | I2S | Audio output/speech |

### 2.3 Visual Display
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| OLED Display | 0.96" 128x64 SSD1306 | I2C | Emotion display (animated eyes) |
| RGB LED | WS2812 (1x) | GPIO data | Emotion color indication |

### 2.4 Motion System
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Servo Pan | SG90 | PWM | Head horizontal movement |
| Servo Tilt | SG90 | PWM | Head vertical movement |

### 2.5 User Input
| Component | Model | Interface | Purpose |
|-----------|-------|-----------|---------|
| Touch Sensor | TTP223 | GPIO 10 (active HIGH) | Push-to-talk activation |

## 3. Pin Assignment (ESP32-S3 SuperMini)

### 3.1 I2S Audio (Shared Bus)
| Function | GPIO | Component |
|----------|------|-----------|
| I2S_DIN (Mic Data In) | 4 | INMP441 SD |
| I2S_WS (Word Select) | 5 | INMP441 + MAX98357A |
| I2S_BCK (Bit Clock) | 6 | INMP441 + MAX98357A |
| I2S_DOUT (Amp Data Out) | 7 | MAX98357A DIN |

### 3.2 I2C Display
| Function | GPIO | Component |
|----------|------|-----------|
| I2C_SDA | 8 | SSD1306 OLED |
| I2C_SCL | 9 | SSD1306 OLED |

### 3.3 Control Outputs
| Function | GPIO | Component |
|----------|------|-----------|
| SERVO_PAN | 1 | SG90 (horizontal) |
| SERVO_TILT | 2 | SG90 (vertical) |
| WS2812_DATA | 48 | RGB LED |

### 3.4 User Input
| Function | GPIO | Component |
|----------|------|-----------|
| TOUCH_PIN | 10 | TTP223 capacitive touch sensor (active HIGH) |

## 4. Communication

### 4.1 Host Connection
- **USB Serial** - Primary connection for development and reliable operation
- **WiFi** - Wireless mode for standalone deployment

### 4.2 Protocol
- **Commands**: JSON format for control messages
- **Audio Capture**: Raw PCM data (16kHz, 16-bit, mono) - ESP32 → Host
- **Audio Playback**: Raw PCM data (8kHz, 16-bit, mono) - Host → ESP32
- **Status Updates**: JSON status messages from ESP32

## 5. Software Architecture

### 5.1 System Overview
```
┌─────────────────────────────────────────────────────────┐
│                    Host PC (Python)                      │
├─────────────────┬─────────────────┬─────────────────────┤
│ Whisper (local) │   Claude API    │   Piper TTS (local) │
│ Speech-to-Text  │   AI Processing │   Text-to-Speech    │
└────────┬────────┴────────┬────────┴──────────┬──────────┘
         │                 │                    │
         └────────────────┬┴───────────────────┘
                          │ USB Serial / WiFi
┌─────────────────────────┴───────────────────────────────┐
│              ESP32-S3 SuperMini (Arduino)               │
├───────────┬───────────┬───────────┬───────────┬─────────┤
│ INMP441   │ MAX98357A │ SSD1306   │ 2x SG90   │ WS2812  │
│ Mic In    │ Audio Out │ Emotions  │ Head Move │ LED     │
└───────────┴───────────┴───────────┴───────────┴─────────┘
```

### 5.2 ESP32 Firmware (Arduino/PlatformIO)
- **Framework**: Arduino with PlatformIO build system
- **Communication**: USB Serial (921600 baud) + WiFi (switchable modes)
- **Audio Input**: I2S microphone, 16kHz, 32-bit frame
- **Audio Output**: I2S speaker, 8kHz, 32-bit frame (buffer-then-play)
- **Protocol**: JSON commands for control, raw PCM for audio streaming

### 5.3 Host Application (Python)
| Component | Technology | Purpose |
|-----------|------------|---------|
| Speech-to-Text | Whisper (local) | Convert user speech to text |
| AI Backend | Claude API | Process queries, generate responses |
| Text-to-Speech | Piper TTS (local) | Convert response text to audio |
| Communication | pyserial / websockets | Serial or WiFi link to ESP32 |

### 5.4 Data Flow
1. User touches sensor → ESP32 starts recording
2. INMP441 captures audio → ESP32 streams 16kHz PCM to Host
3. Whisper transcribes speech to text (German/English)
4. LLM processes query, returns response + emotion tag (e.g., `[happy]`)
5. Piper TTS generates audio (22kHz) → resampled to 8kHz
6. Host sends: emotion command + audio packets to ESP32
7. ESP32 buffers audio, switches to speaker mode, plays via MAX98357A
8. ESP32 updates OLED eyes and LED color to match emotion

## 6. Activation Modes

| Mode | Description | Status |
|------|-------------|--------|
| Button Press | Push-to-talk with silence detection | **Implemented** |
| Wake Word | Local detection of "Hey Hinze" | Planned |
| Always Listening | Continuous speech detection | Planned |

### 6.1 Touch Sensor Behavior (Push-to-Talk)
- **Sensor**: TTP223 capacitive touch sensor
- **GPIO**: 10 (active HIGH)
- **Debounce**: Software debounce (200ms)
- **Detection**: Interrupt on RISING edge
- **Mode**: Push-to-talk with automatic silence detection

#### TTP223 Wiring
| TTP223 Pin | ESP32-S3 |
|------------|----------|
| VCC | 3.3V |
| GND | GND |
| SIG/OUT | GPIO 10 |

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
- [x] Capture audio via INMP441 I2S microphone
- [x] Stream raw PCM audio to host (16kHz, 16-bit)
- [x] Silence detection for automatic recording stop
- [x] Receive audio responses from host (8kHz, 16-bit)
- [x] Play audio through MAX98357A I2S amplifier (32-bit I2S)

### 7.2 Emotion Display
- [x] Display animated eye graphics on OLED
- [x] Support 10 emotion states with unique eye styles
- [x] LED color/pattern mapping to emotions
- [x] Frame-based animation at ~30 FPS
- [x] Button input triggers recording (GPIO 10)

### 7.3 Head Movement
- [ ] Pan servo control (left/right, 0-180°)
- [ ] Tilt servo control (up/down, 0-180°)
- [ ] Synchronized movement with emotion states
- [ ] Smooth motion interpolation

### 7.4 Host Communication
- [x] USB serial command interface (JSON)
- [x] Status reporting
- [x] Audio streaming (ESP32 → Host)
- [x] Event notifications (button, audio start/end)
- [x] Audio streaming (Host → ESP32)
- [ ] WiFi/WebSocket interface (JSON)

## 8. OLED Eye Graphics

### 8.1 Display Specifications
- **Resolution**: 128x64 pixels, monochrome
- **Layout**: Two eyes side by side (each ~48x48 pixels, 16px gap)
- **Style**: Blocky pixel art, retro robot aesthetic
- **Animation**: Frame-based sprites, 10-15 FPS

### 8.2 Eye Structure (per eye)
```
┌────────────────┐
│  ██████████    │  <- Upper eyelid (adjustable)
│ ████████████   │
│ ██  ████  ██   │  <- Pupil position (movable)
│ ████████████   │
│  ██████████    │  <- Lower eyelid (adjustable)
└────────────────┘
```

### 8.3 Idle State (Natural Eye Behavior)
The idle state features lifelike eye animation:
- **Looking Around**: Pupils move randomly to different positions (left, right, up, down, center)
- **Natural Blinking**: Random blink interval (2-6 seconds), quick close/open cycle
- **Smooth Transitions**: Pupil position changes smoothly, not instant jumps

```
Idle Animation Cycle:
┌─────────────────────────────────────────────────────────┐
│  [Look center] → [pause 1-3s] → [Look random dir]       │
│       ↑                              ↓                  │
│       └──── [Blink] ←── [pause 2-6s] ←──────────────────│
└─────────────────────────────────────────────────────────┘
```

### 8.4 Implemented Eye Animations
| Emotion | Eye Style | Animation Effect |
|---------|-----------|------------------|
| Idle | Round eyes with pupils | Look around randomly + natural blink (2-6s interval) |
| Listening | Wide open, larger eyes | Subtle size pulse |
| Thinking | Squinted, pupils up-left | Animated dots "..." appearing |
| Happy | Curved ^ ^ anime style | Static happy expression |
| Sad | Droopy eyes + sad brows | Falling tear drop animation |
| Angry | Narrow eyes + angled brows | Intense static glare |
| Surprised | Very wide, tiny pupils | Raised eyebrows above eyes |
| Confused | Asymmetric (one raised) | Question mark "?" displayed |
| Sleepy | Half-closed slit eyes | Floating "Zzz" animation |
| Excited | Large bouncy eyes | Sparkle effects + vertical bounce |

## 9. Emotion States

| # | Emotion | OLED Eyes | LED Color | Head Position |
|---|---------|-----------|-----------|---------------|
| 1 | Idle | Look around + natural blink | Rainbow cycle | Center |
| 2 | Listening | Wide open eyes | Blue pulse | Slight tilt toward sound |
| 3 | Thinking | Squinted eyes, dots | Yellow pulse | Look up-left |
| 4 | Happy | Curved smile eyes (^ ^) | Green | Gentle nod |
| 5 | Sad | Droopy eyes, tear | Blue | Look down |
| 6 | Angry | Furrowed brows, narrow | Red | Shake side-to-side |
| 7 | Surprised | Very wide, raised brows | White flash | Quick tilt back |
| 8 | Confused | One raised brow, tilted | Purple | Head tilt |
| 9 | Sleepy | Half-closed, yawn | Dim orange | Slow droop down |
| 10 | Excited | Sparkle eyes, bouncy | Rainbow fast | Quick nods |

## 10. WS2812 LED Behavior

### 10.1 LED Modes
| Mode | Description | Speed |
|------|-------------|-------|
| Solid | Static single color | - |
| Pulse | Fade in/out | 1-2 Hz |
| Rainbow Cycle | Smooth hue rotation through full spectrum | ~5 sec/cycle |
| Rainbow Fast | Quick hue rotation | ~1 sec/cycle |
| Flash | Quick on/off burst | 100ms |

### 10.2 Idle Rainbow Cycle
```
Hue rotation: 0° → 360° (Red → Yellow → Green → Cyan → Blue → Magenta → Red)
Cycle time: ~5 seconds for full spectrum
Brightness: 50% (not distracting)
Update rate: 20-30 FPS for smooth transition
```

### 10.3 Emotion-to-LED Mapping
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

## 11. Serial Protocol

### 11.1 JSON Commands (Host → ESP32)
```json
{"cmd": "emotion", "state": "happy"}     // Set emotion state
{"cmd": "status"}                         // Request status
{"cmd": "record_start"}                   // Start recording (manual)
{"cmd": "record_stop"}                    // Stop recording (manual)
```

**Available emotions**: `idle`, `listening`, `thinking`, `happy`, `sad`, `angry`, `surprised`, `confused`, `sleepy`, `excited`

### 11.2 JSON Events (ESP32 → Host)
```json
{"event": "ready"}                        // Firmware initialized
{"event": "button", "action": "press"}    // Button pressed
{"event": "audio_start"}                  // Recording started
{"event": "audio_end"}                    // Recording stopped
```

### 11.3 JSON Responses (ESP32 → Host)
```json
{"status": "ok", "version": "0.4", "emotion": "idle", "audio": "idle"}
{"ok": true, "emotion": "happy"}
{"error": "JSON parse failed: InvalidInput"}
```

### 11.4 Audio Streaming Protocol (ESP32 → Host)
Binary audio packets are sent during recording:
```
[0xAA][0x55][len_high][len_low][pcm_data...]
```
- **Header**: `0xAA 0x55` (2 bytes)
- **Length**: Big-endian uint16 (2 bytes) - number of PCM bytes
- **Data**: Raw 16-bit signed PCM, little-endian, mono, 16kHz

### 11.5 Audio Playback Protocol (Host → ESP32)
Audio playback uses a buffer-then-play approach:

1. Host sends `{"cmd":"play_start"}` command
2. Host sends binary audio packets (same format as above)
3. ESP32 buffers all audio data (max ~11 seconds at 8kHz)
4. Host sends `{"cmd":"play_stop"}` command
5. ESP32 plays buffered audio via I2S speaker

**Audio Format**:
- Input: 16-bit signed PCM, little-endian, mono, 8kHz
- I2S Output: 32-bit stereo (16-bit samples shifted left by 16 bits)

**I2S Port Switching**: Since mic and speaker share BCK/WS pins, the firmware dynamically switches:
- Uninstall mic driver → Install speaker driver → Play → Uninstall speaker → Reinstall mic

### 11.6 Future Commands (Not Yet Implemented)
```json
{"cmd": "servo", "pan": 90, "tilt": 45}   // Head movement
{"cmd": "led", "r": 0, "g": 255, "b": 0}  // Manual LED control
{"cmd": "audio_play"}                      // Play audio (followed by PCM data)
```

## 12. Development & Debugging

### 12.1 Build Environment
- **IDE**: PlatformIO (VS Code extension or CLI)
- **Framework**: Arduino
- **Board**: `lolin_s3_mini` (compatible with ESP32-S3 SuperMini)

### 12.2 USB Connection
The ESP32-S3 SuperMini uses native USB with CDC (Communications Device Class):
- **Linux**: Appears as `/dev/ttyACM0`
- **Windows**: COM port (install ESP32-S3 USB driver if needed)
- **macOS**: `/dev/cu.usbmodem*`

### 12.3 PlatformIO Commands
```bash
# Build firmware
pio run

# Upload to device
pio run -t upload

# Monitor serial output (interactive terminal required)
pio device monitor

# Build + Upload + Monitor
pio run -t upload -t monitor

# Clean build files
pio run -t clean

# List connected devices
pio device list
```

### 12.4 Serial Monitor Settings
- **Baud Rate**: 921600 (high speed for audio streaming)
- **Line Ending**: Newline (LF)
- **Encoding**: UTF-8

### 12.5 Troubleshooting

| Issue | Solution |
|-------|----------|
| Device not detected | Check USB cable (must support data, not charge-only) |
| Upload fails | Hold BOOT button while pressing RST, then release |
| No serial output | Verify `monitor_speed = 115200` in platformio.ini |
| I2C device not found | Check wiring, verify pullup resistors (4.7kΩ) |
| I2S no audio | Verify WS/BCK/DATA connections, check sample rate |

### 12.6 Debug Output Format
The firmware outputs initialization status on boot:
```
=================================
  Hinze Robot Companion v0.6
  ESP32-S3 SuperMini
  I2S Mic + Speaker + Serial
=================================
[INIT] Setting up OLED...
[OLED] Display initialized
[INIT] Setting up LED...
[LED] WS2812 on GPIO 48
[INIT] Setting up touch sensor...
[TOUCH] TTP223 on GPIO 10 (interrupt enabled)
[INIT] Setting up I2S microphone...
[I2S] Microphone configured: 16000Hz, 32-bit
[I2S] Pins: WS=5, BCK=6, DIN=4
[INIT] Setup complete!
{"event":"ready"}
```

### 12.7 Runtime Debug Messages
```
{"event":"button","action":"press"}
[AUDIO] Starting recording...
{"event":"audio_start"}
[AUDIO] Speech detected
[AUDIO] Silence detected - stopping
{"event":"audio_end"}
```

### 12.8 platformio.ini Configuration
```ini
[env:esp32-s3-supermini]
platform = espressif32
board = lolin_s3_mini
framework = arduino

monitor_speed = 921600

build_flags =
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1

lib_deps =
    adafruit/Adafruit SSD1306
    adafruit/Adafruit GFX Library
    fastled/FastLED@3.6.0
    bblanchon/ArduinoJson@^7.0.0
```

## 13. Host Application (Python)

### 13.1 Directory Structure
```
host/
├── hinze_host.py      # Main application
├── config.py          # Configuration template
├── config_local.py    # Local config (API keys, not in git)
└── requirements.txt   # Python dependencies
```

### 13.2 Dependencies
```
pyserial>=3.5
openai-whisper>=20231117
anthropic>=0.21.0
piper-tts>=1.2.0
numpy>=1.24.0
sounddevice>=0.4.6
```

### 13.3 Running the Host
```bash
cd host
python3 -m venv venv           # Create virtual environment
source venv/bin/activate       # Activate venv (Linux/macOS)
pip install -r requirements.txt
cp config.py config_local.py   # Edit with your API key
python hinze_host.py --port /dev/ttyACM0
```

**Note**: On first run, Whisper will download the model (~140MB for "base").

### 13.4 Host Components
| Component | Class | Purpose |
|-----------|-------|---------|
| Serial Handler | `SerialHandler` | Manages ESP32 connection, parses packets |
| Audio Buffer | `AudioBuffer` | Collects incoming PCM samples |
| Speech Recognition | `SpeechRecognizer` | Whisper transcription |
| Conversation | `ConversationEngine` | Claude API with emotion extraction |
| Text-to-Speech | `TextToSpeech` | Piper TTS audio generation |

### 13.5 Claude System Prompt
The AI is configured with a personality prompt that instructs it to:
- Keep responses brief (1-2 sentences)
- Be friendly and slightly playful
- End every response with an emotion tag: `[happy]`, `[sad]`, etc.

---
*Document Version: 1.5*
*Created: 2026-01-31*
*Last Updated: 2026-02-03*
