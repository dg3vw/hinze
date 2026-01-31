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
| Component | Interface | Purpose |
|-----------|-----------|---------|
| Button | GPIO 0 (active low) | Activation trigger |

## 3. Pin Assignment (ESP32-S3 SuperMini)

### 3.1 I2S Audio (Shared Bus)
| Function | GPIO | Component |
|----------|------|-----------|
| I2S_WS (Word Select) | 4 | INMP441 + MAX98357A |
| I2S_BCK (Bit Clock) | 5 | INMP441 + MAX98357A |
| I2S_DIN (Mic Data In) | 6 | INMP441 SD |
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
| BUTTON | 0 | Activation button (active low, internal pullup) |

## 4. Communication

### 4.1 Host Connection
- **USB Serial** - Primary connection for development and reliable operation
- **WiFi** - Wireless mode for standalone deployment

### 4.2 Protocol
- **Commands**: JSON format for control messages
- **Audio Streaming**: Raw PCM data (16kHz, 16-bit, mono)
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
- **Communication**: USB Serial + WiFi (switchable modes)
- **Audio Format**: I2S input/output, 16kHz sample rate, 16-bit depth
- **Protocol**: JSON commands for control, raw PCM for audio streaming

### 5.3 Host Application (Python)
| Component | Technology | Purpose |
|-----------|------------|---------|
| Speech-to-Text | Whisper (local) | Convert user speech to text |
| AI Backend | Claude API | Process queries, generate responses |
| Text-to-Speech | Piper TTS (local) | Convert response text to audio |
| Communication | pyserial / websockets | Serial or WiFi link to ESP32 |

### 5.4 Data Flow
1. User speaks → INMP441 captures audio
2. ESP32 streams raw PCM audio to Host PC
3. Whisper transcribes speech to text
4. Claude API processes query, returns response + emotion tag
5. Piper TTS generates audio from response text
6. Host sends: audio stream + emotion command to ESP32
7. ESP32 plays audio via MAX98357A, updates display/LED/servos

## 6. Activation Modes

| Mode | Description | Use Case |
|------|-------------|----------|
| Wake Word | Local detection of "Hey Hinze" | Hands-free, privacy-focused |
| Always Listening | Continuous speech detection | Quick response, demo mode |
| Button Press | GPIO 0 button triggers listening | Battery saving, controlled |

## 7. Functional Requirements

### 7.1 Voice Interaction
- [ ] Capture audio via INMP441 I2S microphone
- [ ] Stream raw PCM audio to host (16kHz, 16-bit)
- [ ] Receive audio responses from host
- [ ] Play audio through MAX98357A I2S amplifier

### 7.2 Emotion Display
- [ ] Display animated eye graphics on OLED
- [ ] Support 10 emotion states with smooth transitions
- [ ] LED color/pattern mapping to emotions
- [ ] Frame-based animation at 10-15 FPS

### 7.3 Head Movement
- [ ] Pan servo control (left/right, 0-180°)
- [ ] Tilt servo control (up/down, 0-180°)
- [ ] Synchronized movement with emotion states
- [ ] Smooth motion interpolation

### 7.4 Host Communication
- [ ] USB serial command interface (JSON)
- [ ] WiFi/WebSocket interface (JSON)
- [ ] Status reporting
- [ ] Audio streaming in both directions

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

### 8.3 Animation Frames per Emotion
| Emotion | Frames | Animation |
|---------|--------|-----------|
| Idle | 4 | Slow blink cycle |
| Listening | 2 | Wide open, slight pulse |
| Thinking | 3 | Eyes roll up, dots appear |
| Happy | 3 | Eyes curve into ^ ^ shape |
| Sad | 2 | Droopy with tear drop |
| Angry | 2 | Narrowed, angled down |
| Surprised | 2 | Maximum open, round |
| Confused | 3 | Asymmetric, one raised |
| Sleepy | 4 | Gradual closing, Z's |
| Excited | 4 | Sparkle effect, bouncy |

## 9. Emotion States

| # | Emotion | OLED Eyes | LED Color | Head Position |
|---|---------|-----------|-----------|---------------|
| 1 | Idle | Neutral eyes, slow blink | Dim white | Center |
| 2 | Listening | Wide open eyes | Blue pulse | Slight tilt toward sound |
| 3 | Thinking | Squinted eyes, dots | Yellow pulse | Look up-left |
| 4 | Happy | Curved smile eyes (^ ^) | Green | Gentle nod |
| 5 | Sad | Droopy eyes, tear | Blue | Look down |
| 6 | Angry | Furrowed brows, narrow | Red | Shake side-to-side |
| 7 | Surprised | Very wide, raised brows | White flash | Quick tilt back |
| 8 | Confused | One raised brow, tilted | Purple | Head tilt |
| 9 | Sleepy | Half-closed, yawn | Dim orange | Slow droop down |
| 10 | Excited | Sparkle eyes, bouncy | Rainbow cycle | Quick nods |

## 10. JSON Command Protocol

### 10.1 Commands (Host → ESP32)
```json
{"cmd": "emotion", "state": "happy"}
{"cmd": "audio_start", "sample_rate": 16000}
{"cmd": "audio_stop"}
{"cmd": "servo", "pan": 90, "tilt": 45}
{"cmd": "led", "r": 0, "g": 255, "b": 0}
{"cmd": "status"}
```

### 10.2 Responses (ESP32 → Host)
```json
{"status": "ok", "emotion": "idle", "listening": false}
{"event": "button_press"}
{"event": "audio_ready"}
```

---
*Document Version: 1.0*
*Created: 2026-01-31*
*Last Updated: 2026-01-31*
