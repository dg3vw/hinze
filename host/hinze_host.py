#!/usr/bin/env python3
"""
Hinze Robot Companion - Host Application

This application connects to the Hinze ESP32 robot via serial,
handles speech recognition (Whisper), AI responses (Claude),
and text-to-speech (Piper).

Usage:
    python hinze_host.py [--port /dev/ttyACM0]
"""

import argparse
import json
import re
import struct
import sys
import threading
import time
from queue import Queue, Empty

import numpy as np
import serial

# Try to import optional dependencies
try:
    import whisper
    WHISPER_AVAILABLE = True
except ImportError:
    WHISPER_AVAILABLE = False
    print("Warning: whisper not installed. Speech recognition disabled.")

try:
    import anthropic
    ANTHROPIC_AVAILABLE = True
except ImportError:
    ANTHROPIC_AVAILABLE = False

try:
    import ollama
    OLLAMA_AVAILABLE = True
except ImportError:
    OLLAMA_AVAILABLE = False

try:
    from openai import OpenAI
    OPENAI_AVAILABLE = True  # Used for OpenRouter
except ImportError:
    OPENAI_AVAILABLE = False

try:
    from piper import PiperVoice
    PIPER_AVAILABLE = True
except ImportError:
    PIPER_AVAILABLE = False
    print("Warning: piper-tts not installed. Text-to-speech disabled.")

# Load configuration
try:
    from config_local import *
except ImportError:
    from config import *


class AudioBuffer:
    """Collects audio samples from serial packets."""

    def __init__(self, sample_rate=SAMPLE_RATE):
        self.sample_rate = sample_rate
        self.samples = []
        self.lock = threading.Lock()

    def add_samples(self, data: bytes):
        """Add raw PCM samples (16-bit signed, little-endian)."""
        with self.lock:
            samples = np.frombuffer(data, dtype=np.int16)
            self.samples.extend(samples)

    def get_audio(self) -> np.ndarray:
        """Get collected audio as float32 normalized array."""
        with self.lock:
            if not self.samples:
                return np.array([], dtype=np.float32)
            audio = np.array(self.samples, dtype=np.float32) / 32768.0
            return audio

    def clear(self):
        """Clear the buffer."""
        with self.lock:
            self.samples = []

    def duration(self) -> float:
        """Get duration of collected audio in seconds."""
        with self.lock:
            return len(self.samples) / self.sample_rate


class SerialHandler:
    """Handles serial communication with ESP32."""

    HEADER_1 = 0xAA
    HEADER_2 = 0x55

    def __init__(self, port: str, baud: int = SERIAL_BAUD):
        self.port = port
        self.baud = baud
        self.serial = None
        self.running = False
        self.event_queue = Queue()
        self.audio_buffer = AudioBuffer()
        self.read_thread = None

    def connect(self) -> bool:
        """Connect to serial port."""
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0.1)
            time.sleep(2)  # Wait for ESP32 reset
            self.serial.reset_input_buffer()
            print(f"[SERIAL] Connected to {self.port}")
            return True
        except serial.SerialException as e:
            print(f"[SERIAL] Error connecting: {e}")
            return False

    def disconnect(self):
        """Disconnect from serial port."""
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
        print("[SERIAL] Disconnected")

    def start_reading(self):
        """Start background thread to read serial data."""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        """Background thread: read serial data."""
        json_buffer = ""

        while self.running and self.serial:
            try:
                # Check for audio packet header
                byte = self.serial.read(1)
                if not byte:
                    continue

                if byte[0] == self.HEADER_1:
                    # Possible audio packet
                    byte2 = self.serial.read(1)
                    if byte2 and byte2[0] == self.HEADER_2:
                        # Read length (2 bytes, big-endian)
                        len_bytes = self.serial.read(2)
                        if len(len_bytes) == 2:
                            data_len = (len_bytes[0] << 8) | len_bytes[1]
                            # Read audio data
                            audio_data = self.serial.read(data_len)
                            if len(audio_data) == data_len:
                                self.audio_buffer.add_samples(audio_data)
                                if DEBUG:
                                    print(f"[AUDIO] Received {data_len} bytes", end='\r')
                    else:
                        # Not an audio packet, treat as JSON
                        json_buffer += chr(byte[0])
                        if byte2:
                            json_buffer += chr(byte2[0])
                else:
                    # Regular character (JSON)
                    char = chr(byte[0])
                    if char == '\n':
                        if json_buffer.strip():
                            self._process_json(json_buffer.strip())
                        json_buffer = ""
                    else:
                        json_buffer += char

            except serial.SerialException as e:
                print(f"[SERIAL] Read error: {e}")
                break

    def _process_json(self, line: str):
        """Process a JSON line from serial."""
        try:
            data = json.loads(line)
            if "event" in data:
                self.event_queue.put(data)
                if DEBUG:
                    print(f"[EVENT] {data}")
            elif DEBUG:
                print(f"[JSON] {data}")
        except json.JSONDecodeError:
            if DEBUG:
                print(f"[RAW] {line}")

    def send_command(self, cmd: dict):
        """Send a JSON command to ESP32."""
        if self.serial:
            json_str = json.dumps(cmd) + "\n"
            self.serial.write(json_str.encode())
            if DEBUG:
                print(f"[SEND] {cmd}")

    def set_emotion(self, emotion: str):
        """Set emotion on ESP32."""
        self.send_command({"cmd": "emotion", "state": emotion})

    def get_status(self):
        """Request status from ESP32."""
        self.send_command({"cmd": "status"})

    def start_recording(self):
        """Start audio recording."""
        self.audio_buffer.clear()
        self.send_command({"cmd": "record_start"})

    def stop_recording(self):
        """Stop audio recording."""
        self.send_command({"cmd": "record_stop"})

    def wait_for_event(self, event_name: str, timeout: float = 10.0) -> dict | None:
        """Wait for a specific event."""
        start = time.time()
        while time.time() - start < timeout:
            try:
                event = self.event_queue.get(timeout=0.1)
                if event.get("event") == event_name:
                    return event
            except Empty:
                continue
        return None


class SpeechRecognizer:
    """Transcribes audio using Whisper."""

    def __init__(self, model_name: str = WHISPER_MODEL):
        self.model = None
        self.model_name = model_name

    def load(self):
        """Load Whisper model."""
        if not WHISPER_AVAILABLE:
            print("[WHISPER] Not available")
            return False
        print(f"[WHISPER] Loading model '{self.model_name}'...")
        self.model = whisper.load_model(self.model_name)
        print("[WHISPER] Model loaded")
        return True

    def transcribe(self, audio: np.ndarray) -> str:
        """Transcribe audio to text."""
        if not self.model:
            return ""
        result = self.model.transcribe(
            audio,
            language=WHISPER_LANGUAGE,
            fp16=False
        )
        return result["text"].strip()


class ConversationEngine:
    """Handles conversation with multiple LLM backends."""

    def __init__(self):
        self.backend = getattr(sys.modules[__name__], 'LLM_BACKEND', 'ollama')
        self.client = None
        self.conversation_history = []

    def connect(self) -> bool:
        """Initialize LLM client based on configured backend."""
        try:
            backend = self.backend
        except:
            backend = "ollama"

        if backend == "anthropic":
            return self._connect_anthropic()
        elif backend == "ollama":
            return self._connect_ollama()
        elif backend == "openrouter":
            return self._connect_openrouter()
        else:
            print(f"[LLM] Unknown backend: {backend}")
            return False

    def _connect_anthropic(self) -> bool:
        if not ANTHROPIC_AVAILABLE:
            print("[ANTHROPIC] Not available - install: pip install anthropic")
            return False
        try:
            self.client = anthropic.Anthropic(api_key=ANTHROPIC_API_KEY)
            print("[ANTHROPIC] Connected")
            return True
        except Exception as e:
            print(f"[ANTHROPIC] Error: {e}")
            return False

    def _connect_ollama(self) -> bool:
        if not OLLAMA_AVAILABLE:
            print("[OLLAMA] Not available - install: pip install ollama")
            return False
        try:
            # Test connection by listing models
            ollama.list()
            print(f"[OLLAMA] Connected to {OLLAMA_HOST}")
            return True
        except Exception as e:
            print(f"[OLLAMA] Error connecting: {e}")
            print("[OLLAMA] Make sure Ollama is running: ollama serve")
            return False

    def _connect_openrouter(self) -> bool:
        if not OPENAI_AVAILABLE:
            print("[OPENROUTER] Not available - install: pip install openai")
            return False
        try:
            self.client = OpenAI(
                base_url="https://openrouter.ai/api/v1",
                api_key=OPENROUTER_API_KEY
            )
            print(f"[OPENROUTER] Connected")
            return True
        except Exception as e:
            print(f"[OPENROUTER] Error: {e}")
            return False

    def chat(self, user_message: str) -> tuple[str, str]:
        """Send message to LLM, get response and emotion."""
        self.conversation_history.append({
            "role": "user",
            "content": user_message
        })

        try:
            backend = self.backend
        except:
            backend = "ollama"

        try:
            if backend == "anthropic":
                response = self._chat_anthropic()
            elif backend == "ollama":
                response = self._chat_ollama()
            elif backend == "openrouter":
                response = self._chat_openrouter()
            else:
                return "I don't know how to think.", "confused"

            self.conversation_history.append({
                "role": "assistant",
                "content": response
            })

            # Keep conversation history reasonable
            if len(self.conversation_history) > 20:
                self.conversation_history = self.conversation_history[-10:]

            return self._extract_emotion(response)

        except Exception as e:
            print(f"[LLM] Error: {e}")
            return "I'm having trouble thinking right now.", "confused"

    def _chat_anthropic(self) -> str:
        response = self.client.messages.create(
            model=ANTHROPIC_MODEL,
            max_tokens=150,
            system=SYSTEM_PROMPT,
            messages=self.conversation_history
        )
        return response.content[0].text

    def _chat_ollama(self) -> str:
        # Build messages with system prompt
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(self.conversation_history)

        response = ollama.chat(
            model=OLLAMA_MODEL,
            messages=messages
        )
        return response['message']['content']

    def _chat_openrouter(self) -> str:
        # Build messages with system prompt
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(self.conversation_history)

        response = self.client.chat.completions.create(
            model=OPENROUTER_MODEL,
            max_tokens=150,
            messages=messages
        )
        return response.choices[0].message.content

    def _extract_emotion(self, text: str) -> tuple[str, str]:
        """Extract emotion tag from response."""
        pattern = r'\[(\w+)\]\s*$'
        match = re.search(pattern, text)

        if match:
            emotion = match.group(1).lower()
            clean_text = text[:match.start()].strip()
            valid_emotions = [
                "idle", "listening", "thinking", "happy", "sad",
                "angry", "surprised", "confused", "sleepy", "excited"
            ]
            if emotion in valid_emotions:
                return clean_text, emotion

        return text, "idle"


class TextToSpeech:
    """Generates speech audio using Piper."""

    def __init__(self, voice: str = PIPER_VOICE):
        self.voice_name = voice
        self.voice = None

    def load(self) -> bool:
        """Load Piper voice model."""
        if not PIPER_AVAILABLE:
            print("[PIPER] Not available")
            return False
        try:
            self.voice = PiperVoice.load(self.voice_name)
            print(f"[PIPER] Loaded voice '{self.voice_name}'")
            return True
        except Exception as e:
            print(f"[PIPER] Error loading voice: {e}")
            return False

    def synthesize(self, text: str) -> np.ndarray:
        """Generate audio from text."""
        if not self.voice:
            return np.array([], dtype=np.int16)

        audio_data = []
        for audio_bytes in self.voice.synthesize_stream_raw(text):
            audio_data.append(np.frombuffer(audio_bytes, dtype=np.int16))

        if audio_data:
            return np.concatenate(audio_data)
        return np.array([], dtype=np.int16)


class HinzeHost:
    """Main application class."""

    def __init__(self, port: str):
        self.serial = SerialHandler(port)
        self.recognizer = SpeechRecognizer()
        self.conversation = ConversationEngine()
        self.tts = TextToSpeech()
        self.running = False

    def setup(self) -> bool:
        """Initialize all components."""
        print("\n=== Hinze Host Application ===\n")

        # Connect to ESP32
        if not self.serial.connect():
            return False

        # Start serial reading
        self.serial.start_reading()

        # Wait for ready event
        print("[HOST] Waiting for ESP32 ready...")
        event = self.serial.wait_for_event("ready", timeout=5.0)
        if not event:
            print("[HOST] Warning: No ready event received")

        # Load speech recognition
        self.recognizer.load()

        # Connect to Claude
        self.conversation.connect()

        # Load TTS
        self.tts.load()

        print("\n[HOST] Setup complete!")
        print("[HOST] Press the button on Hinze to start talking.\n")

        return True

    def run(self):
        """Main loop."""
        self.running = True
        self.serial.set_emotion("idle")

        try:
            while self.running:
                # Wait for button press event
                event = self.serial.wait_for_event("button", timeout=1.0)

                if event and event.get("action") == "press":
                    self._handle_interaction()

        except KeyboardInterrupt:
            print("\n[HOST] Shutting down...")

        self.serial.disconnect()

    def _handle_interaction(self):
        """Handle a complete interaction cycle."""
        print("\n[HOST] === New Interaction ===")

        # 1. Recording phase (button already triggered recording on ESP32)
        print("[HOST] Recording... (press button to stop)")
        self.serial.set_emotion("listening")

        # Wait for audio_end event
        event = self.serial.wait_for_event("audio_end", timeout=RECORDING_TIMEOUT)
        if not event:
            print("[HOST] Recording timeout")
            self.serial.stop_recording()

        # Get audio duration
        duration = self.serial.audio_buffer.duration()
        print(f"[HOST] Captured {duration:.1f}s of audio")

        if duration < 0.5:
            print("[HOST] Audio too short, ignoring")
            self.serial.set_emotion("idle")
            return

        # 2. Transcription phase
        print("[HOST] Transcribing...")
        self.serial.set_emotion("thinking")

        audio = self.serial.audio_buffer.get_audio()
        transcript = self.recognizer.transcribe(audio)

        if not transcript:
            print("[HOST] No speech detected")
            self.serial.set_emotion("confused")
            time.sleep(1)
            self.serial.set_emotion("idle")
            return

        print(f"[HOST] Transcript: \"{transcript}\"")

        # 3. Get Claude response
        print("[HOST] Thinking...")
        response, emotion = self.conversation.chat(transcript)
        print(f"[HOST] Response: \"{response}\" [{emotion}]")

        # 4. Set emotion and speak
        self.serial.set_emotion(emotion)

        # Generate and play TTS (if available)
        if PIPER_AVAILABLE and self.tts.voice:
            print("[HOST] Speaking...")
            audio = self.tts.synthesize(response)
            # TODO: Send audio to ESP32 for playback
            # For now, just wait a bit
            time.sleep(len(response) * 0.05)

        # 5. Return to idle
        time.sleep(2)
        self.serial.set_emotion("idle")
        print("[HOST] === Interaction Complete ===\n")


def main():
    parser = argparse.ArgumentParser(description="Hinze Robot Companion Host")
    parser.add_argument("--port", "-p", default=SERIAL_PORT,
                        help=f"Serial port (default: {SERIAL_PORT})")
    args = parser.parse_args()

    app = HinzeHost(args.port)

    if not app.setup():
        print("[HOST] Setup failed!")
        sys.exit(1)

    app.run()


if __name__ == "__main__":
    main()
