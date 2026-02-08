#!/usr/bin/env python3
"""
Hinze Robot Companion - Host Application

This application connects to the Hinze ESP32 robot via serial or TCP,
handles speech recognition (Whisper), AI responses (Claude/Ollama/etc),
and text-to-speech (Piper) with streaming audio playback.

Usage:
    python hinze_host.py [--port /dev/ttyACM0]
    python hinze_host.py --tcp [--host hinze.local]
    python hinze_host.py --serial
"""

import argparse
import json
import os
import re
import select
import socket
import struct
import sys
import threading
import time
from abc import ABC, abstractmethod
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
    print("[CONFIG] Loaded config_local.py")
except ImportError:
    from config import *
    print("[CONFIG] Loaded config.py (no config_local.py found)")

print(f"[CONFIG] System prompt starts with: {SYSTEM_PROMPT[:160].strip()}...")


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


class ConnectionHandler(ABC):
    """Abstract base class for ESP32 connections."""

    HEADER_1 = 0xAA
    HEADER_2 = 0x55

    def __init__(self):
        self.running = False
        self.event_queue = Queue()
        self.audio_buffer = AudioBuffer()
        self.read_thread = None

    @abstractmethod
    def connect(self) -> bool:
        pass

    @abstractmethod
    def disconnect(self):
        pass

    @abstractmethod
    def _write_bytes(self, data: bytes):
        pass

    @abstractmethod
    def _read_byte(self) -> bytes:
        pass

    @abstractmethod
    def _read_bytes(self, count: int) -> bytes:
        pass

    @abstractmethod
    def _available(self) -> bool:
        pass

    def start_reading(self):
        """Start background thread to read data."""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()

    def _read_loop(self):
        """Background thread: read data from connection."""
        json_buffer = ""

        while self.running:
            try:
                byte = self._read_byte()
                if not byte:
                    continue

                if byte[0] == self.HEADER_1:
                    byte2 = self._read_byte()
                    if byte2 and byte2[0] == self.HEADER_2:
                        len_bytes = self._read_bytes(2)
                        if len(len_bytes) == 2:
                            data_len = (len_bytes[0] << 8) | len_bytes[1]
                            audio_data = self._read_bytes(data_len)
                            if len(audio_data) == data_len:
                                self.audio_buffer.add_samples(audio_data)
                                if DEBUG:
                                    print(f"[AUDIO] Received {data_len} bytes", end='\r')
                    else:
                        json_buffer += chr(byte[0])
                        if byte2:
                            json_buffer += chr(byte2[0])
                else:
                    char = chr(byte[0])
                    if char == '\n':
                        if json_buffer.strip():
                            self._process_json(json_buffer.strip())
                        json_buffer = ""
                    else:
                        json_buffer += char

            except Exception as e:
                if self.running:
                    print(f"[CONN] Read error: {e}")
                break

    def _process_json(self, line: str):
        """Process a JSON line."""
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
        json_str = json.dumps(cmd) + "\n"
        self._write_bytes(json_str.encode())
        if DEBUG:
            print(f"[SEND] {cmd}")

    def set_emotion(self, emotion: str):
        self.send_command({"cmd": "emotion", "state": emotion})

    def get_status(self):
        self.send_command({"cmd": "status"})

    def start_recording(self):
        self.audio_buffer.clear()
        self.send_command({"cmd": "record_start"})

    def stop_recording(self):
        self.send_command({"cmd": "record_stop"})

    def start_playback(self):
        self.send_command({"cmd": "play_start"})

    def stop_playback(self):
        self.send_command({"cmd": "play_stop"})

    def send_audio(self, audio: np.ndarray, chunk_size: int = 1024):
        """Send audio data to ESP32 for playback."""
        if audio.dtype != np.int16:
            audio = (audio * 32767).astype(np.int16)

        for i in range(0, len(audio), chunk_size):
            chunk = audio[i:i + chunk_size]
            data = chunk.tobytes()
            data_len = len(data)

            packet = bytes([
                self.HEADER_1,
                self.HEADER_2,
                (data_len >> 8) & 0xFF,
                data_len & 0xFF
            ]) + data

            self._write_bytes(packet)

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


class SerialHandler(ConnectionHandler):
    """Handles serial communication with ESP32."""

    def __init__(self, port: str, baud: int = SERIAL_BAUD):
        super().__init__()
        self.port = port
        self.baud = baud
        self.serial = None

    def connect(self) -> bool:
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
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.serial:
            self.serial.close()
            self.serial = None
        print("[SERIAL] Disconnected")

    def _write_bytes(self, data: bytes):
        if self.serial:
            self.serial.write(data)

    def _read_byte(self) -> bytes:
        if self.serial:
            return self.serial.read(1)
        return b''

    def _read_bytes(self, count: int) -> bytes:
        if self.serial:
            return self.serial.read(count)
        return b''

    def _available(self) -> bool:
        return self.serial and self.serial.in_waiting > 0

    def send_audio(self, audio: np.ndarray, chunk_size: int = 1024):
        """Send audio data with serial pacing."""
        if audio.dtype != np.int16:
            audio = (audio * 32767).astype(np.int16)

        for i in range(0, len(audio), chunk_size):
            chunk = audio[i:i + chunk_size]
            data = chunk.tobytes()
            data_len = len(data)

            packet = bytes([
                self.HEADER_1,
                self.HEADER_2,
                (data_len >> 8) & 0xFF,
                data_len & 0xFF
            ]) + data

            self._write_bytes(packet)
            time.sleep(0.01)  # Serial needs pacing


class TCPHandler(ConnectionHandler):
    """Handles TCP communication with ESP32."""

    def __init__(self, host: str = None, port: int = None):
        super().__init__()
        self.host = host or getattr(sys.modules[__name__], 'ESP32_HOST', 'hinze.local')
        self.port = port or getattr(sys.modules[__name__], 'ESP32_PORT', 8266)
        self.sock = None
        self._lock = threading.Lock()
        # Manual read buffer: recv large chunks, serve bytes from buffer
        self._recv_buf = b''

    def connect(self) -> bool:
        try:
            print(f"[TCP] Connecting to {self.host}:{self.port}...")
            self.sock = socket.create_connection((self.host, self.port), timeout=5.0)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            # Fully blocking for writes (TCP flow control back-pressure)
            # Use select() in _fill_buffer() for read timeouts
            self.sock.settimeout(None)
            self._recv_buf = b''
            print(f"[TCP] Connected to {self.host}:{self.port}")
            return True
        except (socket.error, OSError) as e:
            print(f"[TCP] Error connecting: {e}")
            return False

    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=1.0)
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
            self.sock = None
        self._recv_buf = b''
        print("[TCP] Disconnected")

    def _fill_buffer(self):
        """Read a chunk from socket using select() for timeout."""
        try:
            ready, _, _ = select.select([self.sock], [], [], 0.1)
            if ready:
                data = self.sock.recv(4096)
                if data:
                    self._recv_buf += data
        except (socket.error, OSError):
            pass

    def _write_bytes(self, data: bytes):
        with self._lock:
            if self.sock:
                try:
                    self.sock.sendall(data)
                except socket.timeout:
                    # Should not happen with settimeout(None), but retry once
                    print(f"[TCP] Write timeout, retrying...")
                    try:
                        self.sock.sendall(data)
                    except (socket.error, OSError) as e:
                        print(f"[TCP] Write retry failed: {e}")
                except (socket.error, OSError) as e:
                    print(f"[TCP] Write error: {e}")

    def _read_byte(self) -> bytes:
        if not self.sock:
            return b''
        if not self._recv_buf:
            self._fill_buffer()
        if self._recv_buf:
            byte = self._recv_buf[:1]
            self._recv_buf = self._recv_buf[1:]
            return byte
        return b''

    def _read_bytes(self, count: int) -> bytes:
        if not self.sock:
            return b''
        # Wait until we have enough data (with timeout retries)
        attempts = 0
        while len(self._recv_buf) < count and attempts < 50:
            self._fill_buffer()
            if len(self._recv_buf) >= count:
                break
            attempts += 1
        # Return what we have (up to count)
        data = self._recv_buf[:count]
        self._recv_buf = self._recv_buf[count:]
        return data

    def _available(self) -> bool:
        return self.sock is not None

    def send_audio(self, audio: np.ndarray, chunk_size: int = 1024):
        """Send audio data over TCP. No pacing needed - ESP32 ring buffer
        blocking write provides natural TCP back-pressure."""
        if audio.dtype != np.int16:
            audio = (audio * 32767).astype(np.int16)

        for i in range(0, len(audio), chunk_size):
            chunk = audio[i:i + chunk_size]
            data = chunk.tobytes()
            data_len = len(data)

            packet = bytes([
                self.HEADER_1,
                self.HEADER_2,
                (data_len >> 8) & 0xFF,
                data_len & 0xFF
            ]) + data

            self._write_bytes(packet)


class SpeechRecognizer:
    """Transcribes audio using Whisper."""

    def __init__(self, model_name: str = WHISPER_MODEL):
        self.model = None
        self.model_name = model_name

    def load(self):
        if not WHISPER_AVAILABLE:
            print("[WHISPER] Not available")
            return False
        print(f"[WHISPER] Loading model '{self.model_name}'...")
        self.model = whisper.load_model(self.model_name)
        print("[WHISPER] Model loaded")
        return True

    def transcribe(self, audio: np.ndarray) -> str:
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
        elif backend == "deepseek":
            return self._connect_deepseek()
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

    def _connect_deepseek(self) -> bool:
        if not OPENAI_AVAILABLE:
            print("[DEEPSEEK] Not available - install: pip install openai")
            return False
        try:
            self.client = OpenAI(
                base_url="https://api.deepseek.com",
                api_key=DEEPSEEK_API_KEY
            )
            print(f"[DEEPSEEK] Connected")
            return True
        except Exception as e:
            print(f"[DEEPSEEK] Error: {e}")
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
            elif backend == "deepseek":
                response = self._chat_deepseek()
            else:
                return "I don't know how to think.", "confused"

            self.conversation_history.append({
                "role": "assistant",
                "content": response
            })

            if len(self.conversation_history) > 6:
                self.conversation_history = self.conversation_history[-6:]

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
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(self.conversation_history)

        response = ollama.chat(
            model=OLLAMA_MODEL,
            messages=messages
        )
        return response['message']['content']

    def _chat_openrouter(self) -> str:
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(self.conversation_history)

        response = self.client.chat.completions.create(
            model=OPENROUTER_MODEL,
            max_tokens=150,
            messages=messages
        )
        return response.choices[0].message.content

    def _chat_deepseek(self) -> str:
        messages = [{"role": "system", "content": SYSTEM_PROMPT}]
        messages.extend(self.conversation_history)

        response = self.client.chat.completions.create(
            model=DEEPSEEK_MODEL,
            max_tokens=150,
            messages=messages
        )
        return response.choices[0].message.content

    def _extract_emotion(self, text: str) -> tuple[str, str]:
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
        if not PIPER_AVAILABLE:
            print("[PIPER] Not available")
            return False
        try:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            voice_path = os.path.join(script_dir, f"{self.voice_name}.onnx")
            self.voice = PiperVoice.load(voice_path)
            print(f"[PIPER] Loaded voice '{self.voice_name}'")
            return True
        except Exception as e:
            print(f"[PIPER] Error loading voice: {e}")
            return False

    def synthesize(self, text: str) -> np.ndarray:
        """Generate all audio from text at once (legacy)."""
        if not self.voice:
            return np.array([], dtype=np.int16)

        audio_data = []
        for chunk in self.voice.synthesize(text):
            audio_data.append(chunk.audio_int16_array)

        if not audio_data:
            return np.array([], dtype=np.int16)

        audio = np.concatenate(audio_data)

        from scipy import signal
        num_samples = int(len(audio) * 8000 / 22050)
        audio = signal.resample(audio, num_samples).astype(np.int16)

        return audio

    def synthesize_streaming(self, text: str):
        """Generate audio chunks as they become available (generator).

        Yields resampled int16 numpy arrays at 8kHz.
        """
        if not self.voice:
            return

        from scipy import signal

        for chunk in self.voice.synthesize(text):
            raw = chunk.audio_int16_array
            if len(raw) == 0:
                continue
            # Resample from 22050 to 8000 Hz
            num_samples = int(len(raw) * 8000 / 22050)
            if num_samples == 0:
                continue
            resampled = signal.resample(raw, num_samples).astype(np.int16)
            yield resampled


class HinzeHost:
    """Main application class."""

    def __init__(self, connection_mode: str, serial_port: str, tcp_host: str = None, tcp_port: int = None):
        self.connection_mode = connection_mode
        self.serial_port = serial_port
        self.tcp_host = tcp_host
        self.tcp_port = tcp_port
        self.conn = None  # Will be set during setup
        self.recognizer = SpeechRecognizer()
        self.conversation = ConversationEngine()
        self.tts = TextToSpeech()
        self.running = False

    def _create_connection(self) -> ConnectionHandler | None:
        """Create connection based on mode."""
        if self.connection_mode == "tcp":
            handler = TCPHandler(self.tcp_host, self.tcp_port)
            if handler.connect():
                return handler
            print("[HOST] TCP connection failed")
            return None

        elif self.connection_mode == "serial":
            handler = SerialHandler(self.serial_port)
            if handler.connect():
                return handler
            print("[HOST] Serial connection failed")
            return None

        else:  # auto mode
            # Try TCP first
            print("[HOST] Auto mode: trying TCP first...")
            handler = TCPHandler(self.tcp_host, self.tcp_port)
            if handler.connect():
                print("[HOST] Using TCP connection")
                return handler

            # Fall back to serial
            print("[HOST] TCP unavailable, trying serial...")
            handler = SerialHandler(self.serial_port)
            if handler.connect():
                print("[HOST] Using serial connection")
                return handler

            print("[HOST] No connection available")
            return None

    def setup(self) -> bool:
        """Initialize all components."""
        print("\n=== Hinze Host Application v0.7 ===\n")

        # Create connection
        self.conn = self._create_connection()
        if not self.conn:
            return False

        # Start reading
        self.conn.start_reading()

        # Wait for ready event
        print("[HOST] Waiting for ESP32 ready...")
        event = self.conn.wait_for_event("ready", timeout=5.0)
        if not event:
            print("[HOST] Warning: No ready event received")

        # Load speech recognition
        self.recognizer.load()

        # Connect to LLM
        self.conversation.connect()

        # Load TTS
        self.tts.load()

        print("\n[HOST] Setup complete!")
        print("[HOST] Press the button on Hinze to start talking.\n")

        return True

    def run(self):
        """Main loop."""
        self.running = True
        self.conn.set_emotion("idle")

        try:
            while self.running:
                event = self.conn.wait_for_event("button", timeout=1.0)

                if event and event.get("action") == "press":
                    self._handle_interaction()

        except KeyboardInterrupt:
            print("\n[HOST] Shutting down...")

        self.conn.disconnect()

    def _handle_interaction(self):
        """Handle a complete interaction cycle with streaming TTS."""
        print("\n[HOST] === New Interaction ===")

        # 1. Recording phase
        print("[HOST] Recording... (press button to stop)")
        self.conn.set_emotion("listening")

        event = self.conn.wait_for_event("audio_end", timeout=RECORDING_TIMEOUT)
        if not event:
            print("[HOST] Recording timeout")
            self.conn.stop_recording()

        duration = self.conn.audio_buffer.duration()
        print(f"[HOST] Captured {duration:.1f}s of audio")

        if duration < 0.5:
            print("[HOST] Audio too short, ignoring")
            self.conn.set_emotion("idle")
            return

        # 2. Transcription phase
        print("[HOST] Transcribing...")
        self.conn.set_emotion("thinking")

        audio = self.conn.audio_buffer.get_audio()
        self.conn.audio_buffer.clear()
        transcript = self.recognizer.transcribe(audio)

        if not transcript:
            print("[HOST] No speech detected")
            self.conn.set_emotion("confused")
            time.sleep(1)
            self.conn.set_emotion("idle")
            return

        print(f"[HOST] Transcript: \"{transcript}\"")

        # 3. Get LLM response
        print("[HOST] Thinking...")
        response, emotion = self.conversation.chat(transcript)
        print(f"[HOST] Response: \"{response}\" [{emotion}]")

        # 4. Set emotion and stream TTS
        self.conn.set_emotion(emotion)

        if PIPER_AVAILABLE and self.tts.voice:
            print("[HOST] Streaming TTS...")
            self.conn.start_playback()
            time.sleep(0.1)  # Wait for ESP32 to enter playback mode

            total_samples = 0
            for chunk in self.tts.synthesize_streaming(response):
                self.conn.send_audio(chunk)
                total_samples += len(chunk)

            # Let last TCP packets arrive before signaling end
            time.sleep(0.3)
            # Signal end of stream - ESP32 will play until ring buffer drains
            self.conn.stop_playback()
            print(f"[HOST] Streamed {total_samples/8000:.1f}s of audio")

        # 5. Return to idle
        time.sleep(1)
        self.conn.set_emotion("idle")
        print("[HOST] === Interaction Complete ===\n")


def main():
    # Get defaults from config
    default_mode = getattr(sys.modules[__name__], 'CONNECTION_MODE', 'auto')
    default_host = getattr(sys.modules[__name__], 'ESP32_HOST', 'hinze.local')
    default_port = getattr(sys.modules[__name__], 'ESP32_PORT', 8266)

    parser = argparse.ArgumentParser(description="Hinze Robot Companion Host")
    parser.add_argument("--port", "-p", default=SERIAL_PORT,
                        help=f"Serial port (default: {SERIAL_PORT})")
    parser.add_argument("--tcp", action="store_true",
                        help="Force TCP connection")
    parser.add_argument("--serial", action="store_true",
                        help="Force serial connection")
    parser.add_argument("--host", default=default_host,
                        help=f"ESP32 hostname or IP (default: {default_host})")
    parser.add_argument("--tcp-port", type=int, default=default_port,
                        help=f"ESP32 TCP port (default: {default_port})")
    args = parser.parse_args()

    # Determine connection mode
    if args.tcp:
        mode = "tcp"
    elif args.serial:
        mode = "serial"
    else:
        mode = default_mode

    app = HinzeHost(
        connection_mode=mode,
        serial_port=args.port,
        tcp_host=args.host,
        tcp_port=args.tcp_port
    )

    if not app.setup():
        print("[HOST] Setup failed!")
        sys.exit(1)

    app.run()



if __name__ == "__main__":
    main()
