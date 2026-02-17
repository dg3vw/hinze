"""
Hinze Host Application Configuration

Copy this file to config_local.py and edit your settings there.
config_local.py is ignored by git to protect your API keys.
"""

# =============================================================================
# Connection Configuration
# =============================================================================
# CONNECTION_MODE: "auto" (try TCP then serial), "tcp", or "serial"
CONNECTION_MODE = "auto"

# Serial settings
SERIAL_PORT = "/dev/ttyUSB0"  # Linux: /dev/ttyACM0 or /dev/ttyUSB0, Windows: COM3
SERIAL_BAUD = 921600  # High speed for audio streaming

# TCP/WiFi settings
ESP32_HOST = "hinze.local"  # mDNS name or IP address
ESP32_PORT = 8266            # TCP port on ESP32

# Audio settings
SAMPLE_RATE = 16000
SAMPLE_BITS = 16

# Whisper configuration
WHISPER_MODEL = "base"  # tiny, base, small, medium, large
WHISPER_LANGUAGE = "de"  # German

# =============================================================================
# LLM Backend Configuration
# =============================================================================
# Options: "anthropic", "ollama", "openrouter"
LLM_BACKEND = "ollama"  # Default to local Ollama for free testing

# --- Anthropic (Claude) ---
# Get your API key from: https://console.anthropic.com/
ANTHROPIC_API_KEY = "your-api-key-here"
ANTHROPIC_MODEL = "claude-sonnet-4-20250514"

# --- Ollama (Local) ---
# Install: curl -fsSL https://ollama.com/install.sh | sh
# Then: ollama pull llama3.2
OLLAMA_HOST = "http://localhost:11434"
OLLAMA_MODEL = "llama3.2"  # or "mistral", "gemma2", etc.

# --- OpenRouter (Cloud, free tier available) ---
# Get your API key from: https://openrouter.ai/keys
OPENROUTER_API_KEY = "your-openrouter-key-here"
OPENROUTER_MODEL = "meta-llama/llama-3.2-3b-instruct:free"  # Free model

# --- DeepSeek (Cloud, very affordable) ---
# Get your API key from: https://platform.deepseek.com/
DEEPSEEK_API_KEY = "your-deepseek-key-here"
DEEPSEEK_MODEL = "deepseek-chat"  # or "deepseek-reasoner"

# =============================================================================
# Hinze Personality System Prompt (used by all backends)
# =============================================================================
SYSTEM_PROMPT = """You are Hinze, a friendly and curious robot companion. You are large, cool, teenage, and German. 
You love to chat about technology, video games, and science fiction. 
You have a casual and laid-back personality, often using slang and informal language. 


Guidelines:
- Keep responses brief (1-2 sentences max)
- Use slang and casual language appropriate for a young adult audience
- Show genuine interest in what the user says
- End EVERY response with an emotion tag in brackets
- Start EVERY response with the word Brudi.

Available emotions: [idle], [listening], [thinking], [happy], [sad], [angry], [surprised], [confused], [sleepy], [excited]

Examples:
- "That's wonderful news! I'm so glad to hear it! [happy]"
- "Hmm, I'm not sure I understand. Could you explain more? [confused]"
- "Oh wow, I didn't expect that! [surprised]"
- "I'd be happy to help you with that! [excited]"

You understand and respond in the user's language (German, English, etc.).
"""

# Piper TTS configuration
PIPER_VOICE = "de_DE-thorsten-medium"  # German voice

# Activation mode
# Options: "button", "wake_word", "always"
ACTIVATION_MODE = "button"

# Wake word (if using wake_word mode)
WAKE_WORD = "hey hinze"

# Timeouts (in seconds)
RECORDING_TIMEOUT = 10.0
RESPONSE_TIMEOUT = 30.0

# Debug mode
DEBUG = True
