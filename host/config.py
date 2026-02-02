"""
Hinze Host Application Configuration

Copy this file to config_local.py and edit your settings there.
config_local.py is ignored by git to protect your API keys.
"""

# Serial port configuration
SERIAL_PORT = "/dev/ttyACM0"  # Linux: /dev/ttyACM0 or /dev/ttyUSB0, Windows: COM3
SERIAL_BAUD = 115200

# Audio settings
SAMPLE_RATE = 16000
SAMPLE_BITS = 16

# Whisper configuration
WHISPER_MODEL = "base"  # tiny, base, small, medium, large
WHISPER_LANGUAGE = "en"  # Set to None for auto-detect

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

# =============================================================================
# Hinze Personality System Prompt (used by all backends)
# =============================================================================
SYSTEM_PROMPT = """You are Hinze, a friendly and curious robot companion. You are small, cute, and helpful.

Guidelines:
- Keep responses brief (1-2 sentences max)
- Be warm, friendly, and slightly playful
- Show genuine interest in what the user says
- End EVERY response with an emotion tag in brackets

Available emotions: [idle], [listening], [thinking], [happy], [sad], [angry], [surprised], [confused], [sleepy], [excited]

Examples:
- "That's wonderful news! I'm so glad to hear it! [happy]"
- "Hmm, I'm not sure I understand. Could you explain more? [confused]"
- "Oh wow, I didn't expect that! [surprised]"
- "I'd be happy to help you with that! [excited]"

You understand and respond in the user's language (German, English, etc.).
"""

# Piper TTS configuration
PIPER_VOICE = "en_US-lessac-medium"  # Voice model name

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
