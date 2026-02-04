#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <FastLED.h>
#include <driver/i2s.h>
#include <ArduinoJson.h>
#include <math.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <Preferences.h>

// =============================================================================
// Pin Definitions - ESP32-S3 SuperMini
// =============================================================================

// I2S Audio
#define I2S_WS          5
#define I2S_BCK         6
#define I2S_DIN         4
#define I2S_DOUT        7

// I2C Display
#define I2C_SDA         8
#define I2C_SCL         9

// Servos (for future use)
#define SERVO_PAN       1
#define SERVO_TILT      2

// LED & Touch Sensor
#define WS2812_DATA     48
#define TOUCH_PIN       10  // TTP223 touch sensor (active HIGH)

// =============================================================================
// I2S Audio Configuration
// =============================================================================

#define I2S_PORT_MIC    I2S_NUM_0
#define I2S_PORT_SPK    I2S_NUM_0  // Use same port as mic (switch RX/TX)
#define SAMPLE_RATE     16000
#define SAMPLE_RATE_TTS 8000   // Low rate for speech
#define SAMPLE_BITS     16
#define I2S_BUFFER_SIZE 1024
#define AUDIO_BUFFER_SIZE 4096

// Audio packet header bytes for protocol
#define AUDIO_HEADER_1  0xAA
#define AUDIO_HEADER_2  0x55

// =============================================================================
// Ring Buffer for Streaming Playback
// =============================================================================

#define RING_BUFFER_SIZE    8000    // 1 second at 8kHz (16 KB)
#define PREFILL_SAMPLES     4000    // 0.5 sec prefill before playback starts
#define REBUFFER_SAMPLES    2000    // 0.25 sec rebuffer after underrun
#define PLAYBACK_CHUNK_SIZE 256     // Samples per loop iteration (~32ms at 8kHz)

static int16_t ringBuffer[RING_BUFFER_SIZE];
static volatile size_t ringWritePos = 0;
static volatile size_t ringReadPos = 0;
static volatile bool ringStreamEnded = false;  // Host sent play_stop

// Ring buffer helper functions
static inline size_t ringAvailable() {
    size_t w = ringWritePos;
    size_t r = ringReadPos;
    if (w >= r) return w - r;
    return RING_BUFFER_SIZE - r + w;
}

static inline size_t ringFree() {
    return RING_BUFFER_SIZE - 1 - ringAvailable();
}

static void ringWrite(const int16_t* data, size_t samples) {
    size_t free = ringFree();
    if (samples > free) {
        samples = free;  // Drop overflow
    }
    for (size_t i = 0; i < samples; i++) {
        ringBuffer[ringWritePos] = data[i];
        ringWritePos = (ringWritePos + 1) % RING_BUFFER_SIZE;
    }
}

static size_t ringRead(int16_t* data, size_t maxSamples) {
    size_t avail = ringAvailable();
    if (maxSamples > avail) maxSamples = avail;
    for (size_t i = 0; i < maxSamples; i++) {
        data[i] = ringBuffer[ringReadPos];
        ringReadPos = (ringReadPos + 1) % RING_BUFFER_SIZE;
    }
    return maxSamples;
}

static void ringReset() {
    ringWritePos = 0;
    ringReadPos = 0;
    ringStreamEnded = false;
}

// =============================================================================
// Audio State Machine
// =============================================================================

enum AudioState {
    AUDIO_IDLE = 0,
    AUDIO_LISTENING,
    AUDIO_STREAMING,
    AUDIO_BUFFERING,     // Receiving audio, prefilling ring buffer
    AUDIO_PLAYING,       // Streaming playback from ring buffer
    AUDIO_REBUFFERING    // Underrun recovery, waiting for rebuffer fill
};

AudioState audioState = AUDIO_IDLE;
int32_t rawAudioBuffer[AUDIO_BUFFER_SIZE];  // 32-bit buffer for INMP441
int16_t audioBuffer[AUDIO_BUFFER_SIZE];     // 16-bit buffer for I/O
unsigned long recordingStartTime = 0;
#define MAX_RECORDING_MS 10000  // 10 second max recording

// Silence detection
#define SILENCE_THRESHOLD   1000    // Audio level threshold
#define SILENCE_DURATION_MS 1500    // Stop after this much silence (ms)
#define MIN_SPEECH_MS       300     // Minimum speech before silence detection kicks in

// Software gain for microphone (after 32-bit to 16-bit conversion)
#define MIC_GAIN            4       // Small additional gain if needed

unsigned long lastSoundTime = 0;
bool speechDetected = false;

// =============================================================================
// WiFi + TCP Configuration
// =============================================================================

#define WIFI_TCP_PORT       8266
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define WIFI_NET_TASK_STACK 8192

Preferences preferences;
WiFiServer tcpServer(WIFI_TCP_PORT);
WiFiClient tcpClient;
bool wifiConnected = false;
bool tcpClientConnected = false;

// Transport: which channel is active for the current session
enum Transport {
    TRANSPORT_SERIAL = 0,
    TRANSPORT_TCP
};
Transport activeTransport = TRANSPORT_SERIAL;

// TCP receive buffer (for JSON line assembly on network task)
#define TCP_BUFFER_SIZE 512
static char tcpBuffer[TCP_BUFFER_SIZE];
static int tcpBufferIndex = 0;

// =============================================================================
// Serial Command Buffer
// =============================================================================

#define SERIAL_BUFFER_SIZE 256
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// =============================================================================
// Display Configuration
// =============================================================================

#define SCREEN_WIDTH    128
#define SCREEN_HEIGHT   64
#define OLED_RESET      -1
#define OLED_ADDRESS    0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =============================================================================
// LED Configuration
// =============================================================================

#define NUM_LEDS        1
#define LED_BRIGHTNESS  128

CRGB leds[NUM_LEDS];
uint8_t rainbowHue = 0;

// =============================================================================
// Button Configuration
// =============================================================================

volatile bool buttonPressed = false;
volatile unsigned long lastButtonPress = 0;
#define DEBOUNCE_MS     200  // Increased to handle release bounce

void IRAM_ATTR buttonISR() {
    unsigned long now = millis();
    if (now - lastButtonPress > DEBOUNCE_MS) {
        buttonPressed = true;
        lastButtonPress = now;
    }
}

// =============================================================================
// Emotion States
// =============================================================================

enum Emotion {
    EMOTION_IDLE = 0,
    EMOTION_LISTENING,
    EMOTION_THINKING,
    EMOTION_HAPPY,
    EMOTION_SAD,
    EMOTION_ANGRY,
    EMOTION_SURPRISED,
    EMOTION_CONFUSED,
    EMOTION_SLEEPY,
    EMOTION_EXCITED
};

Emotion currentEmotion = EMOTION_IDLE;

// =============================================================================
// Eye Configuration
// =============================================================================

#define EYE_WIDTH       40
#define EYE_HEIGHT      40
#define EYE_SPACING     48
#define PUPIL_SIZE      10
#define PUPIL_MAX_OFFSET 8

// Eye state
float pupilX = 0.0;
float pupilY = 0.0;
float targetPupilX = 0.0;
float targetPupilY = 0.0;

// Blink state
bool isBlinking = false;
int blinkFrame = 0;
#define BLINK_FRAMES    4

// Animation state
unsigned long lastLookChange = 0;
unsigned long lastBlink = 0;
unsigned long nextLookInterval = 2000;
unsigned long nextBlinkInterval = 3000;
unsigned long animationFrame = 0;

// =============================================================================
// Function Prototypes
// =============================================================================

void setupDisplay();
void setupLED();
void setupButton();
void setupI2SMic();
void setupI2SSpk();
void setupWiFi();
void updateEyes();
void drawEyes();
void updateLED();
void handleButton();
void updateIdleBehavior();
void handleSerialCommands();
void handleSerialInput();
void processCommand(const char* json, Transport source);
void setEmotionByName(const char* name);
void sendStatus(Transport source);
void sendResponse(const char* json, Transport source);
void sendEvent(const char* event, const char* action = nullptr);
void updateAudioCapture();
void startRecording();
void stopRecording();
void streamAudioPacket(int16_t* data, size_t samples);
void handleAudioInput(Stream& stream);
void updateStreamingPlayback();
void startPlaybackMode(Transport source);
void stopPlaybackMode();
void wifiNetTask(void* param);

// Eye drawing functions for each emotion
void drawIdleEyes();
void drawListeningEyes();
void drawThinkingEyes();
void drawHappyEyes();
void drawSadEyes();
void drawAngryEyes();
void drawSurprisedEyes();
void drawConfusedEyes();
void drawSleepyEyes();
void drawExcitedEyes();

// Helper functions
void drawRoundEye(int cx, int cy, int w, int h, int pupilOffX, int pupilOffY, int cornerRadius);
void drawPupil(int cx, int cy, int eyeH, int offX, int offY);

// =============================================================================
// Setup
// =============================================================================

void setup() {
    Serial.setRxBufferSize(16384);  // Large buffer for audio streaming
    Serial.begin(921600);  // High speed for audio streaming
    while (!Serial && millis() < 3000);

    Serial.println();
    Serial.println("=================================");
    Serial.println("  Hinze Robot Companion v0.7");
    Serial.println("  ESP32-S3 SuperMini");
    Serial.println("  WiFi Streaming Audio");
    Serial.println("=================================");

    Wire.begin(I2C_SDA, I2C_SCL);

    setupDisplay();
    setupLED();
    setupButton();
    setupI2SMic();
    setupWiFi();

    Serial.println("[INIT] Setup complete!");
    sendEvent("ready");
}

// =============================================================================
// Main Loop (Core 1) - Eyes/LED/Buttons/Playback
// =============================================================================

void loop() {
    handleSerialInput();
    handleButton();
    updateAudioCapture();
    updateStreamingPlayback();

    if (currentEmotion == EMOTION_IDLE && audioState == AUDIO_IDLE) {
        updateIdleBehavior();
    }

    updateEyes();
    drawEyes();
    updateLED();

    animationFrame++;
    delay(33);  // ~30 FPS
}

// =============================================================================
// Display Setup
// =============================================================================

void setupDisplay() {
    Serial.println("[INIT] Setting up OLED...");

    if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS)) {
        Serial.println("[OLED] ERROR: Display not found!");
        return;
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(30, 28);
    display.print("Hinze v0.7");
    display.display();
    delay(1000);

    Serial.println("[OLED] Display initialized");
}

// =============================================================================
// WiFi Setup
// =============================================================================

void setupWiFi() {
    preferences.begin("hinze", true);  // read-only
    String ssid = preferences.getString("wifi_ssid", "");
    String pass = preferences.getString("wifi_pass", "");
    preferences.end();

    if (ssid.length() == 0) {
        Serial.println("[WIFI] No credentials stored. Use wifi_config command to set.");
        Serial.println("[WIFI] Running in serial-only mode.");
        return;
    }

    Serial.printf("[WIFI] Connecting to '%s'...\n", ssid.c_str());
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid.c_str(), pass.c_str());

    unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startMs < WIFI_CONNECT_TIMEOUT_MS) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        wifiConnected = true;
        Serial.printf("[WIFI] Connected! IP: %s\n", WiFi.localIP().toString().c_str());

        // Start mDNS
        if (MDNS.begin("hinze")) {
            MDNS.addService("hinze", "tcp", WIFI_TCP_PORT);
            Serial.printf("[WIFI] mDNS: hinze.local:%d\n", WIFI_TCP_PORT);
        }

        // Start TCP server
        tcpServer.begin();
        tcpServer.setNoDelay(true);
        Serial.printf("[WIFI] TCP server listening on port %d\n", WIFI_TCP_PORT);

        // Launch network task on core 0
        xTaskCreatePinnedToCore(
            wifiNetTask,
            "wifiNet",
            WIFI_NET_TASK_STACK,
            NULL,
            1,          // Priority
            NULL,
            0           // Core 0
        );
        Serial.println("[WIFI] Network task started on core 0");
    } else {
        Serial.println("[WIFI] Connection failed. Running in serial-only mode.");
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
    }
}

// =============================================================================
// WiFi Network Task (Core 0) - TCP server + audio receive
// =============================================================================

void wifiNetTask(void* param) {
    Serial.println("[NET] WiFi network task running on core 0");

    while (true) {
        // Accept new client if none connected
        if (!tcpClientConnected) {
            WiFiClient newClient = tcpServer.available();
            if (newClient) {
                tcpClient = newClient;
                tcpClient.setNoDelay(true);
                tcpClientConnected = true;
                tcpBufferIndex = 0;
                Serial.printf("[NET] Client connected from %s\n",
                             tcpClient.remoteIP().toString().c_str());
            }
        }

        // Handle connected client
        if (tcpClientConnected) {
            if (!tcpClient.connected()) {
                Serial.println("[NET] Client disconnected");
                tcpClientConnected = false;
                tcpClient.stop();
                // If we were playing via TCP, stop
                if (activeTransport == TRANSPORT_TCP &&
                    (audioState == AUDIO_BUFFERING || audioState == AUDIO_PLAYING || audioState == AUDIO_REBUFFERING)) {
                    ringStreamEnded = true;
                }
                activeTransport = TRANSPORT_SERIAL;
                vTaskDelay(10 / portTICK_PERIOD_MS);
                continue;
            }

            // Read available data from TCP
            while (tcpClient.available()) {
                uint8_t b = tcpClient.peek();

                // Check for audio packet header
                if (b == AUDIO_HEADER_1 &&
                    (audioState == AUDIO_BUFFERING || audioState == AUDIO_PLAYING || audioState == AUDIO_REBUFFERING)) {
                    handleAudioInput(tcpClient);
                } else {
                    // JSON line assembly
                    char c = tcpClient.read();
                    if (c == '\n' || c == '\r') {
                        if (tcpBufferIndex > 0) {
                            tcpBuffer[tcpBufferIndex] = '\0';
                            processCommand(tcpBuffer, TRANSPORT_TCP);
                            tcpBufferIndex = 0;
                        }
                    } else if (tcpBufferIndex < TCP_BUFFER_SIZE - 1) {
                        tcpBuffer[tcpBufferIndex++] = c;
                    }
                }
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);  // Yield to other tasks
    }
}

// =============================================================================
// Transport Abstraction
// =============================================================================

void sendResponse(const char* json, Transport source) {
    if (source == TRANSPORT_TCP && tcpClientConnected) {
        tcpClient.println(json);
    } else {
        Serial.println(json);
    }
}

void sendEvent(const char* event, const char* action) {
    char buf[128];
    if (action) {
        snprintf(buf, sizeof(buf), "{\"event\":\"%s\",\"action\":\"%s\"}", event, action);
    } else {
        snprintf(buf, sizeof(buf), "{\"event\":\"%s\"}", event);
    }
    // Send events on both transports so host always sees them
    Serial.println(buf);
    if (tcpClientConnected) {
        tcpClient.println(buf);
    }
}

// =============================================================================
// Idle Behavior (look around + blink)
// =============================================================================

void updateIdleBehavior() {
    unsigned long now = millis();

    if (now - lastLookChange > nextLookInterval) {
        targetPupilX = random(-100, 101) / 100.0;
        targetPupilY = random(-100, 101) / 100.0;
        lastLookChange = now;
        nextLookInterval = random(1500, 4000);
    }

    if (!isBlinking && now - lastBlink > nextBlinkInterval) {
        isBlinking = true;
        blinkFrame = 0;
        lastBlink = now;
        nextBlinkInterval = random(2000, 6000);
    }
}

void updateEyes() {
    // Smooth pupil movement
    float speed = 0.15;
    pupilX += (targetPupilX - pupilX) * speed;
    pupilY += (targetPupilY - pupilY) * speed;

    // Blink animation
    if (isBlinking) {
        blinkFrame++;
        if (blinkFrame >= BLINK_FRAMES * 2) {
            isBlinking = false;
            blinkFrame = 0;
        }
    }
}

// =============================================================================
// Main Eye Drawing - Routes to emotion-specific functions
// =============================================================================

void drawEyes() {
    display.clearDisplay();

    switch (currentEmotion) {
        case EMOTION_IDLE:       drawIdleEyes(); break;
        case EMOTION_LISTENING:  drawListeningEyes(); break;
        case EMOTION_THINKING:   drawThinkingEyes(); break;
        case EMOTION_HAPPY:      drawHappyEyes(); break;
        case EMOTION_SAD:        drawSadEyes(); break;
        case EMOTION_ANGRY:      drawAngryEyes(); break;
        case EMOTION_SURPRISED:  drawSurprisedEyes(); break;
        case EMOTION_CONFUSED:   drawConfusedEyes(); break;
        case EMOTION_SLEEPY:     drawSleepyEyes(); break;
        case EMOTION_EXCITED:    drawExcitedEyes(); break;
    }

    display.display();
}

// =============================================================================
// Helper Functions
// =============================================================================

void drawRoundEye(int cx, int cy, int w, int h, int pupilOffX, int pupilOffY, int cornerRadius) {
    int x = cx - w / 2;
    int y = cy - h / 2;
    display.fillRoundRect(x, y, w, h, cornerRadius, SSD1306_WHITE);

    // Draw pupil
    if (h > 8) {
        int maxOff = (h / 2) - (PUPIL_SIZE / 2) - 2;
        if (maxOff < 0) maxOff = 0;
        int py = constrain(cy + pupilOffY, cy - maxOff, cy + maxOff);
        int px = cx + pupilOffX;
        display.fillCircle(px, py, PUPIL_SIZE / 2, SSD1306_BLACK);
    }
}

// =============================================================================
// Emotion: IDLE - Look around with natural blink
// =============================================================================

void drawIdleEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    int blinkState = 0;
    if (isBlinking) {
        blinkState = (blinkFrame < BLINK_FRAMES) ? blinkFrame : (BLINK_FRAMES * 2 - blinkFrame - 1);
    }

    int h = EYE_HEIGHT - (blinkState * EYE_HEIGHT / (BLINK_FRAMES + 1));
    if (h < 4) h = 4;

    int pupilOffX = (int)(pupilX * PUPIL_MAX_OFFSET);
    int pupilOffY = (int)(pupilY * PUPIL_MAX_OFFSET);

    drawRoundEye(leftX, eyeY, EYE_WIDTH, h, pupilOffX, pupilOffY, 8);
    drawRoundEye(rightX, eyeY, EYE_WIDTH, h, pupilOffX, pupilOffY, 8);
}

// =============================================================================
// Emotion: LISTENING - Wide open, alert eyes
// =============================================================================

void drawListeningEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    // Slightly larger, very round eyes
    int w = 44;
    int h = 44;

    // Subtle size pulse
    int pulse = (animationFrame % 30 < 15) ? 2 : 0;
    h += pulse;
    w += pulse;

    drawRoundEye(leftX, eyeY, w, h, 0, -2, 10);
    drawRoundEye(rightX, eyeY, w, h, 0, -2, 10);
}

// =============================================================================
// Emotion: THINKING - Eyes look up-left, slightly squinted
// =============================================================================

void drawThinkingEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    // Squinted eyes
    int w = 38;
    int h = 28;

    // Looking up-left
    int pupilOffX = -6;
    int pupilOffY = -4;

    drawRoundEye(leftX, eyeY, w, h, pupilOffX, pupilOffY, 6);
    drawRoundEye(rightX, eyeY, w, h, pupilOffX, pupilOffY, 6);

    // Draw thinking dots
    int dotY = 50;
    int numDots = ((animationFrame / 15) % 4);
    for (int i = 0; i < numDots && i < 3; i++) {
        display.fillCircle(52 + i * 12, dotY, 3, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: HAPPY - Curved ^ ^ anime-style happy eyes
// =============================================================================

void drawHappyEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 32;

    // Draw happy curved eyes (like ^ ^)
    // Left eye - arc curving up
    for (int i = -18; i <= 18; i++) {
        int y = eyeY - 8 + abs(i) / 2;
        display.fillRect(leftX + i - 2, y, 5, 8, SSD1306_WHITE);
    }

    // Right eye - arc curving up
    for (int i = -18; i <= 18; i++) {
        int y = eyeY - 8 + abs(i) / 2;
        display.fillRect(rightX + i - 2, y, 5, 8, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: SAD - Droopy eyes angled down
// =============================================================================

void drawSadEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Droopy eyes
    int w = 36;
    int h = 30;

    // Draw eyes
    drawRoundEye(leftX, eyeY, w, h, 0, 4, 6);
    drawRoundEye(rightX, eyeY, w, h, 0, 4, 6);

    // Sad eyebrows (angled down toward center)
    // Left eyebrow
    display.drawLine(leftX - 18, eyeY - 22, leftX + 12, eyeY - 18, SSD1306_WHITE);
    display.drawLine(leftX - 18, eyeY - 21, leftX + 12, eyeY - 17, SSD1306_WHITE);

    // Right eyebrow
    display.drawLine(rightX - 12, eyeY - 18, rightX + 18, eyeY - 22, SSD1306_WHITE);
    display.drawLine(rightX - 12, eyeY - 17, rightX + 18, eyeY - 21, SSD1306_WHITE);

    // Tear drop on left eye
    if ((animationFrame / 20) % 2 == 0) {
        int tearY = eyeY + 20 + (animationFrame % 20) / 2;
        display.fillCircle(leftX + 10, tearY, 3, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: ANGRY - Narrow eyes with angled eyebrows
// =============================================================================

void drawAngryEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Narrow, intense eyes
    int w = 42;
    int h = 20;

    drawRoundEye(leftX, eyeY, w, h, 0, 0, 4);
    drawRoundEye(rightX, eyeY, w, h, 0, 0, 4);

    // Angry eyebrows (angled down toward center, thick)
    // Left eyebrow - angled down to right
    for (int t = 0; t < 4; t++) {
        display.drawLine(leftX - 20, eyeY - 16 + t, leftX + 15, eyeY - 24 + t, SSD1306_WHITE);
    }

    // Right eyebrow - angled down to left
    for (int t = 0; t < 4; t++) {
        display.drawLine(rightX - 15, eyeY - 24 + t, rightX + 20, eyeY - 16 + t, SSD1306_WHITE);
    }
}

// =============================================================================
// Emotion: SURPRISED - Very wide, round eyes
// =============================================================================

void drawSurprisedEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Very large round eyes
    int w = 46;
    int h = 46;

    // Small pupils for surprised look
    display.fillRoundRect(leftX - w/2, eyeY - h/2, w, h, 12, SSD1306_WHITE);
    display.fillRoundRect(rightX - w/2, eyeY - h/2, w, h, 12, SSD1306_WHITE);

    // Small pupils
    display.fillCircle(leftX, eyeY - 2, 4, SSD1306_BLACK);
    display.fillCircle(rightX, eyeY - 2, 4, SSD1306_BLACK);

    // Raised eyebrows
    display.fillRoundRect(leftX - 18, eyeY - 32, 36, 4, 2, SSD1306_WHITE);
    display.fillRoundRect(rightX - 18, eyeY - 32, 36, 4, 2, SSD1306_WHITE);
}

// =============================================================================
// Emotion: CONFUSED - Asymmetric eyes, one raised
// =============================================================================

void drawConfusedEyes() {
    int leftX = 40;
    int rightX = 88;

    // Left eye - normal but looking to side
    drawRoundEye(leftX, 34, 38, 36, 6, 0, 8);

    // Right eye - raised and smaller (questioning)
    drawRoundEye(rightX, 28, 34, 32, -4, -2, 8);

    // One raised eyebrow on right
    display.fillRoundRect(rightX - 16, 8, 32, 4, 2, SSD1306_WHITE);

    // Question mark
    display.setTextSize(2);
    display.setCursor(108, 8);
    display.print("?");
}

// =============================================================================
// Emotion: SLEEPY - Half-closed eyes
// =============================================================================

void drawSleepyEyes() {
    int leftX = 40;
    int rightX = 88;
    int eyeY = 34;

    // Half-closed droopy eyes
    int w = 40;
    int h = 14;  // Very squished

    // Slight droop animation
    int droop = (animationFrame / 40) % 2 == 0 ? 0 : 2;
    h -= droop;
    if (h < 8) h = 8;

    drawRoundEye(leftX, eyeY + 2, w, h, 0, 0, 4);
    drawRoundEye(rightX, eyeY + 2, w, h, 0, 0, 4);

    // Heavy eyelids (top of eye darker/covered)
    display.fillRect(leftX - 22, eyeY - 12, 44, 12, SSD1306_BLACK);
    display.fillRect(rightX - 22, eyeY - 12, 44, 12, SSD1306_BLACK);

    // Z's floating
    display.setTextSize(1);
    int zOffset = (animationFrame / 10) % 10;
    display.setCursor(100, 10 - zOffset/2);
    display.print("Z");
    display.setCursor(108, 5 - zOffset/2);
    display.print("z");
    display.setCursor(114, 0);
    display.print("z");
}

// =============================================================================
// Emotion: EXCITED - Sparkly bouncy eyes
// =============================================================================

void drawExcitedEyes() {
    int leftX = 40;
    int rightX = 88;

    // Bouncy vertical offset
    int bounce = sin(animationFrame * 0.3) * 4;
    int eyeY = 32 + bounce;

    // Bright wide eyes
    int w = 42;
    int h = 42;

    drawRoundEye(leftX, eyeY, w, h, 0, -2, 10);
    drawRoundEye(rightX, eyeY, w, h, 0, -2, 10);

    // Sparkle effects around eyes
    int sparklePhase = (animationFrame / 5) % 4;

    // Sparkles at different positions based on phase
    if (sparklePhase == 0) {
        display.drawPixel(leftX - 24, eyeY - 20, SSD1306_WHITE);
        display.drawPixel(rightX + 24, eyeY - 18, SSD1306_WHITE);
    } else if (sparklePhase == 1) {
        display.drawLine(leftX + 20, eyeY - 24, leftX + 24, eyeY - 28, SSD1306_WHITE);
        display.drawLine(rightX - 20, eyeY - 24, rightX - 24, eyeY - 28, SSD1306_WHITE);
    } else if (sparklePhase == 2) {
        display.fillCircle(leftX - 26, eyeY, 2, SSD1306_WHITE);
        display.fillCircle(rightX + 26, eyeY, 2, SSD1306_WHITE);
    } else {
        display.drawLine(leftX - 22, eyeY + 18, leftX - 18, eyeY + 22, SSD1306_WHITE);
        display.drawLine(rightX + 22, eyeY + 18, rightX + 18, eyeY + 22, SSD1306_WHITE);
    }

    // Star sparkle in eyes
    display.drawPixel(leftX + 6, eyeY - 8, SSD1306_WHITE);
    display.drawPixel(rightX + 6, eyeY - 8, SSD1306_WHITE);
}

// =============================================================================
// LED Setup & Update
// =============================================================================

void setupLED() {
    Serial.println("[INIT] Setting up LED...");
    FastLED.addLeds<WS2812, WS2812_DATA, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(LED_BRIGHTNESS);
    leds[0] = CRGB::Black;
    FastLED.show();
    Serial.printf("[LED] WS2812 on GPIO %d\n", WS2812_DATA);
}

void updateLED() {
    switch (currentEmotion) {
        case EMOTION_IDLE:
            leds[0] = CHSV(rainbowHue++, 255, 255);
            break;
        case EMOTION_LISTENING:
            leds[0] = CHSV(160, 255, beatsin8(60, 100, 255));
            break;
        case EMOTION_THINKING:
            leds[0] = CHSV(43, 255, beatsin8(90, 100, 255));
            break;
        case EMOTION_HAPPY:
            leds[0] = CHSV(96, 255, 255);
            break;
        case EMOTION_SAD:
            leds[0] = CHSV(160, 255, 180);
            break;
        case EMOTION_ANGRY:
            leds[0] = CHSV(0, 255, 255);
            break;
        case EMOTION_SURPRISED:
            leds[0] = CRGB::White;
            break;
        case EMOTION_CONFUSED:
            leds[0] = CHSV(192, 255, 255);
            break;
        case EMOTION_SLEEPY:
            leds[0] = CHSV(32, 255, 80);
            break;
        case EMOTION_EXCITED:
            leds[0] = CHSV(rainbowHue, 255, 255);
            rainbowHue += 5;
            break;
    }
    FastLED.show();
}

// =============================================================================
// Button Handling
// =============================================================================

void setupButton() {
    Serial.println("[INIT] Setting up touch sensor...");
    pinMode(TOUCH_PIN, INPUT);  // TTP223 has its own output driver
    attachInterrupt(digitalPinToInterrupt(TOUCH_PIN), buttonISR, RISING);  // Active HIGH
    Serial.printf("[TOUCH] TTP223 on GPIO %d (interrupt enabled)\n", TOUCH_PIN);
}

void handleButton() {
    // Push-to-talk: press to start, auto-stop on silence or timeout

    if (buttonPressed) {
        buttonPressed = false;

        if (audioState == AUDIO_IDLE) {
            sendEvent("button", "press");
            startRecording();

            // Reset animation states
            targetPupilX = 0;
            targetPupilY = 0;
            animationFrame = 0;
        } else if (audioState == AUDIO_STREAMING) {
            // Only allow manual stop after minimum recording time (prevent bounce)
            if (millis() - recordingStartTime > 500) {
                sendEvent("button", "press");
                stopRecording();
            }
        }
    }
}

// =============================================================================
// I2S Microphone Setup
// =============================================================================

void setupI2SMic() {
    Serial.println("[INIT] Setting up I2S microphone...");

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // INMP441 outputs 24-bit in 32-bit frame
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // L/R pin to GND = LEFT channel
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = I2S_BUFFER_SIZE,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN
    };

    esp_err_t err = i2s_driver_install(I2S_PORT_MIC, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: Driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT_MIC, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: Pin config failed: %d\n", err);
        return;
    }

    Serial.printf("[I2S] Microphone configured: %dHz, %d-bit\n", SAMPLE_RATE, SAMPLE_BITS);
    Serial.printf("[I2S] Pins: WS=%d, BCK=%d, DIN=%d\n", I2S_WS, I2S_BCK, I2S_DIN);
}

// =============================================================================
// I2S Speaker Setup
// =============================================================================

void setupI2SSpk() {
    Serial.println("[INIT] Setting up I2S speaker...");

    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE_TTS,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,  // 32-bit frame
        .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 16,
        .dma_buf_len = 1024,
        .use_apll = false,  // Use default PLL
        .tx_desc_auto_clear = true,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_DOUT,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    esp_err_t err = i2s_driver_install(I2S_PORT_SPK, &i2s_config, 0, NULL);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: Speaker driver install failed: %d\n", err);
        return;
    }

    err = i2s_set_pin(I2S_PORT_SPK, &pin_config);
    if (err != ESP_OK) {
        Serial.printf("[I2S] ERROR: Speaker pin config failed: %d\n", err);
        return;
    }

    Serial.printf("[I2S] Speaker configured: %dHz, 32-bit stereo\n", SAMPLE_RATE_TTS);
    Serial.printf("[I2S] Pins: WS=%d, BCK=%d, DOUT=%d\n", I2S_WS, I2S_BCK, I2S_DOUT);
}

// =============================================================================
// Streaming Playback - Non-blocking, ring buffer based
// =============================================================================

static int audioPacketCount = 0;

void startPlaybackMode(Transport source) {
    // Switch from mic to speaker I2S
    i2s_driver_uninstall(I2S_PORT_MIC);
    setupI2SSpk();

    ringReset();
    audioPacketCount = 0;
    activeTransport = source;
    audioState = AUDIO_BUFFERING;

    char resp[] = "{\"ok\":true,\"action\":\"play_ready\"}";
    sendResponse(resp, source);
    Serial.println("[AUDIO] Playback mode: buffering...");
}

void stopPlaybackMode() {
    Serial.printf("[AUDIO] Playback complete. %d packets received.\n", audioPacketCount);
    audioState = AUDIO_IDLE;
    i2s_driver_uninstall(I2S_PORT_SPK);
    setupI2SMic();
}

void updateStreamingPlayback() {
    // Only run in playback-related states
    if (audioState != AUDIO_BUFFERING && audioState != AUDIO_PLAYING && audioState != AUDIO_REBUFFERING) {
        return;
    }

    size_t avail = ringAvailable();

    // BUFFERING: wait for prefill before starting playback
    if (audioState == AUDIO_BUFFERING) {
        if (avail >= PREFILL_SAMPLES || (ringStreamEnded && avail > 0)) {
            Serial.printf("[AUDIO] Prefill complete (%d samples), starting playback\n", avail);
            audioState = AUDIO_PLAYING;
        } else if (ringStreamEnded && avail == 0) {
            // Stream ended before we got any audio
            Serial.println("[AUDIO] Stream ended with no audio");
            stopPlaybackMode();
            return;
        }
        return;  // Don't play yet
    }

    // REBUFFERING: wait for partial refill after underrun
    if (audioState == AUDIO_REBUFFERING) {
        if (avail >= REBUFFER_SAMPLES || (ringStreamEnded && avail > 0)) {
            Serial.printf("[AUDIO] Rebuffer complete (%d samples), resuming\n", avail);
            audioState = AUDIO_PLAYING;
        } else if (ringStreamEnded && avail == 0) {
            Serial.println("[AUDIO] Stream ended during rebuffer");
            stopPlaybackMode();
            return;
        }
        return;
    }

    // PLAYING: feed I2S from ring buffer
    if (avail == 0) {
        if (ringStreamEnded) {
            // All audio played
            Serial.println("[AUDIO] Ring buffer drained, playback done");
            stopPlaybackMode();
            return;
        }
        // Underrun - need to rebuffer
        Serial.println("[AUDIO] Buffer underrun, rebuffering...");
        audioState = AUDIO_REBUFFERING;
        return;
    }

    // Read up to PLAYBACK_CHUNK_SIZE samples from ring buffer
    int16_t playChunk[PLAYBACK_CHUNK_SIZE];
    size_t toPlay = min(avail, (size_t)PLAYBACK_CHUNK_SIZE);
    size_t got = ringRead(playChunk, toPlay);

    // Convert to 32-bit stereo for I2S
    int32_t stereoBuffer[PLAYBACK_CHUNK_SIZE * 2];
    for (size_t i = 0; i < got; i++) {
        int32_t sample = (int32_t)playChunk[i] * 2;
        if (sample > 32767) sample = 32767;
        if (sample < -32768) sample = -32768;
        sample <<= 16;
        stereoBuffer[i * 2] = sample;
        stereoBuffer[i * 2 + 1] = sample;
    }

    size_t bytesWritten;
    i2s_write(I2S_PORT_SPK, stereoBuffer, got * 8, &bytesWritten, 10 / portTICK_PERIOD_MS);
}

// =============================================================================
// Audio Capture Functions
// =============================================================================

void startRecording() {
    Serial.println("[AUDIO] Starting recording...");
    audioState = AUDIO_LISTENING;
    currentEmotion = EMOTION_LISTENING;
    recordingStartTime = millis();
    lastSoundTime = millis();
    speechDetected = false;
    sendEvent("audio_start");

    // Small delay then start streaming
    delay(50);
    audioState = AUDIO_STREAMING;
}

void stopRecording() {
    Serial.println("[AUDIO] Stopping recording...");
    audioState = AUDIO_IDLE;
    currentEmotion = EMOTION_THINKING;
    sendEvent("audio_end");
}

// Calculate average audio level (simple absolute mean)
uint16_t calculateAudioLevel(int16_t* samples, size_t count) {
    if (count == 0) return 0;

    uint32_t sum = 0;
    for (size_t i = 0; i < count; i++) {
        sum += abs(samples[i]);
    }
    return sum / count;
}

void updateAudioCapture() {
    if (audioState != AUDIO_STREAMING) {
        return;
    }

    unsigned long now = millis();
    unsigned long elapsed = now - recordingStartTime;

    // Check for timeout
    if (elapsed > MAX_RECORDING_MS) {
        Serial.println("[AUDIO] Recording timeout");
        stopRecording();
        return;
    }

    // Read 32-bit audio data from I2S (INMP441 outputs 24-bit in 32-bit frame)
    size_t bytesRead = 0;
    esp_err_t err = i2s_read(I2S_PORT_MIC, rawAudioBuffer,
                             AUDIO_BUFFER_SIZE * sizeof(int32_t),
                             &bytesRead, 10 / portTICK_PERIOD_MS);

    if (err == ESP_OK && bytesRead > 0) {
        size_t samplesRead = bytesRead / sizeof(int32_t);

        // Convert 32-bit to 16-bit (INMP441 data is in upper 24 bits, left-justified)
        for (size_t i = 0; i < samplesRead; i++) {
            int32_t sample = rawAudioBuffer[i] >> 14;  // Extract upper bits
            sample = sample * MIC_GAIN;
            if (sample > 32767) sample = 32767;
            if (sample < -32768) sample = -32768;
            audioBuffer[i] = (int16_t)sample;
        }

        // Calculate audio level for silence detection
        uint16_t level = calculateAudioLevel(audioBuffer, samplesRead);

        // Debug: print audio level every ~500ms
        static unsigned long lastLevelPrint = 0;
        if (now - lastLevelPrint > 500) {
            Serial.printf("[MIC] Level: %5d (threshold: %d)\n", level, SILENCE_THRESHOLD);
            lastLevelPrint = now;
        }

        // Check if sound detected
        if (level > SILENCE_THRESHOLD) {
            lastSoundTime = now;
            if (!speechDetected && elapsed > MIN_SPEECH_MS) {
                speechDetected = true;
                Serial.println("[AUDIO] Speech detected");
            }
        }

        // Check for silence (only after minimum speech time)
        if (speechDetected && (now - lastSoundTime > SILENCE_DURATION_MS)) {
            Serial.println("[AUDIO] Silence detected - stopping");
            stopRecording();
            return;
        }

        // Stream the audio
        streamAudioPacket(audioBuffer, samplesRead);
    }
}

void streamAudioPacket(int16_t* data, size_t samples) {
    // Send audio packet with header: [0xAA][0x55][len_high][len_low][pcm_data...]
    size_t dataBytes = samples * sizeof(int16_t);

    // Always send mic audio via serial (mic capture is always serial-initiated for now)
    Serial.write(AUDIO_HEADER_1);
    Serial.write(AUDIO_HEADER_2);
    Serial.write((uint8_t)(dataBytes >> 8));
    Serial.write((uint8_t)(dataBytes & 0xFF));
    Serial.write((uint8_t*)data, dataBytes);

    // Also send via TCP if client connected
    if (tcpClientConnected) {
        uint8_t header[4] = {
            AUDIO_HEADER_1,
            AUDIO_HEADER_2,
            (uint8_t)(dataBytes >> 8),
            (uint8_t)(dataBytes & 0xFF)
        };
        tcpClient.write(header, 4);
        tcpClient.write((uint8_t*)data, dataBytes);
    }
}

// =============================================================================
// Audio Input Handling (from Serial or TCP stream)
// =============================================================================

void handleAudioInput(Stream& stream) {
    // Read and verify header
    uint8_t header1 = stream.read();
    if (header1 != AUDIO_HEADER_1) return;

    // Wait for second header byte with timeout
    unsigned long start = millis();
    while (!stream.available()) {
        if (millis() - start > 100) return;
        delay(1);
    }
    uint8_t header2 = stream.read();
    if (header2 != AUDIO_HEADER_2) return;

    // Wait for length bytes
    while (stream.available() < 2) {
        if (millis() - start > 100) return;
        delay(1);
    }
    uint8_t lenHigh = stream.read();
    uint8_t lenLow = stream.read();
    uint16_t dataLen = (lenHigh << 8) | lenLow;

    // Sanity check
    if (dataLen > AUDIO_BUFFER_SIZE * 2) {
        Serial.printf("[AUDIO] Packet too large: %d bytes\n", dataLen);
        return;
    }

    // Read audio data into temp buffer
    uint16_t bytesRead = 0;
    while (bytesRead < dataLen) {
        if (stream.available()) {
            ((uint8_t*)audioBuffer)[bytesRead++] = stream.read();
        } else if (millis() - start > 1000) {
            Serial.println("[AUDIO] Read timeout");
            return;
        } else {
            delay(1);
        }
    }

    audioPacketCount++;
    if (audioPacketCount % 20 == 1) {
        Serial.printf("[PLAY] Pkt#%d: %d bytes, ring: %d/%d\n",
            audioPacketCount, dataLen, ringAvailable(), RING_BUFFER_SIZE);
    }

    // Write to ring buffer
    size_t samples = dataLen / 2;
    ringWrite(audioBuffer, samples);
}

// =============================================================================
// Serial Command Handling
// =============================================================================

void handleSerialInput() {
    // In playback mode via serial, handle audio packets
    if (activeTransport == TRANSPORT_SERIAL &&
        (audioState == AUDIO_BUFFERING || audioState == AUDIO_PLAYING || audioState == AUDIO_REBUFFERING)) {

        while (Serial.available()) {
            uint8_t b = Serial.peek();
            if (b == AUDIO_HEADER_1) {
                handleAudioInput(Serial);
            } else if (b == '{') {
                // Could be a JSON command (like play_stop) - parse it
                char c = Serial.read();
                serialBuffer[0] = c;
                serialBufferIndex = 1;
                // Read until newline
                unsigned long start = millis();
                while (millis() - start < 100) {
                    if (Serial.available()) {
                        c = Serial.read();
                        if (c == '\n' || c == '\r') {
                            serialBuffer[serialBufferIndex] = '\0';
                            processCommand(serialBuffer, TRANSPORT_SERIAL);
                            serialBufferIndex = 0;
                            break;
                        } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
                            serialBuffer[serialBufferIndex++] = c;
                        }
                    }
                }
                serialBufferIndex = 0;
            } else {
                Serial.read();  // Skip non-audio, non-JSON bytes
            }
        }
        return;
    }

    // Normal JSON command handling
    while (Serial.available()) {
        char c = Serial.read();

        if (c == '\n' || c == '\r') {
            if (serialBufferIndex > 0) {
                serialBuffer[serialBufferIndex] = '\0';
                processCommand(serialBuffer, TRANSPORT_SERIAL);
                serialBufferIndex = 0;
            }
        } else if (serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
            serialBuffer[serialBufferIndex++] = c;
        }
    }
}

void processCommand(const char* json, Transport source) {
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    if (error) {
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"error\":\"JSON parse failed: %s\"}", error.c_str());
        sendResponse(buf, source);
        return;
    }

    const char* cmd = doc["cmd"];
    if (!cmd) {
        sendResponse("{\"error\":\"Missing cmd field\"}", source);
        return;
    }

    if (strcmp(cmd, "emotion") == 0) {
        const char* state = doc["state"];
        if (state) {
            setEmotionByName(state);
            char buf[64];
            snprintf(buf, sizeof(buf), "{\"ok\":true,\"emotion\":\"%s\"}", state);
            sendResponse(buf, source);
        } else {
            sendResponse("{\"error\":\"Missing state field\"}", source);
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        sendStatus(source);
    }
    else if (strcmp(cmd, "record_start") == 0) {
        if (audioState == AUDIO_IDLE) {
            startRecording();
            sendResponse("{\"ok\":true,\"action\":\"recording\"}", source);
        } else {
            sendResponse("{\"error\":\"Already recording\"}", source);
        }
    }
    else if (strcmp(cmd, "record_stop") == 0) {
        if (audioState == AUDIO_STREAMING) {
            stopRecording();
            sendResponse("{\"ok\":true,\"action\":\"stopped\"}", source);
        } else {
            sendResponse("{\"error\":\"Not recording\"}", source);
        }
    }
    else if (strcmp(cmd, "play_start") == 0) {
        if (audioState == AUDIO_IDLE) {
            startPlaybackMode(source);
        } else {
            sendResponse("{\"error\":\"Not idle\"}", source);
        }
    }
    else if (strcmp(cmd, "play_stop") == 0) {
        // Signal end of stream - playback continues until ring drains
        ringStreamEnded = true;
        sendResponse("{\"ok\":true,\"action\":\"stream_ended\"}", source);
        Serial.println("[AUDIO] Stream end marker set");
    }
    else if (strcmp(cmd, "wifi_config") == 0) {
        const char* ssid = doc["ssid"];
        const char* pass = doc["pass"];
        if (ssid) {
            preferences.begin("hinze", false);  // read-write
            preferences.putString("wifi_ssid", ssid);
            preferences.putString("wifi_pass", pass ? pass : "");
            preferences.end();
            char buf[128];
            snprintf(buf, sizeof(buf), "{\"ok\":true,\"action\":\"wifi_saved\",\"ssid\":\"%s\"}", ssid);
            sendResponse(buf, source);
            Serial.printf("[WIFI] Credentials saved. Reboot to connect.\n");
        } else {
            sendResponse("{\"error\":\"Missing ssid field\"}", source);
        }
    }
    else if (strcmp(cmd, "wifi_status") == 0) {
        char buf[256];
        if (wifiConnected) {
            snprintf(buf, sizeof(buf),
                "{\"ok\":true,\"wifi\":\"connected\",\"ip\":\"%s\",\"rssi\":%d,\"tcp_client\":%s}",
                WiFi.localIP().toString().c_str(),
                WiFi.RSSI(),
                tcpClientConnected ? "true" : "false");
        } else {
            snprintf(buf, sizeof(buf), "{\"ok\":true,\"wifi\":\"disconnected\"}");
        }
        sendResponse(buf, source);
    }
    else if (strcmp(cmd, "test_tone") == 0) {
        // Play alternating tones: beep-beep pattern
        Serial.println("[TEST] Playing beep pattern (low-high-low-high) 32-bit...");
        i2s_driver_uninstall(I2S_PORT_MIC);
        setupI2SSpk();

        int32_t toneBuffer[512];  // 32-bit stereo buffer (L, R, L, R, ...)
        size_t bytesWritten;
        float phase = 0.0f;
        const float PI2 = 6.28318530718f;

        // Play 4 beeps: low, high, low, high (0.5 sec each)
        float frequencies[] = {400.0f, 800.0f, 400.0f, 800.0f};

        for (int tone = 0; tone < 4; tone++) {
            float freq = frequencies[tone];
            float phaseInc = freq / 8000.0f * PI2;
            phase = 0.0f;

            Serial.printf("[TEST] %.0fHz...\n", freq);

            // 0.5 sec = 8000 samples = 31 buffers of 256 stereo samples
            for (int repeat = 0; repeat < 31; repeat++) {
                for (int i = 0; i < 256; i++) {
                    int32_t sample = (int32_t)(sinf(phase) * 10000) << 16;
                    toneBuffer[i * 2] = sample;      // Left
                    toneBuffer[i * 2 + 1] = sample;  // Right
                    phase += phaseInc;
                    if (phase >= PI2) phase -= PI2;
                }
                i2s_write(I2S_PORT_SPK, toneBuffer, 2048, &bytesWritten, portMAX_DELAY);
            }
        }

        i2s_driver_uninstall(I2S_PORT_SPK);
        setupI2SMic();
        sendResponse("{\"ok\":true,\"action\":\"test_complete\"}", source);
    }
    else {
        char buf[128];
        snprintf(buf, sizeof(buf), "{\"error\":\"Unknown command: %s\"}", cmd);
        sendResponse(buf, source);
    }
}

void setEmotionByName(const char* name) {
    if (strcmp(name, "idle") == 0) currentEmotion = EMOTION_IDLE;
    else if (strcmp(name, "listening") == 0) currentEmotion = EMOTION_LISTENING;
    else if (strcmp(name, "thinking") == 0) currentEmotion = EMOTION_THINKING;
    else if (strcmp(name, "happy") == 0) currentEmotion = EMOTION_HAPPY;
    else if (strcmp(name, "sad") == 0) currentEmotion = EMOTION_SAD;
    else if (strcmp(name, "angry") == 0) currentEmotion = EMOTION_ANGRY;
    else if (strcmp(name, "surprised") == 0) currentEmotion = EMOTION_SURPRISED;
    else if (strcmp(name, "confused") == 0) currentEmotion = EMOTION_CONFUSED;
    else if (strcmp(name, "sleepy") == 0) currentEmotion = EMOTION_SLEEPY;
    else if (strcmp(name, "excited") == 0) currentEmotion = EMOTION_EXCITED;

    // Reset animation states
    targetPupilX = 0;
    targetPupilY = 0;
    animationFrame = 0;
}

void sendStatus(Transport source) {
    const char* emotionNames[] = {
        "idle", "listening", "thinking", "happy", "sad",
        "angry", "surprised", "confused", "sleepy", "excited"
    };
    const char* audioStateNames[] = {
        "idle", "listening", "streaming", "buffering", "playing", "rebuffering"
    };

    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"status\":\"ok\",\"version\":\"0.7\",\"emotion\":\"%s\",\"audio\":\"%s\",\"wifi\":\"%s\",\"ring_buf\":%d}",
        emotionNames[currentEmotion],
        audioStateNames[audioState],
        wifiConnected ? "connected" : "disconnected",
        ringAvailable());
    sendResponse(buf, source);
}
