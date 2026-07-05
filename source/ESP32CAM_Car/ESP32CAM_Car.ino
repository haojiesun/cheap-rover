
/*
 * @Date: 2023-8-16
 * @Description: ESP32 Camera Surveillance Car
 * @FilePath:
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_heap_caps.h"

//
// WARNING!!! Make sure that you have either selected ESP32 Wrover Module,
//            or another board which has PSRAM enabled
//
// Adafruit ESP32 Feather

// Select camera model
// #define CAMERA_MODEL_WROVER_KIT
// #define CAMERA_MODEL_M5STACK_PSRAM
#define CAMERA_MODEL_AI_THINKER

const char *ssidHome = "asus-2.4";
const char *passwordHome = "jacky78901234";

const char *ssidWork = "Pharos-BYOD";
const char *passwordWork = "KnockKnock!iwfw1392";

#if defined(CAMERA_MODEL_WROVER_KIT)
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 21
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 19
#define Y4_GPIO_NUM 18
#define Y3_GPIO_NUM 5
#define Y2_GPIO_NUM 4
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#elif defined(CAMERA_MODEL_AI_THINKER)
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27

#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#else
#error "Camera model not selected"
#endif

// GPIO Setting
extern int gpLb = 2;  // Left 1
extern int gpLf = 14; // Left 2
extern int gpRb = 15; // Right 1
extern int gpRf = 13; // Right 2
extern int gpLed = 4; // Light
extern String WiFiAddr = "";
extern const char* PASSCODE; // Passcode from app_httpd.cpp

// Safety timeout mechanism
extern unsigned long lastCommandTime = 0;
extern bool motorsRunning = false;
const unsigned long MOTOR_TIMEOUT_MS = 1500; // 1.5 seconds timeout

// WiFi monitoring for unattended operation
extern unsigned long lastWiFiActivity = 0;  // Tracks last HTTP request or command
const unsigned long WIFI_IDLE_CHECK_MS = 60000;  // Check WiFi only after 1 minute of no activity
const unsigned long WIFI_RECONNECT_TIMEOUT_MS = 30000;  // 30 seconds to try reconnecting

void startCameraServer();

void setup()
{
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();

  pinMode(gpLb, OUTPUT);  // Left Backward
  pinMode(gpLf, OUTPUT);  // Left Forward
  pinMode(gpRb, OUTPUT);  // Right Forward
  pinMode(gpRf, OUTPUT);  // Right Backward
  pinMode(gpLed, OUTPUT); // Light

  // initialize
  digitalWrite(gpLb, LOW);
  digitalWrite(gpLf, LOW);
  digitalWrite(gpRb, LOW);
  digitalWrite(gpRf, LOW);
  digitalWrite(gpLed, LOW);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  // init with high specs to pre-allocate larger buffers
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  }
  else
  {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  // drop down frame size for higher initial frame rate
  sensor_t *s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_CIF);

  // Try connecting to WiFi networks
  bool connected = false;
  int attempt = 0;
  const int maxAttempts = 20; // 10 seconds per network

  // Try work network first (fast double blink pattern)
  Serial.println("Attempting to connect to work WiFi...");
  WiFi.begin(ssidWork, passwordWork);

  attempt = 0;
  while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts)
  {
    // Fast double blink
    digitalWrite(gpLed, HIGH);
    delay(200);
    digitalWrite(gpLed, LOW);
    delay(200);
    digitalWrite(gpLed, HIGH);
    delay(200);
    digitalWrite(gpLed, LOW);
    delay(400);
    Serial.print(".");
    attempt++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    connected = true;
    Serial.println("");
    Serial.println("Connected to work WiFi!");
  }
  else
  {
    Serial.println("");
    Serial.println("Work WiFi connection failed. Trying home WiFi...");

    // Disconnect and try home network (slow single blink pattern)
    WiFi.disconnect();
    delay(100);
    WiFi.begin(ssidHome, passwordHome);

    attempt = 0;
    while (WiFi.status() != WL_CONNECTED && attempt < maxAttempts)
    {
      digitalWrite(gpLed, HIGH);
      delay(500);
      digitalWrite(gpLed, LOW);
      delay(500);
      Serial.print(".");
      attempt++;
    }

    if (WiFi.status() == WL_CONNECTED)
    {
      connected = true;
      Serial.println("");
      Serial.println("Connected to home WiFi!");
    }
    else
    {
      Serial.println("");
      Serial.println("Failed to connect to any WiFi network!");
    }
  }

  if (!connected)
  {
    Serial.println("ERROR: No WiFi connection. Restarting...");
    delay(3000);
    ESP.restart();
    return;
  }

  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.print(":8081/");
  Serial.print(PASSCODE);
  Serial.println("' to connect");
  WiFiAddr = WiFi.localIP().toString();
  
  // Initialize WiFi activity tracking
  lastWiFiActivity = millis();
}

void WheelAct(int nLf, int nLb, int nRf, int nRb)
{
  digitalWrite(gpLf, nLf);
  digitalWrite(gpLb, nLb);
  digitalWrite(gpRf, nRf);
  digitalWrite(gpRb, nRb);

  // Update motorsRunning based on actual motor state
  motorsRunning = (nLf == HIGH || nLb == HIGH || nRf == HIGH || nRb == HIGH);
}

// Print current heap stats. Used by the 30s monitor and the "heap" command.
void logHeap()
{
  Serial.printf("Heap: free=%u  min_free=%u  largest_block=%u\n",
                ESP.getFreeHeap(),
                ESP.getMinFreeHeap(),
                heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
}

// Read newline-terminated commands from the serial console and dispatch them.
// This is the only serial input path; the console was previously output-only.
void handleSerialCommands()
{
  static String cmdBuffer = "";

  while (Serial.available() > 0)
  {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r')
    {
      cmdBuffer.trim();
      if (cmdBuffer.length() == 0)
      {
        continue;
      }

      if (cmdBuffer.equalsIgnoreCase("heap"))
      {
        logHeap();
      }
      else if (cmdBuffer.equalsIgnoreCase("stop"))
      {
        WheelAct(LOW, LOW, LOW, LOW);
        Serial.println("Motors stopped");
      }
      else if (cmdBuffer.equalsIgnoreCase("ip"))
      {
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());
        Serial.print("WiFi status: ");
        Serial.println(WiFi.status() == WL_CONNECTED ? "connected" : "disconnected");
      }
      else if (cmdBuffer.equalsIgnoreCase("uptime"))
      {
        Serial.printf("Uptime: %lu ms\n", millis());
      }
      else if (cmdBuffer.equalsIgnoreCase("restart"))
      {
        Serial.println("Restarting ESP32...");
        delay(100);
        ESP.restart();
      }
      else if (cmdBuffer.equalsIgnoreCase("help"))
      {
        Serial.println("Commands: heap, stop, ip, uptime, restart, help");
      }
      else
      {
        Serial.printf("Unknown command: '%s' (type 'help')\n", cmdBuffer.c_str());
      }

      cmdBuffer = "";
    }
    else
    {
      cmdBuffer += c;
      // Guard against a runaway line with no newline exhausting memory.
      if (cmdBuffer.length() > 64)
      {
        cmdBuffer = "";
        Serial.println("Command too long, discarded");
      }
    }
  }
}

void loop()
{
  // Heap monitor: log free/min/largest-block every 30s to diagnose leaks vs fragmentation.
  // If largest_block shrinks while free stays flat -> fragmentation.
  // If free itself trends down over hours -> genuine leak.
  static unsigned long lastHeapLog = 0;
  if (millis() - lastHeapLog > 60000)
  {
    lastHeapLog = millis();
    logHeap();
  }

  // Handle any commands typed on the serial console
  handleSerialCommands();

  // Safety check: auto-stop motors if no command received within timeout
  if (motorsRunning && (millis() - lastCommandTime > MOTOR_TIMEOUT_MS))
  {
    WheelAct(LOW, LOW, LOW, LOW);
    Serial.println("AUTO-STOP: Motor timeout");
  }

  // WiFi monitoring: Only check when idle (no activity for over 1 minute)
  unsigned long currentMillis = millis();
  if (currentMillis - lastWiFiActivity > WIFI_IDLE_CHECK_MS)
  {
    // Check WiFi status only during idle periods
    if (WiFi.status() != WL_CONNECTED)
    {
      Serial.println("WiFi disconnected! Attempting reconnection...");
      
      // Try to reconnect
      WiFi.disconnect();
      WiFi.begin(ssidHome, passwordHome);
      
      unsigned long reconnectStart = millis();
      bool reconnected = false;
      
      while (millis() - reconnectStart < WIFI_RECONNECT_TIMEOUT_MS)
      {
        if (WiFi.status() == WL_CONNECTED)
        {
          reconnected = true;
          Serial.println("WiFi reconnected successfully!");
          Serial.print("IP: ");
          Serial.println(WiFi.localIP());
          lastWiFiActivity = millis(); // Reset activity timer
          break;
        }
        delay(500);
        Serial.print(".");
      }
      
      if (!reconnected)
      {
        Serial.println("\nWiFi reconnection failed. Restarting ESP32...");
        delay(1000);
        ESP.restart();
      }
    }
    else
    {
      // WiFi is connected, update activity time to check again after idle period
      lastWiFiActivity = currentMillis;
    }
  }

  delay(50); // Check every 50ms
}
