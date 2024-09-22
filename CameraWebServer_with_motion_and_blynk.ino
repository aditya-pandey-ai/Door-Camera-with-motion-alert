/**********************************************************************************
 *  Preferences--> Additional boards Manager URLs : https://dl.espressif.com/dl/package_esp32_index.json, http://arduino.esp8266.com/stable/package_esp8266com_index.json
 *  Board Settings:
 *  Board: "ESP32 Wrover Module"
 *  Upload Speed: "921600"
 *  Flash Frequency: "80MHz"
 *  Flash Mode: "QIO"
 *  Partition Scheme: "Hue APP (3MB No OTA/1MB SPIFFS)"
 *  Core Debug Level: "None"
 *  COM Port: Depends *On Your System*
 *  GPIO 0 must be connected to GND pin while uploading the sketch
 *  After connecting GPIO 0 to GND pin, press the ESP32 CAM on-board RESET button to put the board in flashing mode
 ***************************************************************************************/

#define BLYNK_TEMPLATE_ID "TMPL3pT7ncyb3"
#define BLYNK_TEMPLATE_NAME "Door Alarm"
#define BLYNK_AUTH_TOKEN "4ZbNtemGDV_dlG5fveKDfzUogpV4ZzmH"

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "soc/soc.h"           // Disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Disable brownout problems

// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
// Ensure ESP32 Wrover Module or other board with PSRAM is selected
// Partial images will be transmitted if image exceeds buffer size

// Select camera model
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include "camera_pins.h"

#define PIR 13        // PIR sensor connected to GPIO 13
#define PHOTO 14      // Button connected to GPIO 14
#define LED 4         // LED connected to GPIO 4

const char* ssid = "Galaxy S";       // WIFI NAME
const char* password = "Frequency";  // WIFI PASSWORD
char auth[] = "4ZbNtemGDV_dlG5fveKDfzUogpV4ZzmH";  // AUTH TOKEN sent by Blynk

String local_IP;

void startCameraServer();

void takePhoto()
{
  digitalWrite(LED, HIGH);
  delay(200);
  uint32_t randomNum = random(50000);
  Serial.println("http://"+local_IP+"/capture?_cb="+ (String)randomNum);
  Blynk.setProperty(V1, "urls", "http://"+local_IP+"/capture?_cb="+(String)randomNum);
  digitalWrite(LED, LOW);
  delay(1000);
}

// Blynk button on virtual pin V3 to take a photo
BLYNK_WRITE(V3) {
  bool RelayTwo = param.asInt();
  if (RelayTwo == 1) {
    digitalWrite(PHOTO, LOW);
    Serial.println("Capture Photo");
    takePhoto();
  }
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); // Disable brownout detector
  Serial.begin(115200);
  pinMode(LED, OUTPUT);   // Set LED pin as output
  pinMode(PIR, INPUT);    // Set PIR pin as input
  pinMode(PHOTO, INPUT);  // Set button pin as input
  Serial.setDebugOutput(true);
  Serial.println();

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
  
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);       // Flip it back
    s->set_brightness(s, 1);  // Adjust brightness slightly
    s->set_saturation(s, -2); // Adjust saturation
  }
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
  startCameraServer();
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  local_IP = WiFi.localIP().toString();
  Serial.println("' to connect");
  Blynk.begin(auth, ssid, password);
}

void loop() {
  Blynk.run();
  
  int pirStatus = digitalRead(PIR);  // Read PIR status

  if (pirStatus == HIGH) {
    Serial.println("Motion detected! Sending alert and capturing photo.");
    Blynk.logEvent("motion_detected", "Someone is there.");
    takePhoto();
    delay(3000);  // Wait for 3 seconds before taking the next photo
  }

  // Check if the button is pressed
  if (digitalRead(PHOTO) == LOW) {
    Serial.println("Button pressed, capturing photo.");
    takePhoto();
  }
}
