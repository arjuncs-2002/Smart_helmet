#define BLYNK_TEMPLATE_ID "TMPL38lh0wddk"
#define BLYNK_TEMPLATE_NAME "SMART HELMET"

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <BlynkSimpleEsp32.h>

// Blynk Credentials
#define BLYNK_AUTH_TOKEN "D5SmGPcM5C8X7ByIHgLJ92_JUhp3zgss"
#define WIFI_SSID "SNOW"
#define WIFI_PASS "12345678009"

// LCD & Sensor Objects
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_BMP280 bmp;

// Sensor and Output Pins
#define LM35_PIN 32
#define MQ135_PIN 34
#define VIBRATION_PIN 35
#define LED_PIN 5
#define BUZZER_PIN 18

// Thresholds
const float SAFE_TEMP_LOW = 15.0;
const float SAFE_TEMP_HIGH = 45.0;
const int SAFE_AIR_QUALITY = 900;
const float SAFE_PRESSURE_LOW = 900.0;
const float SAFE_PRESSURE_HIGH = 1100.0;

volatile bool vibrationDetected = false;

// Vibration Interrupt Function
void IRAM_ATTR detectVibration() {
    vibrationDetected = true;
}

void setup() {
    Serial.begin(115200);
    
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(VIBRATION_PIN, INPUT_PULLUP);

    digitalWrite(LED_PIN, LOW);
    digitalWrite(BUZZER_PIN, LOW);

    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.clear();

    // Initialize BMP280
    if (!bmp.begin(0x76)) {  
        Serial.println("BMP280 not detected!");
        while (1);
    }

    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);

    // Attach interrupt for vibration detection
    attachInterrupt(digitalPinToInterrupt(VIBRATION_PIN), detectVibration, FALLING);

    // Connect to WiFi & Blynk
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi!");
    
    Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASS);
}

void loop() {
    Blynk.run();

    bool isUnsafe = false;
    String dangerMessage = "";
    float dangerValue = 0.0;

    // Read LM35 Temperature Sensor
    int sensorValue = analogRead(LM35_PIN);
    float voltage = sensorValue * (3.3 / 4095.0);
    float temperature = voltage * 100.0;

    // Read MQ135 Air Quality Sensor
    int airQuality = analogRead(MQ135_PIN);

    // Read BMP280 Pressure Sensor
    float pressure = bmp.readPressure() / 100.0;

    // Send Data to Blynk
    Blynk.virtualWrite(V1, temperature);
    Blynk.virtualWrite(V2, pressure);
    Blynk.virtualWrite(V3, airQuality);

    // Check for Unsafe Conditions
    if (temperature < SAFE_TEMP_LOW) {  
        isUnsafe = true;
        dangerMessage = "LOW TEMP";
        dangerValue = temperature;
    } else if (temperature > SAFE_TEMP_HIGH) {
        isUnsafe = true;
        dangerMessage = "HIGH TEMP";
        dangerValue = temperature;
    } else if (airQuality > SAFE_AIR_QUALITY) {
        isUnsafe = true;
        dangerMessage = "BAD AIR";
        dangerValue = airQuality;
    } else if (pressure < SAFE_PRESSURE_LOW) {  
        isUnsafe = true;
        dangerMessage = "LOW PRESS";
        dangerValue = pressure;
    } else if (pressure > SAFE_PRESSURE_HIGH) { 
        isUnsafe = true;
        dangerMessage = "HIGH PRESS";
        dangerValue = pressure;
    }

    // Handle Vibration Alert
    if (vibrationDetected) {  
        Serial.println("⚠ VIBRATION DETECTED! ⚠");
        isUnsafe = true;
        dangerMessage = "VIBRATION";
        dangerValue = 1;

        // Send Vibration Data to Blynk
        Blynk.virtualWrite(V5, 1);
        Blynk.virtualWrite(V4, 255);

        for (int i = 0; i < 3; i++) {  
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(200);
            digitalWrite(LED_PIN, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            delay(200);
        }

        // Send Vibration Alert to Blynk
        Blynk.logEvent("vibration_alert", "Vibration detected!");

        delay(1000);
        vibrationDetected = false;

        // Reset Vibration Display on Blynk
        Blynk.virtualWrite(V5, 0);
        Blynk.virtualWrite(V4, 0);
    }

    // Display Data on LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temperature);
    lcd.print("C P:");
    lcd.print(pressure);
    lcd.print("hPa");

    if (isUnsafe) {
        Serial.println("⚠ UNSAFE CONDITION DETECTED! DANGER!!!!!!");
        
        // Show DANGER message with value on LCD
        lcd.setCursor(0, 1);
        lcd.print(dangerMessage);
        lcd.print(": ");
        lcd.print(dangerValue);
        lcd.print(" DANGER!");

        // Send alerts to Blynk
        Blynk.virtualWrite(V0, "DANGER!!!!!!");
        Blynk.virtualWrite(V4, 255);

        for (int i = 0; i < 5; i++) {
            digitalWrite(LED_PIN, HIGH);
            digitalWrite(BUZZER_PIN, HIGH);
            delay(300);
            digitalWrite(LED_PIN, LOW);
            digitalWrite(BUZZER_PIN, LOW);
            delay(300);
        }

        // Turn OFF LED Widget after 5 seconds
        delay(5000);
        Blynk.virtualWrite(V4, 0);
    } else {
        Serial.println("✅ STATUS NORMAL");

        // LCD Shows Status Normal
        lcd.setCursor(0, 1);
        lcd.print("Status: Normal");

        // Blynk Shows Status Normal
        Blynk.virtualWrite(V0, "Status: Normal");
    }

    delay(2000);
}
