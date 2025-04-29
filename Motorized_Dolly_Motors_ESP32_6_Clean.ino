#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <HardwareSerial.h>
#include <VescUart.h>
#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

#define LED_PIN     2
#define LED_COUNT   72

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
BLEServer* pServer;

VescUart UART;

float RPM = 0.0;
float RPM1 = 0.0;
float RPM2 = 0.0;

const bool serial_print_bool = true;

bool lights_on_bool = true;
bool send_RPM_data_bool = true;

CRGB leds[LED_COUNT];

float curr_Y_magnitude = 0.0;
float max_Y_magnitude = 2000.0;

float accel_factor = 0.25;
float decel_to_zero_factor = 0.60;

float division_factor = 2400.0;
float subtraction_factor = 0.0;

float value_shift = 0.0;
float reverse_factor = 1.0;

float currentDuty1 = 0.0, targetDuty1 = 0.0, currentDuty2 = 0.0, targetDuty2 = 0.0;

float latestJoystickY = 0.0;
float latestJoystickX = 0.0;

uint8_t hue = 0;

void accelerateToMax(HardwareSerial &serial, float &currentDuty, float &targetDuty, float maxDuty, float accelerationRate);
void sendDutyCycle(HardwareSerial &serial, float duty);
uint16_t crc16(const uint8_t* data, uint16_t len);

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value = pCharacteristic->getValue().c_str();
    if (value.length() > 0) {
      if (serial_print_bool) Serial.println("Received: " + value);
      if (value.startsWith("ts,")) {
        int firstComma = value.indexOf(',');
        int secondComma = value.indexOf(',', firstComma + 1);
        int thirdComma = value.indexOf(',', secondComma + 1);

        if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
          String topSpeedStr = value.substring(firstComma + 1, secondComma);
          String accelerationStr = value.substring(thirdComma + 1);

          value_shift = topSpeedStr.toFloat() - 2400.0;
          accel_factor = (accelerationStr.toFloat()) * 1.0 / 1000.0;

          if (serial_print_bool) Serial.println("Top Speed value shift: " + String(value_shift) + ", Acceleration: " + String(accel_factor));
        } else {
          if (serial_print_bool) Serial.println("Invalid format for 'ts,<top_speed>,a,<acceleration>'");
        }
      } else if (value.startsWith("Resume")) {
        send_RPM_data_bool = true;
      } else if (value.startsWith("Pause")) {
        send_RPM_data_bool = false;
      } else if (value.startsWith("L")) {
        lights_on_bool = !lights_on_bool;
        if (serial_print_bool) Serial.println(lights_on_bool ? "Turning on the lights." : "Turning off the lights.");
        FastLED.setBrightness(lights_on_bool ? 100 : 0);
        FastLED.show();
      } else {
        float value1, value2;
        sscanf(value.c_str(), "%f,%f", &value1, &value2);

        latestJoystickY = value1;
        latestJoystickX = value2;

        float motor_val_1 = reverse_factor * (value1 + value_shift) / division_factor - reverse_factor * (value2 + value_shift) / division_factor;
        float motor_val_2 = reverse_factor * (value1 + value_shift) / division_factor + reverse_factor * (value2 + value_shift) / division_factor;

        if ((value1 != 0.0) || (value2 != 0.0)) {
          accelerateToMax(Serial1, currentDuty1, targetDuty1, motor_val_1, accel_factor);
          accelerateToMax(Serial2, currentDuty2, targetDuty2, motor_val_2, accel_factor);
        } else {
          accelerateToMax(Serial1, currentDuty1, targetDuty1, 0.0, decel_to_zero_factor);
          accelerateToMax(Serial2, currentDuty2, targetDuty2, 0.0, decel_to_zero_factor);
        }

        curr_Y_magnitude = abs(value1);
        if (lights_on_bool) {
          hue = map(curr_Y_magnitude, 0, max_Y_magnitude, 96, 0);
          fill_solid(leds, LED_COUNT, CHSV(hue, 255, 255));
          FastLED.show();
        }
      }
    }
  }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    if (serial_print_bool) Serial.println("Device connected");
    fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
    FastLED.show();
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) {
    if (serial_print_bool) Serial.println("Device disconnected");
    deviceConnected = false;
    fill_solid(leds, LED_COUNT, CRGB(0, 255, 255));
    FastLED.show();
    pServer->getAdvertising()->start();
    if (serial_print_bool) Serial.println("Restarting advertising...");
  }
};

void setup() {
  if (serial_print_bool) Serial.begin(115200);
  delay(1200);

  Serial.println("Initializing...");

  Serial2.begin(115200, SERIAL_8N1, 18, 17);
  Serial1.begin(115200, SERIAL_8N1, 10, 9);

  UART.setSerialPort(&Serial1);

  BLEDevice::init("ESP32_S3_BLE");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pCharacteristic->setValue("0");
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("Initializing LED strip...");
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);
  FastLED.clear();
  FastLED.setBrightness(100);
  fill_solid(leds, LED_COUNT, CRGB(0, 255, 255));
  FastLED.show();
  Serial.println("Setup: LED strip initialized.");

  if (serial_print_bool) Serial.println("ESP32-S3 Dual VESC Control initialized.");
}

void loop() {
  static unsigned long lastNotifyTime = 0;
  const unsigned long notifyInterval = 200;

  if (!deviceConnected) {
    BLEDevice::getAdvertising()->start();
    if (serial_print_bool) Serial.println("Re-entering advertising mode...");
    delay(3000);
  }

  if (millis() - lastNotifyTime >= notifyInterval) {
    lastNotifyTime = millis();

    if (deviceConnected) {
      float motor_val_1 = reverse_factor * (latestJoystickY + value_shift) / division_factor
                        - reverse_factor * (latestJoystickX + value_shift) / division_factor;
      float motor_val_2 = reverse_factor * (latestJoystickY + value_shift) / division_factor
                        + reverse_factor * (latestJoystickX + value_shift) / division_factor;

      if ((latestJoystickY != 0.0) || (latestJoystickX != 0.0)) {
        accelerateToMax(Serial1, currentDuty1, targetDuty1, motor_val_1, accel_factor);
        accelerateToMax(Serial2, currentDuty2, targetDuty2, motor_val_2, accel_factor);
      } else {
        accelerateToMax(Serial1, currentDuty1, targetDuty1, 0.0, decel_to_zero_factor);
        accelerateToMax(Serial2, currentDuty2, targetDuty2, 0.0, decel_to_zero_factor);
      }

      curr_Y_magnitude = abs(latestJoystickY);
      if (lights_on_bool) {
        hue = map(curr_Y_magnitude, 0, max_Y_magnitude, 96, 0);
        fill_solid(leds, LED_COUNT, CHSV(hue, 255, 255));
        FastLED.show();
      }
    }

    if (deviceConnected && send_RPM_data_bool && UART.getVescValues()) {
      RPM = UART.data.rpm;
      String rpmStr = String(RPM);
      pCharacteristic->setValue(rpmStr.c_str());
      pCharacteristic->notify();
    }
  }
}

void accelerateToMax(HardwareSerial &serial, float &currentDuty, float &targetDuty, float maxDuty, float accelerationRate) {
  maxDuty = constrain(maxDuty, -1.0, 1.0);
  if (maxDuty != targetDuty) targetDuty = maxDuty;

  if (currentDuty < targetDuty) {
    currentDuty += accelerationRate;
    if (currentDuty > targetDuty) currentDuty = targetDuty;
  } else if (currentDuty > targetDuty) {
    currentDuty -= accelerationRate;
    if (currentDuty < targetDuty) currentDuty = targetDuty;
  }

  sendDutyCycle(serial, currentDuty);
}

void sendDutyCycle(HardwareSerial &serial, float duty) {
  int32_t value = (int32_t)(duty * 100000.0);
  uint8_t payload[5] = { 5, (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                         (uint8_t)(value >> 8), (uint8_t)value };

  uint16_t crc = crc16(payload, 5);
  serial.write(0x02);
  serial.write(0x05);
  serial.write(payload, 5);
  serial.write((uint8_t)(crc >> 8));
  serial.write((uint8_t)(crc & 0xFF));
  serial.write(0x03);
}

uint16_t crc16(const uint8_t* data, uint16_t len) {
  uint16_t crc = 0;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= ((uint16_t)data[i] << 8);
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}