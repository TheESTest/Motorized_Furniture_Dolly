/*
Elation Sports Technologies LLC
Austin Allen

Motorized Furniture Dolly

Flipsky dual ESC Serial/UART motor control with acceleration function.
This code receives 2 x joystick values, and sets corresponding
total speed and turning controls for 2 x BLDC motors using the
Flipsky dual ESC running VESC 6 firmware.


*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <HardwareSerial.h>

#include <VescUart.h> //SolidGeek

#include <Adafruit_NeoPixel.h>
#include <FastLED.h>

#define LED_PIN     2   // GPIO2 for data signal
#define LED_COUNT   72   // Number of LEDs in the strip

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLECharacteristic *pCharacteristic; // Declare the characteristic globally

bool deviceConnected = false; // Tracks BLE connection status

VescUart UART;

float RPM = 0.0;

float RPM1 = 0.0;
float RPM2 = 0.0;

const bool serial_print_bool = true;

bool lights_on_bool = true;
bool send_RPM_data_bool = true; //Boolean for sending the RPM data from the motor controller over BLE

// Define LED array
CRGB leds[LED_COUNT];

//Used for controlling the color of the LEDs
float curr_Y_magnitude = 0.0;
float max_Y_magnitude = 2000.0;

float accel_factor = 0.25;
float decel_to_zero_factor = 0.60;

float division_factor = 2400.0;
float subtraction_factor = 0.0;

float value_shift = 0.0;

float reverse_factor = 1.0; //To reverse the motor direction if needed

float currentDuty1 = 0.0, targetDuty1 = 0.0, currentDuty2 = 0.0, targetDuty2 = 0.0;

// Function prototypes
void accelerateToMax(HardwareSerial &serial, float &currentDuty, float &targetDuty, float maxDuty, float accelerationRate);
void sendDutyCycle(HardwareSerial &serial, float duty);
uint16_t crc16(const uint8_t* data, uint16_t len);

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String value = pCharacteristic->getValue().c_str();
        if (value.length() > 0) {
            if (serial_print_bool) Serial.println("Received: " + value);
            if (value.startsWith("ts,")) {

                // Handle the special "ts,<top_speed_curr>,a,<acceleration_curr>" format
                int firstComma = value.indexOf(',');
                int secondComma = value.indexOf(',', firstComma + 1);
                int thirdComma = value.indexOf(',', secondComma + 1);

                if (firstComma != -1 && secondComma != -1 && thirdComma != -1) {
                    String topSpeedStr = value.substring(firstComma + 1, secondComma);
                    String accelerationStr = value.substring(thirdComma + 1);

                    value_shift = topSpeedStr.toFloat() - 2400.0;
                    accel_factor = (accelerationStr.toFloat())* 1.0/1000.0; //Need this denominator to equal that from the client ESP32-S3 (controller) code

                    if (serial_print_bool) Serial.println("Top Speed value shift: " + String(value_shift) + ", Acceleration: " + String(accel_factor));

                } else {
                    if (serial_print_bool) Serial.println("Invalid format for 'ts,<top_speed>,a,<acceleration>'");
                }
            }

            else if (value.startsWith("Resume")) {
              send_RPM_data_bool = true;
            }

            else if (value.startsWith("Pause")) {
              send_RPM_data_bool = false;
            }
            
            //Turn the lights on or off
            else if (value.startsWith("L")) {
              if (lights_on_bool == true){
                lights_on_bool = false;
                if (serial_print_bool) Serial.println("Turning off the lights.");
                FastLED.setBrightness(0);
                FastLED.show();
              }
              else{
                lights_on_bool = true;
                if (serial_print_bool) Serial.println("Turning on the lights.");
                FastLED.setBrightness(100);
                FastLED.show();
              }
            }
            
            // Handle the "<value1>,<value2>" format
            else {
                float value1, value2;
                sscanf(value.c_str(), "%f,%f", &value1, &value2);

                float motor_val_1 = reverse_factor * (value1 + value_shift) / division_factor - reverse_factor * (value2 + value_shift) / division_factor;
                float motor_val_2 = reverse_factor * (value1 + value_shift) / division_factor + reverse_factor * (value2 + value_shift) / division_factor;

                if (value1 != 0.0 || value2 != 0.0) {
                    accelerateToMax(Serial1, currentDuty1, targetDuty1, motor_val_1, accel_factor);
                    accelerateToMax(Serial2, currentDuty2, targetDuty2, motor_val_2, accel_factor);
                } else {
                    accelerateToMax(Serial1, currentDuty1, targetDuty1, 0.0, decel_to_zero_factor);
                    accelerateToMax(Serial2, currentDuty2, targetDuty2, 0.0, decel_to_zero_factor);
                }

                curr_Y_magnitude = abs(value1);
                if (lights_on_bool == true) {
                    // Map speed to a hue range (0 = Red, 96 = Green)
                    uint8_t hue = map(curr_Y_magnitude, 0, max_Y_magnitude, 96, 0);  
                    fill_solid(leds, LED_COUNT, CHSV(hue, 255, 255)); 
                    FastLED.show();
                }

            }
        }
    }

    void adjustMotorSpeed(int topSpeed, int acceleration) {
        if (serial_print_bool) Serial.printf("Adjusting motor speed: Top Speed = %d, Acceleration = %d\n", topSpeed, acceleration);
    }
};

class ServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
        if (serial_print_bool) Serial.println("Device connected");

        //Show green upon connect
        for (int i = 0; i < LED_COUNT; i++) {
          fill_solid(leds, LED_COUNT, CRGB(0, 255, 0));
        }
        FastLED.show();

        deviceConnected = true; // Set connection status to true
    }

    void onDisconnect(BLEServer* pServer) {
        if (serial_print_bool) Serial.println("Device disconnected");
        deviceConnected = false; // Set connection status to false

        //Show cyan upon disconnect (and on startup)
        for (int i = 0; i < LED_COUNT; i++) {
          fill_solid(leds, LED_COUNT, CRGB(0, 255, 255));
        }
        FastLED.show();

        // Restart advertising
        pServer->startAdvertising();
        if (serial_print_bool) Serial.println("Restarting advertising...");
    }
};




void setup() {
  if (serial_print_bool) Serial.begin(115200);
  delay(1200);

  Serial.println("Initializing...");

  // Initialize UART communication
  Serial2.begin(115200, SERIAL_8N1, 18, 17);
  Serial1.begin(115200, SERIAL_8N1, 10, 9);

  UART.setSerialPort(&Serial1);

  // Initialize BLE
  BLEDevice::init("ESP32_S3_BLE");
  BLEServer *pServer = BLEDevice::createServer();

  // Set server callbacks
  pServer->setCallbacks(new ServerCallbacks());

  // Create a BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY // Enable notifications
  );

  // Initialize the characteristic with a default value
  pCharacteristic->setValue("0"); // Initial RPM value

  // Set callback for characteristic
  pCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();

  Serial.println("Initializing LED strip...");

  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, LED_COUNT);  // Change to GRBW for RGBW strips
  FastLED.clear(); // Ensure all LEDs are off at start
  FastLED.setBrightness(100); // Set initial brightness
  //Set cyan for the initial color
  for (int i = 0; i < LED_COUNT; i++) {
    fill_solid(leds, LED_COUNT, CRGB(0, 255, 255));
  }
  FastLED.show();
  Serial.println("Setup: LED strip initialized.");

  if (serial_print_bool) Serial.println("ESP32-S3 Dual VESC Control initialized.");


}


void loop() {
    static unsigned long lastNotifyTime = 0;
    const unsigned long notifyInterval = 200;

      if (!deviceConnected) {
        BLEDevice::getAdvertising()->start();  // Ensure advertising is active
        if (serial_print_bool) Serial.println("Re-entering advertising mode...");
        delay(3000); // Avoid spamming the console/log
    }

    if (millis() - lastNotifyTime >= notifyInterval) {
        lastNotifyTime = millis(); // Update the last notification time

        if (deviceConnected) { // Only send notifications if BLE is connected
            if (UART.getVescValues()) {
                RPM = UART.data.rpm;

                // Convert RPM to string and send as notification

                //Only send the RPM data if this boolean is true
                if (send_RPM_data_bool == true){
                  String rpmStr = String(RPM);
                  if (serial_print_bool) Serial.println("Sending RPM: " + rpmStr);
                  pCharacteristic->setValue(rpmStr.c_str());
                  pCharacteristic->notify();
                }
            }
        }

    }
}


void accelerateToMax(HardwareSerial &serial, float &currentDuty, float &targetDuty, float maxDuty, float accelerationRate) {
  maxDuty = constrain(maxDuty, -1.0, 1.0);

  if (maxDuty != targetDuty) {
    targetDuty = maxDuty;
  }
  
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
  uint8_t payload[5];
  payload[0] = 5; //SET_DUTY
  payload[1] = value >> 24;
  payload[2] = value >> 16;
  payload[3] = value >> 8;
  payload[4] = value;

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
    crc = crc ^ ((uint16_t)data[i] << 8);
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}
