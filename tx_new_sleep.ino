#include <Arduino.h>
#include <Update.h>
#include <HTTPClient.h>
#include <esp_now.h>
#include <esp_sleep.h>
#include <WiFi.h>
#include <FastLED.h>
#include "esp_adc_cal.h"
#include <esp_wifi.h>
#include <EEPROM.h>
#include "BluetoothSerial.h"

#define device_prefix "petpooja_pager_"
#define NUM_LEDS 6
#define DATA_PIN 4
#define bt_mac_data_length 6
#define EEPROM_ADDRESS 0
#define uS_TO_S_FACTOR 1000000
#define S_TO_M_FACTOR 60
#define TIME_TO_SLEEP 15 * uS_TO_S_FACTOR *S_TO_M_FACTOR
// #define TIME_TO_SLEEP 30 * uS_TO_S_FACTOR
#define BUTTON_PIN_BITMASK 0x4000000
#define BUTTON_PIN_BITMASK1 0x1000
// char *WIFI_AP_SSID = "SSID";
// char *WIFI_AP_PASS = "PASSWORD";
const int MAX_SSID_LENGTH = 32, MAX_HOST_LENGTH = 500, MAX_PASSWORD_LENGTH = 64;
const int LED_PINS[] = { 13, 14, 16, 17, 18, 19 };
const int NUM_LEDS1 = sizeof(LED_PINS) / sizeof(LED_PINS[0]);
const char *pin = "1234";
char HOST[MAX_HOST_LENGTH + 1] = "", ssid[MAX_SSID_LENGTH + 1] = "", password[MAX_PASSWORD_LENGTH + 1] = "";
CRGB leds[NUM_LEDS];
esp_now_peer_info_t peerInfo;

uint8_t broadcast_address[] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF }, broadcastAddress1[6], storedMac[bt_mac_data_length], mac[6];

typedef struct struct_message {
  uint8_t rx_data[5];
} struct_message;
struct_message Receive;

typedef struct struct_message1 {
  uint8_t tx_data[4];
} struct_message1;
struct_message1 Trasmit;

esp_err_t result1;

int current_mode = 0, battery_percentage, myDevice_id;

// uint8_t mac[6]
String device_name;

int battery_pin = 35, charge_pin = 26, button_pin = 12, buzzer_pin = 32, vibrate_pin = 33;

const unsigned long val_to_min = 60000, debounce_delay = 500, send_interval = val_to_min;
const int divider = 10;

float pin_voltage = 0.0, bat_voltage = 0.0;

unsigned long previous_millis = 0, last_millis = 0, lastBluetoothInitTime = 0, bluetoothInitInterval = 60000;

bool repeat_flag = true, first_flag = false, second_flag = false, charge_flag = false, ack_flag = false, Bluetooth_Flag = false, Key_Flag = false,
     receivedSSID = false, receivedHOST = false, receivedPassword = false, bluetoothInitialized = false, Mac_Flag, OTA_Flag = true, Master_Flag = false, timer_started = false;

RTC_DATA_ATTR bool mode_flag = false, zero_flag = false;
RTC_DATA_ATTR int bat_counter = 0;
BluetoothSerial SerialBT;

void button_ISR() {
  if (millis() - previous_millis >= debounce_delay) {
    if (current_mode == 0 || (current_mode == 1 && ack_flag == false)) {
      repeat_flag = false;
      current_mode = 1;
    } else if (current_mode == 2) {
      repeat_flag = false;
    } else if (current_mode == 1) {
      current_mode = 11;
      repeat_flag = false;
    }
    previous_millis = millis();
  }
}
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  for (int i = 0; i < NUM_LEDS1; i++) {
    pinMode(LED_PINS[i], OUTPUT);
  }
  // esp_wifi_set_ps(WIFI_PS_NONE);
  // WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  Serial.println(String(mac[0], HEX) + ":" + String(mac[1], HEX) + ":" + String(mac[2], HEX) + ":" + String(mac[3], HEX) + ":" + String(mac[4], HEX) + ":" + String(mac[5], HEX));
  device_name = device_prefix + String(map((mac[3] + mac[4] + mac[5]) & 0xFF, 0, 255, 1, 255));
  myDevice_id = String(map((mac[3] + mac[4] + mac[5]) & 0xFF, 0, 255, 1, 255)).toInt();
  Serial.println("device_name : " + String(device_name));
  Serial.println("myDevice_id : " + String(myDevice_id));

  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  Bluetooth_Flag = false;
  pinMode(vibrate_pin, OUTPUT);
  pinMode(buzzer_pin, OUTPUT);
  pinMode(charge_pin, INPUT);
  pinMode(button_pin, INPUT_PULLUP);
  if (current_mode == 0 && digitalRead(button_pin) == LOW) {
    repeat_flag = false;
  }
  if (repeat_flag == false) {
    current_mode = 1;
  }
  //  attachInterrupt(digitalPinToInterrupt(charge_pin), charge_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(button_pin), button_ISR, FALLING);
  initEEPROM();

  Mac_Flag = readMacFromEEPROMSetup(broadcastAddress1);
  if (esp_now_init() != ESP_OK) return;

  memcpy(peerInfo.peer_addr, broadcast_address, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) return;

  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  if (digitalRead(charge_pin) == 1) {
    readMacFromEEPROM(broadcastAddress1);  // Read the stored MAC address from EEPROM
    printMacAddress(broadcastAddress1);
    if (mode_flag == false) {
      mode_flag = true;
      zero_flag = false;
      if (Bluetooth_Flag == false) {
        WiFi.disconnect();
        while (WiFi.status() != WL_DISCONNECTED) {
          delay(100);
        }

        if (!bluetoothInitialized) {
          for (int i = 0; i < 6; i++) {
            leds[i] = CRGB::CornflowerBlue;
          }
          FastLED.show();
          for (int i = 0; i < NUM_LEDS1; i++) {
            // pinMode(LED_PINS[i], OUTPUT);
            digitalWrite(LED_PINS[i], HIGH);
          }
          Key_Flag = false;
          receivedSSID = false;
          receivedPassword = false;
          receivedHOST = false;
          SerialBT.begin(device_name);
          Serial.printf("The device with name \"%s\" is started.\nNow you can pair it with Bluetooth!\n", device_name.c_str());

          bluetoothInitialized = true;  // Set the flag to true to indicate Bluetooth is now initialized
          lastBluetoothInitTime = millis();
        }

        while (true) {
          if (SerialBT.available()) {
            if (Key_Flag == false) {
              String receivedString1 = SerialBT.readString();
              receivedString1.trim();
              Serial.println("Received Message: " + receivedString1);

              if (receivedString1.equalsIgnoreCase("kavan")) {
                Key_Flag = true;
              }
            }

            if (Key_Flag) {
              while (!SerialBT.available()) {
              }
              String receivedString = SerialBT.readStringUntil('\n');  // Read until newline character
              Serial.println(receivedString);
              // Check if the received string contains SSID
              if (receivedString.startsWith("ssid:")) {
                receivedString.remove(0, 5);                             // Remove "ssid:" prefix
                receivedString.trim();                                   // Remove leading/trailing whitespace
                strncpy(ssid, receivedString.c_str(), MAX_SSID_LENGTH);  // Copy SSID to ssid array
                ssid[MAX_SSID_LENGTH] = '\0';                            // Ensure null-terminated
                receivedSSID = true;
                Serial.println("Received SSID: " + receivedString);
              }
              // Check if the received string contains password
              else if (receivedString.startsWith("password:")) {
                receivedString.remove(0, 9);                                     // Remove "password:" prefix
                receivedString.trim();                                           // Remove leading/trailing whitespace
                strncpy(password, receivedString.c_str(), MAX_PASSWORD_LENGTH);  // Copy password to password array
                password[MAX_PASSWORD_LENGTH] = '\0';                            // Ensure null-terminated
                receivedPassword = true;
                Serial.println("Received Password: " + receivedString);

              } else if (receivedString.startsWith("host:")) {
                receivedString.remove(0, 5);                             // Remove "ssid:" prefix
                receivedString.trim();                                   // Remove leading/trailing whitespace
                strncpy(HOST, receivedString.c_str(), MAX_HOST_LENGTH);  // Copy SSID to ssid array
                HOST[MAX_HOST_LENGTH] = '\0';                            // Ensure null-terminated
                receivedHOST = true;
                Serial.println("Received HOST " + receivedString);

                if (receivedPassword && receivedSSID) {
                  Serial.println("BT turnning of...");
                  SerialBT.end();
                  delay(10);
                  connectToWiFi();
                }
              } else {

                for (int i = 0; i < 6; i++) {
                  broadcastAddress1[i] = receivedString[i];
                }

                Serial.print("Received MAC Address: ");
                printMacAddress(broadcastAddress1);
                writeMacToEEPROM(broadcastAddress1);
              }
            } else {
              Serial.println("error");
            }
          }
          // Check if 1 minute has passed since Bluetooth initialization
          if (bluetoothInitialized && millis() - lastBluetoothInitTime >= bluetoothInitInterval || digitalRead(charge_pin) == 0 || OTA_Flag == false) {
            Serial.println("BT turnning of...");
            SerialBT.end();  // Turn off Bluetooth
            bluetoothInitialized = false;

            // WiFi.mode(WIFI_STA);
            // WiFi.setSleep(WIFI_PS_NONE);
            // WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASS);
            // esp_wifi_set_ps(WIFI_PS_NONE);

            break;
          }
        }
        Bluetooth_Flag = true;
      }

      current_mode = 0;
      bat_counter = 0;
      send_tx_data(current_mode);
    } else {
      Bluetooth_Flag = false;
      get_battery_voltage();
      esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP);
      esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ALL_LOW);

      Serial.println("Sleep Start....");
      esp_deep_sleep_start();
    }
  } else {
    Bluetooth_Flag = false;
    if (zero_flag == true) {
      esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
      Serial.println("Sleep Start....mode 0");
      esp_deep_sleep_start();
    }
    Serial.print("current_mode= ");
    Serial.println(current_mode);
    Serial.print("repeat_flag= ");
    Serial.println(repeat_flag);
    // if (current_mode == 0 && repeat_flag == true) {
    //   Serial.println("yes");
    //   last_millis = millis();
    //   if (millis() - last_millis >= send_interval) {
    //     Serial.println("sleeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep");
    //     // if(sleep_flag==)
    //     // esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK1, ESP_EXT1_WAKEUP_ALL_LOW);
    //     esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);
    //     esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
    //     esp_deep_sleep_start();
    //   }
    // }
    if (current_mode == 0 && repeat_flag == true) {
      Serial.println("yes");
      if (!timer_started) {
        last_millis = millis();
        timer_started = true;
      }
      if (millis() - last_millis >= send_interval) {
        Serial.println("sleeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeeep");
        // Configure wakeup on GPIO_NUM_12 (logic low)
        esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);
        // Configure wakeup on BUTTON_PIN_BITMASK (any high)
        esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
        esp_deep_sleep_start();
      }
    } else {
      timer_started = false;
    }
    mode_flag = false;
    if (repeat_flag == false) {
      switch (current_mode) {
        case 1:
          for (int i = 0; i < NUM_LEDS1; i++) {
            // pinMode(LED_PINS[i], OUTPUT);
            digitalWrite(LED_PINS[i], HIGH);
          }
          for (int i = 0; i < 6; i++) {
            leds[i] = CRGB::Red;
          }
          FastLED.show();
          delay(1000);
          current_mode = 0;
          send_tx_data(current_mode);
          repeat_flag = true;
          last_millis = millis();

          // Sleep for one minute
          while (millis() - last_millis < send_interval) {
            delay(1500);
            for (int i = 0; i < NUM_LEDS1; i++) {
              // pinMode(LED_PINS[i], OUTPUT);
              digitalWrite(LED_PINS[i], LOW);
            }
            if (repeat_flag == false) {
              break;  // Exit the while loop if button is pressed again
            } else if (current_mode == 1) {
              repeat_flag = true;
              break;
            } else if (digitalRead(charge_pin) == 1) {
              repeat_flag = true;
              break;
            }
            delay(100);
          }
          if (repeat_flag == true && current_mode == 0 && digitalRead(charge_pin) == 0) {
            sleep_();
          }

          break;

        case 2:
          repeat_flag = true;
          digitalWrite(vibrate_pin, LOW);
          digitalWrite(buzzer_pin, LOW);
          battery_percentage = get_battery_percentage();
          Trasmit.tx_data[0] = myDevice_id;
          Trasmit.tx_data[1] = 22;
          Trasmit.tx_data[2] = battery_percentage;
          Trasmit.tx_data[3] = 0;
          result1 = esp_now_send(broadcast_address, (uint8_t *)&Trasmit, sizeof(Trasmit));
          if (result1 == ESP_OK) Serial.println("pdata=" + String(Trasmit.tx_data[0]) + "," + String(Trasmit.tx_data[1]) + "," + String(Trasmit.tx_data[2]) + "," + String(Trasmit.tx_data[3]));

          else Serial.println("Send Failed....");
          break;

        case 11:
          repeat_flag = true;
          battery_percentage = get_battery_percentage();
          Trasmit.tx_data[0] = myDevice_id;
          Trasmit.tx_data[1] = 11;
          Trasmit.tx_data[2] = battery_percentage;
          Trasmit.tx_data[3] = 0;
          result1 = esp_now_send(broadcast_address, (uint8_t *)&Trasmit, sizeof(Trasmit));
          if (result1 == ESP_OK) Serial.println("pdata=" + String(Trasmit.tx_data[0]) + "," + String(Trasmit.tx_data[1]) + "," + String(Trasmit.tx_data[2]) + "," + String(Trasmit.tx_data[3]));

          else Serial.println("Send Failed....");
          break;
      }
    } else {
      if (current_mode == 2) {
        if (second_flag == false) {
          second_flag = true;
          digitalWrite(vibrate_pin, HIGH);
          digitalWrite(buzzer_pin, HIGH);
          battery_percentage = get_battery_percentage();
          Trasmit.tx_data[0] = myDevice_id;
          Trasmit.tx_data[1] = 2;
          Trasmit.tx_data[2] = battery_percentage;
          Trasmit.tx_data[3] = 0;
          result1 = esp_now_send(broadcast_address, (uint8_t *)&Trasmit, sizeof(Trasmit));
          if (result1 == ESP_OK) Serial.println("pdata=" + String(Trasmit.tx_data[0]) + "," + String(Trasmit.tx_data[1]) + "," + String(Trasmit.tx_data[2]) + "," + String(Trasmit.tx_data[3]));

          else Serial.println("Send Failed....");
          last_millis = millis();
        } else {
          if (millis() - last_millis >= send_interval) {
            second_flag = false;
          }
        }
      } else if (current_mode == 1) {
        if (first_flag == false) {
          first_flag = true;
          ack_flag = false;
          send_tx_data(current_mode);
          last_millis = millis();
        } else {
          if (millis() - last_millis >= send_interval) {
            first_flag = false;
          }
        }
      } else if (current_mode == 0) {
        for (int i = 0; i < 6; i++) {
          leds[i] = CRGB::Black;
        }
        FastLED.show();
        for (int i = 0; i < NUM_LEDS1; i++) {
          // pinMode(LED_PINS[i], OUTPUT);
          digitalWrite(LED_PINS[i], LOW);
        }
      } else if (current_mode == 11) {
        current_mode = 1;
      } else {
        Serial.println("Do nothing...");
      }
    }
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len == 5) {
    char macStr[18];
    //    Serial.print("Packet received from: ");
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    //    Serial.println(macStr);
    int master = 0;
    if (!Mac_Flag) {

      for (int i = 0; i < 6; i++) {
        if (broadcastAddress1[i] == mac[i]) {
          master++;
        }
      }
      if (master == 6) {
        Serial.println("mac matched");
        Master_Flag = true;
      } else {

        Serial.println("mac not matched");
        Master_Flag = false;
      }
    } else {
      memcpy(&Receive, incomingData, sizeof(Receive));
      if (Receive.rx_data[0] == myDevice_id) {
        if (Receive.rx_data[1] == 2 && current_mode == 1) {
          ack_data();
        } else if (Receive.rx_data[1] == 1 && current_mode == 0) {
          current_mode = 1;
          for (int i = 0; i < 6; i++) {
            leds[i] = CRGB::Purple;
          }
          FastLED.show();
          for (int i = 0; i < NUM_LEDS1; i++) {
            // pinMode(LED_PINS[i], OUTPUT);
            digitalWrite(LED_PINS[i], HIGH);
          }
          Serial.println("Data recvd successfully : " + String(Receive.rx_data[0]) + "," + String(Receive.rx_data[1]));
        } else if (Receive.rx_data[1] == 0 && current_mode == 2) {
          Serial.println("Mode0");
          zero_flag = true;
        } else {
          compare_rx_data();
        }
      }
    }
    if (Master_Flag) {
      memcpy(&Receive, incomingData, sizeof(Receive));
      if (Receive.rx_data[0] == myDevice_id) {
        if (Receive.rx_data[1] == 2 && current_mode == 1) {
          ack_data();
        } else if (Receive.rx_data[1] == 1 && current_mode == 0) {
          current_mode = 1;
          for (int i = 0; i < 6; i++) {
            leds[i] = CRGB::Purple;
          }
          FastLED.show();
          for (int i = 0; i < NUM_LEDS1; i++) {
            // pinMode(LED_PINS[i], OUTPUT);
            digitalWrite(LED_PINS[i], HIGH);
          }
          Serial.println("Data recvd successfully : " + String(Receive.rx_data[0]) + "," + String(Receive.rx_data[1]));
        } else if (Receive.rx_data[1] == 0 && current_mode == 2) {
          Serial.println("Mode0");
          zero_flag = true;
        } else {
          compare_rx_data();
        }
      }
    }
  }
}

void compare_rx_data() {
  if (Trasmit.tx_data[0] == Receive.rx_data[0] && Trasmit.tx_data[1] == Receive.rx_data[1]) {
    ack_flag = true;
    delay(1500);
    for (int i = 0; i < 6; i++) {
      leds[i] = CRGB::Black;
    }
    FastLED.show();
    for (int i = 0; i < NUM_LEDS1; i++) {
      // pinMode(LED_PINS[i], OUTPUT);
      digitalWrite(LED_PINS[i], LOW);
    }
  }
}

void ack_data() {
  battery_percentage = get_battery_percentage();
  Trasmit.tx_data[0] = Receive.rx_data[0];
  Trasmit.tx_data[1] = Receive.rx_data[1];
  Trasmit.tx_data[2] = battery_percentage;
  Trasmit.tx_data[3] = 0;
  current_mode = 2;
  ack_flag = false;
  esp_err_t result = esp_now_send(broadcast_address, (uint8_t *)&Trasmit, sizeof(Trasmit));
  if (result == ESP_OK) Serial.println("pdata=" + String(Trasmit.tx_data[0]) + "," + String(Trasmit.tx_data[1]) + "," + String(Trasmit.tx_data[2]) + "," + String(Trasmit.tx_data[3]));
  else Serial.println("Send Failed....");
  for (int i = 0; i < 6; i++) {
    leds[i] = CRGB::Green;
  }
  FastLED.show();
  for (int i = 0; i < NUM_LEDS1; i++) {
    // pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[i], HIGH);
  }
}

void send_tx_data(int next_mode) {

  battery_percentage = get_battery_percentage();
  Trasmit.tx_data[0] = myDevice_id;
  Trasmit.tx_data[1] = next_mode;
  Trasmit.tx_data[2] = battery_percentage;
  Trasmit.tx_data[3] = 0;
  if (next_mode == 1) {

    for (int i = 0; i < 40; i++) {
      while (!ack_flag) {
        result1 = esp_now_send(broadcast_address, (uint8_t *)&Trasmit, sizeof(Trasmit));
        delay(1500);
        for (int i = 0; i < 6; i++) {
          leds[i] = CRGB::Black;
        }
        FastLED.show();
        delay(150);
        break;
      }
    }
  } else {
    result1 = esp_now_send(broadcast_address, (uint8_t *)&Trasmit, sizeof(Trasmit));
  }
  if (result1 == ESP_OK) Serial.println("pdata=" + String(Trasmit.tx_data[0]) + "," + String(Trasmit.tx_data[1]) + "," + String(Trasmit.tx_data[2]) + "," + String(Trasmit.tx_data[3]));

  else Serial.println("Send Failed....");
}



uint8_t map_function(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int get_battery_percentage() {
  int percentage;
  unsigned long adc_reading = 0;
  for (int i = 0; i < divider; i++) {
    adc_reading += analogRead(battery_pin);
  }

  adc_reading /= divider;

  pin_voltage = ((readADC_Cal(adc_reading)) / 1000.0);
  bat_voltage = (pin_voltage * 130) / 100;
  Serial.println("Battery Voltage : " + String(bat_voltage));

  if (bat_voltage < 3.7) {
    float final_data = constrain(bat_voltage, 3.3, 3.7);
    //    Serial.println("Battery Voltage : " + String(final_data));

    percentage = map_function(final_data, 3.3, 3.7, 0, 20);
  } else {
    float final_data = constrain(bat_voltage, 3.7, 4.2);
    //    Serial.println("Battery Voltage : " + String(final_data));

    if (final_data > 3.70 && final_data <= 3.75) {
      percentage = 30;
    } else if (final_data > 3.75 && final_data <= 3.80) {
      percentage = 40;
    } else if (final_data > 3.80 && final_data <= 3.85) {
      percentage = 50;
    } else if (final_data > 3.85 && final_data <= 3.90) {
      percentage = 60;
    } else if (final_data > 3.90 && final_data <= 3.95) {
      percentage = 70;
    } else if (final_data > 3.95 && final_data <= 4.00) {
      percentage = 80;
    } else if (final_data > 4.00 && final_data <= 4.05) {
      percentage = 90;
    } else if (final_data > 4.05) {
      percentage = 100;
    }
  }
  return percentage;
}

void get_battery_voltage() {
  unsigned long adc_reading = 0;

  for (int i = 0; i < divider; i++) {
    adc_reading += analogRead(battery_pin);
  }

  adc_reading /= divider;

  pin_voltage = ((readADC_Cal(adc_reading)) / 1000.0);
  bat_voltage = (pin_voltage * 130) / 100;
  bat_voltage = bat_voltage - 0.15;

  Serial.print("Pin Volatge : ");
  Serial.print(pin_voltage);
  Serial.print("  Battery Volatge : ");
  Serial.println(bat_voltage);

  if (bat_voltage <= 3.7) {
    for (int i = 0; i < 6; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();

    // pinMode(LED_PINS[i], OUTPUT);
    digitalWrite(LED_PINS[0], HIGH);
    digitalWrite(LED_PINS[4], HIGH);

  } else if (bat_voltage > 3.7 && bat_voltage < 3.9) {
    for (int i = 0; i < 6; i++) {
      leds[i] = CRGB::Orange;
    }
    FastLED.show();
    digitalWrite(LED_PINS[1], HIGH);
    digitalWrite(LED_PINS[3], HIGH);
  } else if (bat_voltage >= 3.9) {

    bat_counter = bat_counter + 1;

    if (bat_counter >= 12) {
      for (int i = 0; i < 6; i++) {
        leds[i] = CRGB::DarkGreen;
      }
      FastLED.show();
      digitalWrite(LED_PINS[2], HIGH);
      digitalWrite(LED_PINS[5], HIGH);

    } else {
      for (int i = 0; i < 6; i++) {
        leds[i] = CRGB::Orange;
      }
      FastLED.show();
      digitalWrite(LED_PINS[1], HIGH);
      digitalWrite(LED_PINS[3], HIGH);
    }
  }
}

uint32_t readADC_Cal(int ADC_Raw) {
  esp_adc_cal_characteristics_t adc_chars;

  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12, 1100, &adc_chars);
  return (esp_adc_cal_raw_to_voltage(ADC_Raw, &adc_chars));
}

void charge_ISR() {
  if (millis() - previous_millis >= debounce_delay) {
    if (digitalRead(charge_pin) == 1) charge_flag = true;
    else charge_flag = false;
  }
}
void connectToWiFi() {
  WiFi.begin(ssid, password);
  Serial.println("ssid : " + String(ssid));
  Serial.println("password : " + String(password));
  Serial.println("Connecting to WiFi...");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 300 && digitalRead(charge_pin)) {
    for (int i = 0; i < 6; i++) {
      leds[i] = CRGB::Yellow;
    }
    FastLED.show();
    delay(1000);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("");
    Serial.println("WiFi connected!");
    for (int i = 0; i < 6; i++) {
      leds[i] = CRGB::Purple;
    }
    FastLED.show();
    Serial.println("IP address: " + WiFi.localIP().toString());
    check_OTA_update();
  } else {
    Serial.println("");
    Serial.println("Failed to connect to WiFi!");
  }
}

bool check_OTA_update() {

  Serial.println("---OTA_UPDATE---");
  Serial.println(HOST);

  HTTPClient http;
  http.begin(HOST);
  int16_t httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    uint32_t contentLength = http.getSize();
    WiFiClient *client = http.getStreamPtr();
    Serial.printf("size:%d,\n", contentLength);

    if (Update.begin(contentLength)) {
      Serial.println("Downloading firmware binary...");
      size_t written = Update.writeStream(*client);
      Serial.print("written:");
      Serial.println(written);

      if (written == contentLength && Update.end() && Update.isFinished()) {
        Serial.println("Firmware update complete. Restarting...");
        for (int i = 0; i < 6; i++) {
          leds[i] = CRGB::Green;
        }
        FastLED.show();
        delay(2000);
        for (int i = 0; i < 6; i++) {
          leds[i] = CRGB::Black;
        }
        FastLED.show();
        ESP.restart();
        return true;
      } else {
        Serial.printf("Error:%d,\n", Update.getError());
        for (int i = 0; i < 6; i++) {
          leds[i] = CRGB::Red;
        }
        FastLED.show();
        delay(2000);
        OTA_Flag = false;
        Update.abort();
      }
    } else {
      Serial.print("Error: No space for OTA\n");
      for (int i = 0; i < 6; i++) {
        leds[i] = CRGB::Red;
      }
      FastLED.show();
      delay(2000);
      OTA_Flag = false;
    }
  } else {
    Serial.print("Firmware file for update not available\n");
    for (int i = 0; i < 6; i++) {
      leds[i] = CRGB::Red;
    }
    FastLED.show();
    delay(2000);
    OTA_Flag = false;
  }
  return false;
}
void initEEPROM() {
  if (!EEPROM.begin(100)) {  // Specify the EEPROM size (512 bytes)
    Serial.println("Failed to initialize EEPROM!");
    return;
  }
  Serial.println("EEPROM initialized successfully.");
}

void writeMacToEEPROM(uint8_t *mac) {
  for (int i = 0; i < bt_mac_data_length; i++) {
    EEPROM.write(EEPROM_ADDRESS + i, mac[i]);
  }
  EEPROM.commit();
}

void readMacFromEEPROM(uint8_t *mac) {
  for (int i = 0; i < bt_mac_data_length; i++) {
    mac[i] = EEPROM.read(EEPROM_ADDRESS + i);
  }
}

bool readMacFromEEPROMSetup(uint8_t *mac) {
  int temp = 0;
  for (int i = 0; i < bt_mac_data_length; i++) {
    mac[i] = EEPROM.read(EEPROM_ADDRESS + i);
    if ((mac[i] == 0) || (mac[i] == 255)) {
      temp++;
    }
  }
  if (temp == 6) {
    return true;
  } else {
    return false;
  }
}

// Function to print MAC address in HEX format
void printMacAddress(uint8_t *mac) {
  for (int i = 0; i < bt_mac_data_length; i++) {
    Serial.print(mac[i], HEX);
    if (i < bt_mac_data_length - 1) {
      Serial.print(":");
    }
  }
  Serial.println();
}
void sleep_() {
  Serial.println("thank you");
  // if(sleep_flag==)
  // esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK1, ESP_EXT1_WAKEUP_ALL_LOW);
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_12, 0);
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);
  esp_deep_sleep_start();
}