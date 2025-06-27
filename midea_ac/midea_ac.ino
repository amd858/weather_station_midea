
#include <Arduino.h>
#if !defined(ARDUINO_ESP32C3_DEV)  // This is due to a bug in RISC-V compiler, which requires unused function sections :-(.
#define DISABLE_CODE_FOR_RECEIVER  // Disables static receiver code like receive timer ISR handler and static IRReceiver and irparams data. Saves 450 bytes program memory and 269 bytes RAM if receiving functions are not required.
#endif

#include <ClosedCube_HDC1080.h>
#include <DHTesp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
 
#include "PinDefinitionsAndMore.h"  // Define macros for input and output pin etc.
#include <IRremote.hpp>

#define SEND_PWM_BY_TIMER

#define HUMIDIFIER_OFF_PIN 4
#define dip_switch_d 12
#define dip_switch_c 11
#define dip_switch_b 10
#define dip_switch_a 9
#define HEATER_MODE 8
#define ac_disabled_pin 7
#define humidifier_indication 14


IRsend irsend;
ClosedCube_HDC1080 hdc1080;
DHTesp dht;
LiquidCrystal_I2C lcd(0x27, 16, 4);

const int KEY_UP_TIME = 250;
const int KEY_DOWN_TIME = 50;
const int TEMP_UP_STEPS = 16;
const int temp_down_steps = 4;
const float HC_MAX_HIDEX = 27.2;
const float HC_MIN_HIDEX = 26.8;
const float ABOVE_ZERO_LEVEL_HUMD = 33;
const float ZERO_LEVEL_HUMD = 30;
const float MIN_HUMD = 39;  //55
const float MAX_HUMD = 43;  //69
const int MAX_BOUNCE_COUNT = 6;
const int BUTTON_BOUNCE_COUNT = 1;
const long REPEAT_ACTION_INTERVAL = 600000;
bool HEATER_MODE_SELECTOR;
unsigned long repeatActionOldTime = 0;
int bounce_count = 0;






float Dip_Switch_Offset() {
  int a_button = !digitalRead(dip_switch_a);  // 0.5
  int b_button = !digitalRead(dip_switch_b);  // 1.0
  int c_button = !digitalRead(dip_switch_c);  // 2.0
  int minus_button = !digitalRead(dip_switch_d);
  float result = (a_button * 0.5 + b_button * 1 + c_button * 2);
  if (minus_button) {
    result = result * -1;
  }
  return result;
}

enum AcStates {
  AC_OFF,              // AC is off
  BOUNCING_FOR_AC_ON,  // Temperature getting High
  AC_ON,
  BOUNCING_FOR_AC_OFF,  // Temperature Getting Low
  BOUNCING_FOR_AC_DISABLE,
  AC_DISABLED,
  BOUNCING_FOR_AC_ENABLE,
  COOL_TEMPERATURE_SENSOR_FAULT,
  HEAT_MODE
};

enum HeaterStates {
  HEATING_DISABLE,              // AC is off
  BOUNCING_FOR_HEATING_ENABLE,  // Temperature getting High
  HEATING_ENABLE,
  BOUNCING_FOR_HEATING_DISABLE,  // Temperature Getting Low
  BOUNCING_FOR_HEATER_OFF,
  HEATER_OFF,
  BOUNCING_FOR_HEATER_ON,
  HEATER_TEMPERATURE_SENSOR_FAULT,
  COOL_MODE
};
enum HumidifierStates {
  HUMIDIFIER_OFF,
  HUMIDIFIER_ON_BOUNCE,
  HUMIDIFIER_ON,
  HUMIDIFIER_OFF_BOUNCE

};

unsigned char heater_state = HEATING_DISABLE;
unsigned char ac_state = AC_OFF;
unsigned char humidifier_state = HUMIDIFIER_OFF;
const char* ac_state_lcd() {
  const char* result = "";
  switch (ac_state) {
    case AC_OFF:
      result = "AC_OFF";
      break;
    case BOUNCING_FOR_AC_ON:
      result = "BOUNCE_FOR_AC_ON";
      break;
    case AC_ON:
      result = "AC_ON";
      break;
    case BOUNCING_FOR_AC_OFF:
      result = "BOUNC_FOR_AC_OFF";
      break;
    case BOUNCING_FOR_AC_DISABLE:
      result = "BOUNC_FOR_DISABL";
      break;
    case AC_DISABLED:
      result = "AC_DISABLED";
      break;
    case BOUNCING_FOR_AC_ENABLE:
      result = "BOUN_FO_AC_ENABL";
      break;
    case COOL_TEMPERATURE_SENSOR_FAULT:
      result = "!SENSOR_FAULT!";
      break;
  }
  return result;
}
const char* heater_state_lcd() {

  switch (heater_state) {
    case HEATING_DISABLE:
      return "HEATER_OFF";
      break;
    case BOUNCING_FOR_HEATING_ENABLE:
      return "BONC_FOR_HEAT_ON";
      break;
    case HEATING_ENABLE:
      return "HEATER_ON";
      break;
    case BOUNCING_FOR_AC_OFF:
      return "BONC_FOR_HEAT_OF";
      break;
    case BOUNCING_FOR_HEATER_OFF:
      return "BOUNC_FOR_DISABL";
      break;
    case HEATER_OFF:
      return "HEATER_DISABLED";
      break;
    case BOUNCING_FOR_HEATER_ON:
      return "BOUN_FOR_ENABLE";
      break;
    case HEATER_TEMPERATURE_SENSOR_FAULT:
      return "!SENSOR_FAULT!";
      break;
  }
}





float Heater_Dip_Switch_Offset() {
  int a_button = !digitalRead(dip_switch_a);  // 0.5
  int b_button = !digitalRead(dip_switch_b);  // 1.0
  int c_button = !digitalRead(dip_switch_c);  // 2.0
  int minus_button = !digitalRead(dip_switch_d);
  float result = (a_button * 0.5 + b_button * 1 + c_button * 2);
  if (minus_button) {
    result = result * -1;
  }
  return result;
}

void humidifier_on() {
  digitalWrite(HUMIDIFIER_OFF_PIN, HIGH);
}

void humidifier_off() {
  digitalWrite(HUMIDIFIER_OFF_PIN, LOW);
}

const char* repeat_action_remaining_time_string_get() {
  static char timeBuffer[10];
  int repeatActionRemainingTimeInSeconds = (REPEAT_ACTION_INTERVAL - (millis() - repeatActionOldTime)) / 1000;
  int minutes = repeatActionRemainingTimeInSeconds / 60;
  int seconds = repeatActionRemainingTimeInSeconds % 60;
  // Format the time as "mm:ss"
  snprintf(timeBuffer, sizeof(timeBuffer), "%d:%02d", minutes, seconds);

  return timeBuffer;  // Return a pointer to the local static buffer
}
void setup() {


  Serial.begin(115200);
  while (!Serial)
    ;  // Wait for Serial to become available. Is optimized away for some cores.


  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  IrSender.begin();


  hdc1080.begin(0x40);

  delay(300);

  pinMode(dip_switch_d, INPUT_PULLUP);
  pinMode(dip_switch_c, INPUT_PULLUP);
  pinMode(dip_switch_b, INPUT_PULLUP);
  pinMode(dip_switch_a, INPUT_PULLUP);
  pinMode(HEATER_MODE, INPUT_PULLUP);
  pinMode(ac_disabled_pin, INPUT_PULLUP);
  pinMode(HUMIDIFIER_OFF_PIN, OUTPUT);

  delay(100);

  lcd.init();
  lcd.print("     WELCOME    ");
  // Serial.println("     WELCOME    ");
  // Serial.println(!digitalRead(ac_disabled_pin));
  HEATER_MODE_SELECTOR = digitalRead(HEATER_MODE);
  lcd.setCursor(0, 1);
  if (digitalRead(ac_disabled_pin)) {
    lcd.print("indoor is off");
    // Serial.println("     bisable    ");
    ac_state = AC_DISABLED;
    heater_state = HEATER_OFF;
  } else {
    lcd.print("indoor is on");
    // Serial.println("enable");
  }
  lcd.setCursor(0, 2);

  if (HEATER_MODE_SELECTOR) {
    lcd.print("ac = Heater");
    heating_disable();
    // Serial.println("Heater");
  } else {
    lcd.print("ac = Cooler");

    Serial.println("ac_cooling_disable");

    ac_cooling_disable();
    Serial.println("Cooler");
  }

  delay(100);
  lcd.clear();
  humidifier_off();
}

void heater_processing(float min_temp, float max_temp, float room_temp, bool reading_error, bool ac_on_button_on_box)

{
  if (reading_error) {
    if (heater_state != HEATER_OFF) {
      ac_state = HEATER_TEMPERATURE_SENSOR_FAULT;
    }
  }
  switch (heater_state) {
    case HEATING_DISABLE:
      //Serial.println("HEATING_DISABLE");
      if (!ac_on_button_on_box) {

        heater_state = BOUNCING_FOR_HEATER_OFF;
      } else {
        if (room_temp < min_temp) {

          heater_state = BOUNCING_FOR_HEATING_ENABLE;
          bounce_count = 0;
        } else {
          if (millis() - repeatActionOldTime >= REPEAT_ACTION_INTERVAL) {
            heating_disable();
            repeatActionOldTime = millis();
          }
        }
      }
      break;
    case BOUNCING_FOR_HEATING_ENABLE:
      //Serial.println("BOUNCING_FOR_HEATING_ENABLE");

      if (!ac_on_button_on_box) {

        heater_state = BOUNCING_FOR_HEATER_OFF;
      } else {
        if (room_temp < min_temp) {
          bounce_count++;


          if (bounce_count > MAX_BOUNCE_COUNT) {
            heater_state = HEATING_ENABLE;
            heating_enable();
            repeatActionOldTime = millis();
          }
        } else {
          bounce_count = 0;
          heater_state = HEATING_DISABLE;
        }
      }
      break;
    case HEATING_ENABLE:
      if (!ac_on_button_on_box) {
        heater_state = BOUNCING_FOR_HEATER_OFF;
      } else {

        if (room_temp > max_temp) {

          heater_state = BOUNCING_FOR_HEATING_DISABLE;
          bounce_count = 0;
        } else {

          if (millis() - repeatActionOldTime >= REPEAT_ACTION_INTERVAL) {
            heating_enable();
            repeatActionOldTime = millis();
          }
        }
      }
      break;
    case BOUNCING_FOR_HEATING_DISABLE:
      //Serial.println("BOUNCING_FOR_HEATING_DISABLE");

      if (!ac_on_button_on_box) {

        heater_state = BOUNCING_FOR_HEATER_OFF;
      } else {

        if (room_temp > max_temp) {
          bounce_count++;
          if (bounce_count > MAX_BOUNCE_COUNT) {
            heater_state = HEATING_DISABLE;
            repeatActionOldTime = millis();
            heating_disable();
          }
        } else {
          bounce_count = 0;
          heater_state = HEATING_ENABLE;
        }
      }
      break;

      ///////////////////////////////

    case BOUNCING_FOR_HEATER_OFF:
      bounce_count++;

      if (!ac_on_button_on_box) {

        if (bounce_count > BUTTON_BOUNCE_COUNT) {
          heating_disable();
          ac_power_toggle();
          bounce_count = 0;
          heater_state = HEATER_OFF;
        }
      } else {
        bounce_count = 0;
        heater_state = HEATING_DISABLE;
      }
      break;

    case HEATER_OFF:
      if (ac_on_button_on_box) {
        heater_state = BOUNCING_FOR_HEATER_ON;
        bounce_count = 0;
      }
      break;
      //////////////////////////

    case BOUNCING_FOR_HEATER_ON:
      bounce_count++;
      if (ac_on_button_on_box) {
        if (bounce_count > BUTTON_BOUNCE_COUNT) {

          ac_power_toggle();
          heater_state = HEATING_DISABLE;
        }
      } else {
        bounce_count = 0;
        heater_state = HEATER_OFF;
      }
      break;

    case HEATER_TEMPERATURE_SENSOR_FAULT:
      heater_on_sensor_fault();
      if (!reading_error) {
        heater_state = HEATING_DISABLE;
      }
      break;
    case COOL_MODE:
      break;
  }
}

void ac_processing(float min_hidx, float max_hidx, float room_temp, float room_humd, float room_hidx, bool reading_error, bool ac_on_button_on_box) {

  if (reading_error) {
    if (ac_state != AC_DISABLED && ac_state != BOUNCING_FOR_AC_DISABLE) {
      ac_state = COOL_TEMPERATURE_SENSOR_FAULT;
    }
  }

  switch (ac_state) {
    case AC_OFF:

      if (!ac_on_button_on_box) {
        ac_state = BOUNCING_FOR_AC_DISABLE;
      } else {
        if (room_hidx > max_hidx && room_humd > ABOVE_ZERO_LEVEL_HUMD) {
          ac_state = BOUNCING_FOR_AC_ON;
          bounce_count = 0;
        } else {
          ac_state = AC_OFF;
          if (millis() - repeatActionOldTime >= REPEAT_ACTION_INTERVAL) {
            ac_cooling_disable();
            repeatActionOldTime = millis();
          }
        }
      }
      break;
    case BOUNCING_FOR_AC_ON:

      if (ac_on_button_on_box == false) {
        ac_state = BOUNCING_FOR_AC_DISABLE;
      } else {

        if (room_hidx > max_hidx && room_humd > ABOVE_ZERO_LEVEL_HUMD) {
          bounce_count++;
          if (bounce_count > MAX_BOUNCE_COUNT) {
            ac_state = AC_ON;
            ac_cooling_enable();
            repeatActionOldTime = millis();
          }
        } else {
          bounce_count = 0;
          ac_state = AC_OFF;
        }
      }
      break;
    case AC_ON:
      if (!ac_on_button_on_box) {
        ac_state = BOUNCING_FOR_AC_DISABLE;
      } else {

        if (room_hidx < min_hidx || room_humd < ZERO_LEVEL_HUMD) {
          ac_state = BOUNCING_FOR_AC_OFF;
          bounce_count = 0;
        } else {
          ac_state = AC_ON;
          if (millis() - repeatActionOldTime >= REPEAT_ACTION_INTERVAL) {
            ac_cooling_enable();
            repeatActionOldTime = millis();
          }
        }
      }
      break;
    case BOUNCING_FOR_AC_OFF:

      if (!ac_on_button_on_box) {
        ac_state = BOUNCING_FOR_AC_DISABLE;
      } else {

        if (room_hidx < min_hidx || room_humd < ZERO_LEVEL_HUMD) {
          bounce_count++;
          if (bounce_count > MAX_BOUNCE_COUNT) {
            ac_state = AC_OFF;
            repeatActionOldTime = millis();
            ac_cooling_disable();
          }
        } else {
          bounce_count = 0;
          ac_state = AC_ON;
        }
      }
      break;
    case BOUNCING_FOR_AC_DISABLE:
      bounce_count++;
      if (!ac_on_button_on_box) {
        if (bounce_count > BUTTON_BOUNCE_COUNT) {
          ac_power_toggle();
          ac_state = AC_DISABLED;
        }
      } else {
        bounce_count = 0;
        ac_state = AC_OFF;
      }
      break;
    case AC_DISABLED:
      if (ac_on_button_on_box) {
        ac_state = BOUNCING_FOR_AC_ENABLE;
        bounce_count = 0;
      }
      break;
    case BOUNCING_FOR_AC_ENABLE:
      bounce_count++;
      if (ac_on_button_on_box) {
        if (bounce_count > BUTTON_BOUNCE_COUNT) {
          ac_cooling_disable();

          ac_state = AC_OFF;
        }
      } else {
        bounce_count = 0;
        ac_state = AC_DISABLED;
      }
      break;

    case COOL_TEMPERATURE_SENSOR_FAULT:
      ac_on_sensor_fault();
      if (!reading_error) {
        ac_state = AC_OFF;
      }
      break;
  }
}

void humidifier_processing(float room_humd, bool air_conditioner_button_is_on) {

  switch (humidifier_state) {
    case HUMIDIFIER_OFF:
      if (air_conditioner_button_is_on) {
        if (room_humd < MIN_HUMD) {
          humidifier_state = HUMIDIFIER_ON_BOUNCE;
        }
      }
      break;

    case HUMIDIFIER_ON_BOUNCE:
      if (air_conditioner_button_is_on) {
        if (room_humd < MIN_HUMD) {
          humidifier_state = HUMIDIFIER_ON;
          humidifier_on();
        } else
          humidifier_state = HUMIDIFIER_OFF;
      } else {
        humidifier_state = HUMIDIFIER_OFF_BOUNCE;
      }
      break;

    case HUMIDIFIER_ON:
      if (air_conditioner_button_is_on) {
        if (room_humd > MAX_HUMD) {
          humidifier_state = HUMIDIFIER_OFF_BOUNCE;
        }
      } else {
        humidifier_state = HUMIDIFIER_OFF_BOUNCE;
      }

      break;
    case HUMIDIFIER_OFF_BOUNCE:
      if (air_conditioner_button_is_on) {
        if (room_humd > MAX_HUMD) {
          humidifier_off();
          humidifier_state = HUMIDIFIER_OFF;
        }
      } else {
        humidifier_off();
        humidifier_state = HUMIDIFIER_OFF;
      }
      break;
  }
}

void loop() {
  static float room_humd = 0;
  static float room_temp = 0;
  static float room_hidx = 0;
  static float max_hidx = 0;
  static float min_hidx = 0;
  static float dip_switch_offset = 0;
  static bool reading_error = false;
  room_temp = hdc1080.readTemperature();  // Read Temperature
  room_humd = hdc1080.readHumidity();     // Read Humidity
  room_humd -= 15;
  if ((room_temp > 10 && room_temp < 60) && (room_humd > 20 && room_humd < 95)) {
    room_hidx = dht.computeHeatIndex(room_temp, room_humd, false);
    if (room_hidx >= 1 && room_hidx <= 95) {
      reading_error = false;
    } else {
      reading_error = true;
    }
  } else {
    reading_error = true;
  }

  bool ac_on_button = !digitalRead(ac_disabled_pin);
  float min_temp = 0;
  float max_temp = 0;
  float disier_hidx = 0;


  static unsigned char oldstate;
  unsigned char newstate = heater_state;



  if (HEATER_MODE_SELECTOR) {
    dip_switch_offset = Dip_Switch_Offset();
    min_temp = 19.8 + dip_switch_offset;
    max_temp = 20.2 + dip_switch_offset;
    heater_processing(min_temp, max_temp, room_temp, reading_error, ac_on_button);
    if (oldstate != newstate) {
      lcd.setCursor(0, 0);
      lcd.print("                   ");
      oldstate = heater_state;
    }
    lcd.setCursor(0, 0);
    lcd.print(heater_state_lcd());
    lcd.setCursor(16, 0);
    lcd.print("*");
    lcd.print((min_temp + .2), 1);
  } else {
    // Serial.print("HEATER_MODE_SELECTOR=");
    // Serial.println(HEATER_MODE_SELECTOR);
    dip_switch_offset = Dip_Switch_Offset();
    // dip_switch_offset = Heater_Dip_Switch_Offset();
    max_hidx = HC_MAX_HIDEX + dip_switch_offset;
    min_hidx = HC_MIN_HIDEX + dip_switch_offset;
    ac_processing(min_hidx, max_hidx, room_temp, room_humd, room_hidx, reading_error, ac_on_button);
    newstate = ac_state;
    if (oldstate != newstate) {
      lcd.setCursor(0, 0);
      lcd.print("                   ");
      oldstate = ac_state;
    }
    lcd.setCursor(0, 0);
    lcd.print(ac_state_lcd());
    disier_hidx = max_hidx - 0.2;
    lcd.setCursor(7, 2);
    lcd.print("*");
    lcd.print(disier_hidx, 1);
  }




  disier_hidx = max_hidx - 0.2;
  lcd.setCursor(11, 1);
  lcd.print("#");
  lcd.print(room_hidx, 1);

  humidifier_processing(room_humd, ac_on_button);


  if (ac_state == AC_ON || ac_state == AC_OFF) {
    lcd.setCursor(11, 0);
    lcd.print(repeat_action_remaining_time_string_get());
  }
  if (heater_state == HEATING_DISABLE || heater_state == HEATING_ENABLE) {
    lcd.setCursor(11, 0);
    lcd.print(repeat_action_remaining_time_string_get());
  }
  lcd.setCursor(0, 1);
  lcd.print("T");
  lcd.setCursor(1, 1);
  lcd.print(room_temp, 1);
  lcd.setCursor(6, 1);
  lcd.print("H");
  lcd.print(room_humd, 0);
  lcd.print("%");
  lcd.setCursor(0, 3);
  lcd.print("SOGO = ");
  lcd.setCursor(7, 3);
  lcd.print(digitalRead(HUMIDIFIER_OFF_PIN));
  delay(1000);
}


#define BUFFER_LENGHT 270  // this is the max length supported by the arduino library for the arduino used in the AC hardware

uint16_t pronto_buffer[BUFFER_LENGHT];  //232
uint16_t pronto_commands;
void sendProntoFromProgmem(uint16_t button_code[]) {

  pronto_commands = pgm_read_dword(&button_code[2]);

  uint16_t buffer_length = pronto_commands * 2 + 4;
  for (int k = 0; k < buffer_length; k++) {
    pronto_buffer[k] = pgm_read_dword(&button_code[k]);
  }

  irsend.sendPronto(pronto_buffer, buffer_length, 0);
  delay(6);
}


const PROGMEM uint16_t cool_25_off[] = { 0, 109, 100, 0, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 197, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 41 };

const PROGMEM uint16_t cool_25_on_a[] = { 0, 109, 100, 0, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 197, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 41 };

const PROGMEM uint16_t cool_25_on_b[] = { 0, 109, 50, 0, 168, 166, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 41 };

const PROGMEM uint16_t heat_22_off[] = { 0, 109, 100, 0, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 197, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 41 };

const PROGMEM uint16_t heat_22_on_a[] = { 0, 109, 100, 0, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 197, 168, 166, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 41 };

const PROGMEM uint16_t heat_22_on_b[] = { 0, 109, 50, 0, 168, 166, 21, 61, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 20, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 20, 21, 61, 21, 61, 21, 61, 21, 20, 21, 20, 21, 61, 21, 41 };






void ac_on_sensor_fault() {
  // sendProntoFromProgmem((uint16_t*)(cool_27_a));
  // sendProntoFromProgmem((uint16_t*)(cool_27_b));
}
void heater_on_sensor_fault() {

  // sendProntoFromProgmem((uint16_t*)(heat_17_a));
  // sendProntoFromProgmem((uint16_t*)(heat_17_b
}

void heating_enable() {
  sendProntoFromProgmem((uint16_t*)(heat_22_on_a));
  sendProntoFromProgmem((uint16_t*)(heat_22_on_b));

}
void heating_disable() {
  sendProntoFromProgmem((uint16_t*)(heat_22_off));
}
void ac_cooling_enable() {

  sendProntoFromProgmem((uint16_t*)(cool_25_on_a));
  sendProntoFromProgmem((uint16_t*)(cool_25_on_b));

  //Serial.println("COOLING_ENABLE buffer");
}
void ac_cooling_disable() {
  sendProntoFromProgmem((uint16_t*)(cool_25_off));
  Serial.println("COOLING_disable buffer***********");
}
void ac_power_toggle() {
  sendProntoFromProgmem((uint16_t*)(cool_25_off));
}
