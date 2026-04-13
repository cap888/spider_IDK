/**
 * Spider Robot - Transmitter (Пульт управления)
 * 
 * Оборудование:
 * - Arduino Nano
 * - Джойстик KY-023 (X, Y, кнопка)
 * - Bluetooth HC-05 (Master mode)
 * 
 * Протокол передачи:
 * [START 0xFF][JOY_X 0-255][JOY_Y 0-255][BUTTONS][CHECKSUM][END 0xFE]
 */

#include <SoftwareSerial.h>

// Пины джойстика
const int PIN_JOY_X = A0;
const int PIN_JOY_Y = A1;
const int PIN_JOY_BTN = 2;

// Bluetooth HC-05 (SoftwareSerial)
const int BT_RX = 3;
const int BT_TX = 4;
SoftwareSerial bluetooth(BT_RX, BT_TX);

// Параметры джойстика
const int JOY_DEADZONE = 15;  // Мёртвая зона
const int JOY_CENTER = 512;   // Центральная позиция (10-bit ADC)

// Протокол
const byte PACKET_START = 0xFF;
const byte PACKET_END = 0xFE;

// Состояние кнопок
struct ButtonState {
  bool joy_btn : 1;
  bool reserved1 : 1;
  bool reserved2 : 1;
  bool reserved3 : 1;
  bool reserved4 : 1;
  bool reserved5 : 1;
  bool reserved6 : 1;
  bool reserved7 : 1;
};

void setup() {
  Serial.begin(9600);
  bluetooth.begin(38400);  // Скорость HC-05 по умолчанию
  
  pinMode(PIN_JOY_BTN, INPUT_PULLUP);
  
  // Задержка для стабилизации HC-05
  delay(1000);
  
  Serial.println("Spider Robot Transmitter initialized");
}

void loop() {
  // Чтение джойстика
  int joy_x_raw = analogRead(PIN_JOY_X);
  int joy_y_raw = analogRead(PIN_JOY_Y);
  bool joy_btn = digitalRead(PIN_JOY_BTN) == LOW;
  
  // Преобразование в диапазон 0-255
  byte joy_x = mapToByte(joy_x_raw);
  byte joy_y = mapToByte(joy_y_raw);
  
  // Применение мёртвой зоны
  joy_x = applyDeadzone(joy_x, 128, JOY_DEADZONE);
  joy_y = applyDeadzone(joy_y, 128, JOY_DEADZONE);
  
  // Формирование байта кнопок
  ButtonState buttons;
  buttons.joy_btn = joy_btn;
  buttons.reserved1 = 0;
  buttons.reserved2 = 0;
  buttons.reserved3 = 0;
  buttons.reserved4 = 0;
  buttons.reserved5 = 0;
  buttons.reserved6 = 0;
  buttons.reserved7 = 0;
  
  byte buttons_byte = *(byte*)&buttons;
  
  // Расчёт контрольной суммы
  byte checksum = (PACKET_START + joy_x + joy_y + buttons_byte) & 0xFF;
  
  // Отправка пакета
  bluetooth.write(PACKET_START);
  bluetooth.write(joy_x);
  bluetooth.write(joy_y);
  bluetooth.write(buttons_byte);
  bluetooth.write(checksum);
  bluetooth.write(PACKET_END);
  
  // Отладка
  #ifdef DEBUG
  Serial.print("Joy X: ");
  Serial.print(joy_x);
  Serial.print(" | Joy Y: ");
  Serial.print(joy_y);
  Serial.print(" | BTN: ");
  Serial.println(joy_btn);
  #endif
  
  // Задержка для стабильной передачи (~50Hz)
  delay(20);
}

/**
 * Преобразование 10-bit ADC (0-1023) в byte (0-255)
 */
byte mapToByte(int value) {
  return constrain(value / 4, 0, 255);
}

/**
 * Применение мёртвой зоны
 */
byte applyDeadzone(byte value, byte center, byte deadzone) {
  int diff = value - center;
  
  if (abs(diff) <= deadzone) {
    return center;
  }
  
  // Масштабирование за пределами мёртвой зоны
  if (diff > 0) {
    return center + map(diff, deadzone + 1, 255 - center, 1, 255 - center);
  } else {
    return center - map(-diff, deadzone + 1, center, 1, center);
  }
}
