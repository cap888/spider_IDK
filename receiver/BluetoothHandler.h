/**
 * Spider Robot - Bluetooth Handler
 * 
 * Приём и обработка данных от джойстика через Bluetooth HC-05
 * Протокол: [START 0xFF][JOY_X][JOY_Y][BUTTONS][CHECKSUM][END 0xFE]
 */

#ifndef BLUETOOTH_HANDLER_H
#define BLUETOOTH_HANDLER_H

#include "Config.h"

class BluetoothHandler {
private:
  ControlState control_state;
  
  // Буфер приёма
  byte rx_buffer[6];
  int rx_index;
  bool receiving;
  
  /**
   * Проверка контрольной суммы
   */
  bool verifyChecksum(const JoystickPacket& packet) {
    byte calculated = (packet.start_byte + packet.joy_x + packet.joy_y + packet.buttons) & 0xFF;
    return calculated == packet.checksum;
  }
  
  /**
   * Преобразование значения джойстика (0-255) в скорость (-1.0 до 1.0)
   */
  float joystickToSpeed(byte value) {
    // Применение мёртвой зоны
    if (abs(value - 128) <= JOYSTICK_DEADZONE) {
      return 0.0;
    }
    
    // Масштабирование
    return (value - 128) / 127.0;
  }

public:
  BluetoothHandler() : rx_index(0), receiving(false) {
    control_state.speed_x = 0.0;
    control_state.speed_y = 0.0;
    control_state.button_pressed = false;
    control_state.connected = false;
    control_state.last_update = 0;
  }
  
  /**
   * Инициализация Bluetooth
   */
  void begin() {
    BT_SERIAL.begin(BT_BAUDRATE);
    #ifdef DEBUG_BLUETOOTH
    Serial.println("Bluetooth initialized");
    #endif
  }
  
  /**
   * Обновление данных (вызывать в loop)
   */
  void update() {
    // Чтение доступных байтов
    while (BT_SERIAL.available() > 0) {
      byte b = BT_SERIAL.read();
      
      if (!receiving) {
        // Ожидание стартового байта
        if (b == 0xFF) {
          receiving = true;
          rx_buffer[0] = b;
          rx_index = 1;
        }
      } else {
        // Приём данных
        if (rx_index < 6) {
          rx_buffer[rx_index++] = b;
          
          // Пакет полностью принят
          if (rx_index == 6) {
            receiving = false;
            processPacket();
            rx_index = 0;
          }
        } else {
          // Ошибка - сброс
          receiving = false;
          rx_index = 0;
        }
      }
    }
    
    // Проверка таймаута
    if (millis() - control_state.last_update > BLUETOOTH_TIMEOUT) {
      control_state.connected = false;
      control_state.speed_x = 0.0;
      control_state.speed_y = 0.0;
    }
  }
  
  /**
   * Обработка принятого пакета
   */
  void processPacket() {
    JoystickPacket* packet = (JoystickPacket*)rx_buffer;
    
    // Проверка стартового и конечного байтов
    if (packet->start_byte != 0xFF || packet->end_byte != 0xFE) {
      #ifdef DEBUG_BLUETOOTH
      Serial.println("BT: Invalid packet markers");
      #endif
      return;
    }
    
    // Проверка контрольной суммы
    if (!verifyChecksum(*packet)) {
      #ifdef DEBUG_BLUETOOTH
      Serial.println("BT: Checksum error");
      #endif
      return;
    }
    
    // Преобразование в скорость
    control_state.speed_x = joystickToSpeed(packet->joy_x);
    control_state.speed_y = joystickToSpeed(packet->joy_y);
    control_state.button_pressed = (packet->buttons & 0x01) == 0x01;
    control_state.connected = true;
    control_state.last_update = millis();
    
    #ifdef DEBUG_BLUETOOTH
    Serial.print("BT: X=");
    Serial.print(control_state.speed_x, 2);
    Serial.print(" Y=");
    Serial.print(control_state.speed_y, 2);
    Serial.print(" BTN=");
    Serial.println(control_state.button_pressed);
    #endif
  }
  
  /**
   * Получение состояния управления
   */
  const ControlState& getState() const {
    return control_state;
  }
  
  /**
   * Проверка подключения
   */
  bool isConnected() const {
    return control_state.connected;
  }
  
  /**
   * Получение скорости по X (вперёд/назад)
   */
  float getSpeedX() const {
    return control_state.speed_x;
  }
  
  /**
   * Получение скорости по Y (влево/вправо)
   */
  float getSpeedY() const {
    return control_state.speed_y;
  }
  
  /**
   * Получение состояния кнопки
   */
  bool isButtonPressed() const {
    return control_state.button_pressed;
  }
};

#endif // BLUETOOTH_HANDLER_H
