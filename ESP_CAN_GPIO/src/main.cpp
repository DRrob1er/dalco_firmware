#include <Arduino.h>
#include "driver/twai.h"

const int pin_CAN_TX    = GPIO_NUM_7;
const int pin_CAN_RX    = GPIO_NUM_4;

const int pin_drawer_1  = GPIO_NUM_0;   // (J0) Top drawer
const int pin_drawer_2  = GPIO_NUM_1;   // (J1)
const int pin_drawer_3  = GPIO_NUM_3;   // (J2)
const int pin_drawer_4  = GPIO_NUM_5;   // (J3)
const int pin_drawer_5  = GPIO_NUM_6;   // (J4)
const int pin_drawer_6  = GPIO_NUM_10;  // (J5)
const int pin_IO_7      = GPIO_NUM_18;  // (J6)
const int pin_IO_8      = GPIO_NUM_19;  // (J7)


//function declarations:
void setup_pins();

void setup() {
  Serial.begin(115200);
  delay(1000);
  setup_pins();

  // Configure CAN_BUS
  twai_general_config_t g_config  = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_7 , GPIO_NUM_4 , TWAI_MODE_NORMAL);
  twai_timing_config_t t_config   = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config   = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK)
  {
    printf("Driver installed\n");
  }
  else
  {
    printf("Failed to install driver\n");
    return;
  }
  // Start TWAI driver
  if (twai_start() == ESP_OK)
  {
    printf("Driver started\n");
  }
  else
  {
    printf("Failed to start driver\n");
    return;
  }
}

void loop() {
  uint8_t drawer_state = 0;

  // Read each pin and store the value in the correct bit position
  drawer_state |= digitalRead(pin_drawer_1) << 0;
  drawer_state |= digitalRead(pin_drawer_2) << 1;
  drawer_state |= digitalRead(pin_drawer_3) << 2;
  drawer_state |= digitalRead(pin_drawer_4) << 3;
  drawer_state |= digitalRead(pin_drawer_5) << 4;
  drawer_state |= digitalRead(pin_drawer_6) << 5;

  // Two MSB (bits 6 and 7) remain zero
  Serial.print("Binary: ");
  for (int i = 7; i >= 0; i--) {
      Serial.print((drawer_state >> i) & 1);
  }
  Serial.print(" | Hex: 0x");
  Serial.println(drawer_state, HEX);


  // Send CAN
  twai_message_t message_gpio;
  message_gpio.identifier = 0x0cb;
  message_gpio.extd = 0;
  message_gpio.data_length_code = 8;

  message_gpio.data[0] = drawer_state;
  message_gpio.data[1] = 0;
  message_gpio.data[2] = 0;
  message_gpio.data[3] = 0;
  message_gpio.data[4] = 0;
  message_gpio.data[5] = 0;
  message_gpio.data[6] = 0;
  message_gpio.data[7] = 0;

  if (twai_transmit(&message_gpio, pdMS_TO_TICKS(1000)) != ESP_OK){
    Serial.print("CAN bus transit error");
  }

  delay(500); // Wait 0.5 sec
}

//function definitions:
void setup_pins() {
  pinMode(pin_drawer_1, INPUT_PULLUP);
  pinMode(pin_drawer_2, INPUT_PULLUP);
  pinMode(pin_drawer_3, INPUT_PULLUP);
  pinMode(pin_drawer_4, INPUT_PULLUP);
  pinMode(pin_drawer_5, INPUT_PULLUP);
  pinMode(pin_drawer_6, INPUT_PULLUP);
  Serial.print("Setup pins done");
}