#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "driver/twai.h"
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"

#include <Wire.h>
#include "INA226.h"

#include <EEPROM.h>
#include <Adafruit_NeoPixel.h>

INA226 INA(0x40);

// Adafruit_NeoPixel pixels = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);

char names[6][20] =
    {
        "CONFIGURATION: ",
        "        SHUNT: ",
        "  BUS VOLTAGE: ",
        "        POWER: ",
        "      CURRENT: ",
        "  CALIBRATION: "};

// any xbox controller
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("3c:fa:06:77:9b:e1"); // 0000 blue
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("28:ea:0b:d4:fd:8b");    // 0001 bLACK 09710694153347
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("40:8e:2c:01:55:81");    // 0002 blue 
XboxSeriesXControllerESP32_asukiaaa::Core xboxController("40:8e:2c:c0:22:0f");    // 0003 white 

int joy_connected = 0;
int button_pressed = 0;
int button2_pressed = 0;
int joyspeed = 100;
int joyerror = 100;
const int max_joyspeed = 200;
const int max_joysteer = 200;

bool send_0 = false; // Checks if when the joystick is not connected, desired speed 0 is send over the CAN-bus. This needs to be done once.

unsigned long previousMillis = 0; // Stores last time the sensor was updated
const long interval = 1000;
bool sensorInitialized = false;

// -- battery variables
// // Constants for voltage to SoC conversion || Lithion ion
const float FULL_VOLTAGE = 14.3;
const float EMPTY_VOLTAGE = 13.0;
const float BATTERY_CAPACITY_AH = 60.0; // Battery capacity in Ah

// Constants for voltage to SoC conversion || LEAD ACIC
// const float FULL_VOLTAGE = 13.1;
// const float EMPTY_VOLTAGE = 12.0;
// const float BATTERY_CAPACITY_AH = 7.0; //Battery capacity in Ah


// EEPROM address for storing SoC
const int SOC_ADDRESS = 4;
float voltage = 0;
float current = 0;
int battery_percentage = 0;
int soc;           // State of Charge in percentage (0-100)
float totalCharge; // Total charge in As (ampere-seconds)
unsigned long lastStoreMillis = 0;

// Docking station variables
int docking_pin                       = GPIO_NUM_3; // D0 = GPIO_NUM_2, D1 = GPIO_NUM_3
unsigned long previousMillis_docking  = 0;    // Stores last time the docking station was read
const long interval_docking           = 250;  // Interval in ms to read docking station

void storeSoC(int soc)
{
  EEPROM.write(SOC_ADDRESS, soc); // Store SoC as integer (0-100)
  EEPROM.commit();
  Serial.print("Write SoC to EEPROM");
}

int retrieveSoC()
{
  Serial.print("get initialfrom EEPROM");
  return EEPROM.read(SOC_ADDRESS); // Retrieve SoC as integer (0-100)
}

void setup()
{
  // pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  // pixels.setBrightness(0); // not so bright
  // pixels.setPixelColor(0,0,0,0);
  // pixels.show();
  Serial.begin(115200);
  delay(1500);

  // Setup docking pin
  pinMode(docking_pin, INPUT_PULLUP);

  if (!EEPROM.begin(512)) {
    Serial.println("Failed to initialize EEPROM");
    while (true);
}
  Serial.println("Starting NimBLE Client");
  xboxController.begin();

  // Wire.begin(GPIO_NUM_8, GPIO_NUM_9); // C3 DEV_KIT
  Wire.begin(GPIO_NUM_6, GPIO_NUM_7); // C3 MINI
  Serial.println("INA226 Test");

  // Initialize I2C with custom SDA and SCL pins if needed
  // Wire.begin(SDA_PIN, SCL_PIN);

  // Attempt to initialize the INA219 up to 5 times
  for (int attempts = 0; attempts < 5; attempts++)
  {
    if (INA.begin())
    {
      Serial.println("Found INA226 chip");
      sensorInitialized = true;            // Successfully initialized the sensor
      INA.setMaxCurrentShunt(40, 0.00194); // shunt on board = 0.0015 ohm
      break;                               // Exit the loop if initialization was successful
    }
    else
    {
      Serial.println("Couldn't find INA219 chip, retrying...");
      delay(500); // Wait a bit before retrying
    }
  }
  storeSoC(99); //Set inital percetage. only needed at fisrt start of robot when belew 100 voltage

  if (!sensorInitialized)
  {
    Serial.println("Failed to initialize INA219 after 5 attempts, continuing without sensor readings.");
  }

  voltage = INA.getBusVoltage();

  if (voltage >= FULL_VOLTAGE)
  {
    soc = 100;
    totalCharge = BATTERY_CAPACITY_AH * 3600; // Full charge in As (1 Ah = 3600 As)
  }
  else
  {
    
    soc = retrieveSoC();
    totalCharge = (soc / 100.0) * BATTERY_CAPACITY_AH * 3600; // Convert SoC back to total charge in As
  }

  Serial.print("Initial SoC: ");
  Serial.println(soc);
  Serial.print("Initial Charge (As): ");
  Serial.println(totalCharge);

  delay(1000);
  // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_7, GPIO_NUM_4, TWAI_MODE_NORMAL);
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_20, TWAI_MODE_NORMAL);

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

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

void loop()
{
  xboxController.onLoop();
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval && sensorInitialized == true)
  {
    previousMillis = millis();
    // Serial.println("\nVolt\tCURRENT");
    voltage = INA.getBusVoltage();
    current = INA.getCurrent_mA() * -1 / 1000;
    // Serial.print(INA.getBusVoltage(), 3);
    // Serial.print(voltage, 3);
    // Serial.print("\t");
    // Serial.print(current, 3);
    // Serial.print("\t");

    totalCharge += current;
    soc = (totalCharge / (BATTERY_CAPACITY_AH * 3600)) * 100.0;
    if (voltage >= FULL_VOLTAGE)
    {
      soc = 100;
      totalCharge = BATTERY_CAPACITY_AH * 3600; // Full charge in As (1 Ah = 3600 As)
    }

    // Serial.print("SoC: ");
    // Serial.println(soc);
    // Serial.print("Total Charge (As): ");
    // Serial.println(totalCharge);

    // Store SoC every 5 minutes
    if (millis() - lastStoreMillis >= 300000)
    { // 300000 ms = 5 minutes
      lastStoreMillis = millis();
      storeSoC(soc);
    }

    int scaledValue = (int)(voltage * 100);
    uint8_t highByte = (scaledValue >> 8) & 0xFF; // Extract high byte
    uint8_t lowByte = scaledValue & 0xFF;         // Extract low byte

    twai_message_t message_bat;
    message_bat.identifier = 0x0c9;
    message_bat.extd = 0;
    message_bat.data_length_code = 8;

    message_bat.data[0] = soc;
    message_bat.data[1] = (int)(voltage * 10);
    message_bat.data[2] = highByte;
    message_bat.data[3] = lowByte;
    message_bat.data[4] = (int)(current * 10);
    message_bat.data[5] = 0;
    message_bat.data[6] = 0;
    message_bat.data[7] = 0;

    if (twai_transmit(&message_bat, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
      send_0 = true;
    }
    else
    {
      Serial.print("CAN bus transit error");
    }
  }

  if (xboxController.isConnected())
  {
    uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
    joy_connected = 1;
    button_pressed = (xboxController.xboxNotif.btnRB);
    button2_pressed = (xboxController.xboxNotif.btnLB);
    if (button2_pressed == true)
    {
      joyspeed = map(xboxController.xboxNotif.joyLVert, 65535, 0, 0, max_joyspeed); // To foreward max on joystick = 0, To backward max on joystick = 65535
    }
    else
    {
      joyspeed = map(xboxController.xboxNotif.joyLVert, 65535, 0, 50, 150);
    }

    joyerror = map(xboxController.xboxNotif.joyRHori, 65535, 0, 0, max_joysteer); // To left = 0 , to right = 65535

    // Serial.print(button_pressed);
    //  Serial.print("\t");
    // Serial.print(button2_pressed);
    //  Serial.print("\t");
    // Serial.print(joyspeed);
    //  Serial.print("\t");
    // Serial.print(joyerror);
    //  Serial.print("\t");
    // Serial.print(xboxController.xboxNotif.joyLHori);
    // Serial.println();
  }

  if (!xboxController.isConnected())
  {
    // Serial.println("not connected");
    joy_connected = 0;
    button_pressed = 0;
    joyspeed = 100;
    joyerror = 100;

    if (send_0 == false)
    { // Send desired speed = 0 to CAN-bus once, when joystick is disconnected
      twai_message_t message;
      message.identifier = 0x0c8;
      message.extd = 0;
      message.data_length_code = 8;

      message.data[0] = joy_connected;
      message.data[1] = button_pressed;
      message.data[2] = joyspeed;
      message.data[3] = joyerror;
      message.data[4] = 0;
      message.data[5] = 0;
      message.data[6] = 0;
      message.data[7] = 0;

      // Queue message for transmission
      if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
      {
        send_0 = true;
      }
      else
      {
        Serial.print("CAN bus transit error");
      }
    }
  }

  if (joy_connected == 1)
  {
    twai_message_t message;
    message.identifier = 0x0c8;
    message.extd = 0;
    message.data_length_code = 8;

    message.data[0] = joy_connected;
    message.data[1] = button_pressed;
    message.data[2] = joyspeed;
    message.data[3] = joyerror;
    message.data[4] = 0;
    message.data[5] = 0;
    message.data[6] = 0;
    message.data[7] = 0;

    // Queue message for transmission
    if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK)
    {
      send_0 = true;
    }
    else
    {
      Serial.print("CAN bus transit error");
    }

    send_0 = false;
  }

  unsigned long currentMillis_docking = millis();

  if (currentMillis_docking - previousMillis_docking >= interval_docking ){
    previousMillis_docking = millis(); // Update previous interval time

    int input_pin_docking = digitalRead(docking_pin);
    Serial.print(input_pin_docking);

    twai_message_t message_docking;
    message_docking.identifier = 0x0ca;
    message_docking.extd = 0;
    message_docking.data_length_code = 8;

    message_docking.data[0] = input_pin_docking;
    message_docking.data[1] = 0;
    message_docking.data[2] = 0;
    message_docking.data[3] = 0;
    message_docking.data[4] = 0;
    message_docking.data[5] = 0;
    message_docking.data[6] = 0;
    message_docking.data[7] = 0;

    if (twai_transmit(&message_docking, pdMS_TO_TICKS(1000)) != ESP_OK){
      Serial.print("CAN bus transit error");
    }

  }


  if (xboxController.getCountFailedConnection() > 2)
  {
    ESP.restart();
  }

  delay(50);
}

