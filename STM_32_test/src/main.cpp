// // https://techoverflow.net/2021/09/22/how-to-fix-platformio-stm32-error-libusb_open-failed-with-libusb_error_access/
// // sudo usermod -a -G tty daleur
// // sudo apt -y install stlink-tools

// Cheatsheet CANopen:

//  IDX     What                                  Type          Down
//  6060 =  operation motor                       [int_8]       0x2f    02 is speed control 01 is target postion
//  6042 =  target velocity                       [int_16]      0x2b    
//  607A =  Target postion                        [int_32]      0x23
//  6040 =  controlword                           [unsigned16]  0x2b
//  6081 =  PDO target velocity                   [int_32]      0x23
//  607a =  PDO target postition                  [int_32]      0x23

//  - Download Byte - 
//  1 byte = 0x2f
//  2 byte = 0x2b
//  3 byte = 0x27
//  4 byte = 0x23

//  - Control word hex    What -
//    0x0000              Transition to "Ready to Switch On"

#include <eXoCAN.h>
#include <functions.h>


// Init CANbus
eXoCAN myCAN(STD_ID_LEN, BR1M, PORTB_8_9_XCVR);
uint8_t       CAN_txData[8];

struct CAN_frame_t {
    int id;      
    uint8_t data[8];  
    uint8_t len;
};

// Recieve CANopen messages data + flag
uint8_t  data_80_1[8], data_1001_1,    data_603F_1[2], data_6041_1[2], data_6061_1,    data_6064_1[4], data_6044_1[2];
bool     bool_80_1,    bool_1001_1,    bool_603F_1,    bool_6041_1,    bool_6061_1,    bool_6064_1,    bool_6044_1;

uint8_t  data_80_2[8], data_1001_2,    data_603F_2[2], data_6041_2[2], data_6061_2,    data_6064_2[4], data_6044_2[2];
bool     bool_80_2,    bool_1001_2,    bool_603F_2,    bool_6041_2,    bool_6061_2,    bool_6064_2,    bool_6044_2;

uint8_t  data_80_3[8], data_1001_3,    data_603F_3[2], data_6041_3[2], data_6061_3,    data_6064_3[4], data_6044_3[2];
bool     bool_80_3,    bool_1001_3,    bool_603F_3,    bool_6041_3,    bool_6061_3,    bool_6064_3,    bool_6044_3;

uint8_t can_joy_data[8] ,can_nuc_data[8];

int         id_unkown_can;
uint8_t     unknown_can[8];
bool        bool_unknown_can;
bool        can_stop = false;

// --- Pins ----
// Pins
const int pin_tipover_1   = PB3;
const int pin_tipover_2   = PB4;
const int pin_em_stop     = PB5;
const int pin_bumper_1    = PB12;
const int pin_bumper_2    = PB13;
const int pin_brake_relay = PB14;
const int pin_led         = PB15;
//Extra pins
const int pin_spi_clk     = PA5;
const int pin_spi_miso    = PA6;
const int pin_spi_mosi    = PA7;
const int pin_scl         = PB6;
const int pin_sda         = PB7;

// canbus variabels
bool can_bus_timeout_joy      = false;
const int  can_steer_offset   = 120;     // A extra offset is used to. in this case + 120. This should be used with the reader subtracted 
const int  can_speed_offset   = 2000; 
const int  can_rx_timeout     = 1000;    // Timeout in millis
bool can_bus_timeout_nuc      = false;
unsigned long can_oldtime_nuc;
unsigned long can_oldtime_joy;

// NUC 
int nuc_status              = 0;
int nuc_error_reset         = 1;
int nuc_steer_desired       = 0;    // Desired steer angle NUC
int nuc_speed_desired       = 0;    // Desired speed NUC

// drive_train variables
int   wheelbase                     = 478;  //[mm]
// int   wheeltrack                    = 323;  //[mm] Normal Dalco Drive
int   wheeltrack                    = 478;  //[mm] EMC wider platform

double  max_yaw_rate                 = 1.00; // [rad /s]
int     steer_pulses_per_rot         = 3600;
double  steer_transmission_ratio     = 4.0;
int     steer_pulses_homing_offset   = 4025; // 8050 / 2
int     steer_max_angle              = 60;  // Max steeragnel +/-   
int     steer_desired                = 0;    // Desired steer angle
double  current_steer_angle          = 0;
int     speed_max                    = 1500;   // Max speed
int     speed_desired                = 0;      // Desired speed
int     wheel_speed_right            = 0;
int     wheel_speed_left             = 0;
int     bumper_reactivated_delay     = 5000; // ms
int     prev_motor_position_pulse    = 0;
int     prev_wheel_speed_right       = 0;
int     prev_wheel_speed_left        = 0;
int     delta_pos_move_treshold      = 5; //the number of position pulses per iteration to consider the robto moving

// Joystick const int       
const int       joystick_max_speed    = 1000; // Max mm/s
const int       joystick_max_steer    = 88;   // Max steering degrees
int             joystick_speed        = 0;
int             joystick_steer        = 0;
int             joystick_active       = 0;

// CAN open variables
const uint8_t CS_START_REMOTE_NODE  = 0x01;
const uint8_t CS_STOP_REMOTE_NODE   = 0x02;

int operation_mode_1  = -1;
int operation_mode_2  = -1;
int operation_mode_3  = -1;

int32_t actual_position_1       = 0;
int32_t actual_position_2       = 0;
int32_t actual_position_3       = 0;
int32_t prev_actual_position_2  = 0;
int32_t prev_actual_position_3  = 0;

int16_t actual_velocity_1 = 0;
int16_t actual_velocity_2 = 0;
int16_t actual_velocity_3 = 0;

uint16_t error_code_1 = 0;
uint16_t error_code_2 = 0;
uint16_t error_code_3 = 0;

// State
int state_robot           = -1; // -1 = initial state, 0 undefined state , 1 = Nuc controll,   2 = Joystick controll, 3 = idle with braking 4 = idle no bracking, 5 = timebout both nuc and joy, 6 bumper pressed ,7 = emstop pressed.
int prev_state_robot      = 0;
int robot_error           = 0;  // 0= no error, 1, em_stop, 2 = bumper hit, 3 = motorcontroller, 4 = homing failed, 
int state_mot_cont_1      = 0;
int state_mot_cont_2      = 0;
int state_mot_cont_3      = 0;
int state_homing_1        = 0;
int state_em_stop         = 0; // 0 = no emstop 1 = em stop pressed
int brake_relay_state     = 0;

// timers
unsigned long bumper_timer = 0;

// LED error

unsigned long previousMillis  = 0;
const long blinkInterval      = 200; // Interval for blinking in milliseconds (half second)
const long offInterval        = 1000;  // Interval for the LED to stay off after blinking
int ledState                  = LOW;             // Current state of the LED
int blinkCount                = 0;
int currentBlinkCount         = 0;
bool offPeriod                = false;


void canISR() // get CAN bus frame passed by a filter into fifo0
{
  int               id, fltIdx; 
  volatile uint8_t  rxbytes[8]; // init 
  myCAN.receive(id, fltIdx, rxbytes);  // empties fifo0 so that another another rx interrupt can take place
  
  // Copy messages of intresest
  if (id == 0x081)
  { // 081 error message. 8 bytes
      std::copy(rxbytes, rxbytes + 8, data_80_1);
      bool_80_1 = true;
  }
  else if (id == 0x082)
  { // 082 error message. 8 bytes
      std::copy(rxbytes, rxbytes + 8, data_80_2);
      bool_80_2 = true;
  }
  else if (id == 0x083)
  { // 083 error message. 8 bytes
      std::copy(rxbytes, rxbytes + 8, data_80_3);
      bool_80_3 = true;
  }
  else if(id == 0x581 && rxbytes[1] == 0x01 && rxbytes[2] == 0x10)
    { // 1001 status message after sending SDO request
    data_1001_1 = rxbytes[4];
    bool_1001_1 = true;
  } 
  else if(id == 0x582 && rxbytes[1] == 0x01 && rxbytes[2] == 0x10)
    { // 1001 status message after sending SDO request
    data_1001_2 = rxbytes[4];
    bool_1001_2 = true;
  } 
  else if(id == 0x583 && rxbytes[1] == 0x01 && rxbytes[2] == 0x10)
    { // 1001 status message after sending SDO request
    data_1001_3 = rxbytes[4];
    bool_1001_3 = true;
  } 
  else if(id == 0x581 && rxbytes[1] == 0x41 && rxbytes[2] == 0x60)  //(ask for statusword)
    { // 1001 status message after sending SDO request
      data_6041_1[0] = rxbytes[4];
      data_6041_1[1] = rxbytes[5];
      bool_6061_1 = true;

    // Serial1.println("got statusword message");
  } 
  else if(id == 0x582 && rxbytes[1] == 0x41 && rxbytes[2] == 0x60)  //(ask for statusword)
    { // 1001 status message after sending SDO request
      data_6041_2[0] = rxbytes[4];
      data_6041_2[1] = rxbytes[5];
      bool_6061_2 = true;

    // Serial1.println("got statusword message");
  } 
  else if(id == 0x583 && rxbytes[1] == 0x41 && rxbytes[2] == 0x60)  //(ask for statusword)
    { // 1001 status message after sending SDO request
      data_6041_3[0] = rxbytes[4];
      data_6041_3[1] = rxbytes[5];
      bool_6061_3 = true;

    // Serial1.println("got statusword message");
  } 
    else if(id == 0x581 && rxbytes[1] == 0x3F && rxbytes[2] == 0x60)  //(ask for errors)
    { // 603F error codes message after sending SDO request
      data_603F_1[0] = rxbytes[4];
      data_603F_1[1] = rxbytes[5];
      bool_603F_1 = true;

    // Serial1.println("got statusword message");
  } 
  else if(id == 0x582 && rxbytes[1] == 0x3F && rxbytes[2] == 0x60)  //(ask for errors)
    { // 603F error codes message after sending SDO request
      data_603F_2[0] = rxbytes[4];
      data_603F_2[1] = rxbytes[5];
      bool_603F_2 = true;

    // Serial1.println("got statusword message");
  } 
  else if(id == 0x583 && rxbytes[1] == 0x3F && rxbytes[2] == 0x60)  //(ask for errors)
    { // 603F error codes message after sending SDO request
      data_603F_3[0] = rxbytes[4];
      data_603F_3[1] = rxbytes[5];
      bool_603F_3 = true;

    // Serial1.println("got statusword message");
  } 
  else if (id == 0x181) 
  {   // 6041 (statusword) {int16} 
      // 6041 Modes Of Operation Display {int 8}
      data_6041_1[0] = rxbytes[0];
      data_6041_1[1] = rxbytes[1];
      data_6061_1 = rxbytes[2];
      
      bool_6041_1 = true;
      bool_6061_1 = true;

  }
  else if (id == 0x182) 
  {   // 6041 (statusword) {int16} 
      // 6041 Modes Of Operation Display {int 8}
      data_6041_2[0] = rxbytes[0];
      data_6041_2[1] = rxbytes[1];
      data_6061_2 = rxbytes[2];
      
      bool_6041_2 = true;
      bool_6061_2 = true;

  }
  else if (id == 0x183) 
  {   // 6041 (statusword) {int16} 
      // 6041 Modes Of Operation Display {int 8}
      data_6041_3[0] = rxbytes[0];
      data_6041_3[1] = rxbytes[1];
      data_6061_3 = rxbytes[2];
      
      bool_6041_3 = true;
      bool_6061_3 = true;

  }
  else if (id == 0x281) 
  {   // 6064 Position actual value
      data_6064_1[0]  = rxbytes [0];
      data_6064_1[1]  = rxbytes [1];
      data_6064_1[2]  = rxbytes [2];
      data_6064_1[3]  = rxbytes [3];
      
      bool_6064_1 = true;


  }
  else if (id == 0x282) 
  {   // 6064 Position actual value
      data_6064_2[0]  = rxbytes [0];
      data_6064_2[1]  = rxbytes [1];
      data_6064_2[2]  = rxbytes [2];
      data_6064_2[3]  = rxbytes [3];
      
      bool_6064_2 = true;

  }
  else if (id == 0x283) 
  {   // 6064 Position actual value
      data_6064_3[0]  = rxbytes [0];
      data_6064_3[1]  = rxbytes [1];
      data_6064_3[2]  = rxbytes [2];
      data_6064_3[3]  = rxbytes [3];
      
      bool_6064_3 = true;

  }
  else if (id == 0x381) 
  {   // 6044 vl velocity actual value 
      data_6044_1[0]  = rxbytes [0];
      data_6044_1[1]  = rxbytes [1];
      
      bool_6044_1 = true;
  }
  else if (id == 0x382) 
  {   // 6044 vl velocity actual value 
      data_6044_2[0]  = rxbytes [0];
      data_6044_2[1]  = rxbytes [1];
      
      bool_6044_2 = true;
  }
  else if (id == 0x383) 
  {   // 6044 vl velocity actual value 
      data_6044_3[0]  = rxbytes [0];
      data_6044_3[1]  = rxbytes [1];
      
      bool_6044_3 = true;
  }
  
  else if (id == 0X0C8){
    // Jostick data
    can_bus_timeout_joy = false;

    can_joy_data[0] = rxbytes[0];
    can_joy_data[1] = rxbytes[1];
    can_joy_data[2] = rxbytes[2];
    can_joy_data[3] = rxbytes[3];
    can_joy_data[4] = rxbytes[4];
    can_joy_data[5] = rxbytes[5];
    can_joy_data[6] = rxbytes[6];
    can_joy_data[7] = rxbytes[7];
    
    can_oldtime_joy =  millis();
    
  }
  else if (id == 0X500){
    // NUC data
    can_bus_timeout_nuc = false;

    can_nuc_data[0] = rxbytes[0];
    can_nuc_data[1] = rxbytes[1];
    can_nuc_data[2] = rxbytes[2];
    can_nuc_data[3] = rxbytes[3];
    can_nuc_data[4] = rxbytes[4];
    can_nuc_data[5] = rxbytes[5];
    can_nuc_data[6] = rxbytes[6];
    can_nuc_data[7] = rxbytes[7];

    can_oldtime_nuc =  millis();
        
  }
    else if (id == 0X999){
    // CAN stop file
    can_stop = !can_stop;
    Serial1.println("CAN stop flip");
  }
  
  else
  {
      id_unkown_can = id;
      std::copy(rxbytes, rxbytes + 8, unknown_can);
      bool_unknown_can = true;
  }
      
}

void set_motor_postion_working(int nodeId, int position){
  SDO_write(nodeId,0x23,0x607A, 0x00, position); // 0x6042 Motor target Position
  delay(1);
  SDO_write(nodeId,0x2b,0x6040, 0x00, 0x03F);    // 0x6040 Controlword: Position
  delay(1);
  SDO_write(nodeId,0x2b,0x6040, 0x00, 0x00F);    // 0x6040 Controlword: Position
}

void motor_homing_func(int nodeId){

  SDO_write(nodeId,0x2f,0x6060, 0x00, 0x06);   // 0x6060 Motor setup: Go to homing mode
  delay(1);
  sendSDOReadRequest(nodeId, 0x6041, 0x00);
  delay(200);
  state_homing_1    = getHomingState(data_6041_1);

  if (state_homing_1 != 3){

    Serial1.println("Homing mode");
    SDO_write(nodeId,0x2f,0x6060, 0x00, 0x06);   // 0x6060 Motor setup: Go to homing mode
    delay(10);
    enableMotor(nodeId);                         // 6040
    delay(10);
    
    Serial1.println("Start homing");
    SDO_write(nodeId,0x2b,0x6040, 0x00, 0x001F);   // 0x6040 Controlword: Start Homing
    Serial1.println("homing wait state");

    state_homing_1    = getHomingState(data_6041_1);
    int while_loop_counter = 0;
    while (state_homing_1 != 3)
    {
      sendSDOReadRequest(nodeId, 0x6041, 0x00);
      delay(10);
      state_homing_1    = getHomingState(data_6041_1);
      SDO_write(nodeId,0x2f,0x6060, 0x00, 0x06);   // 0x6060 Motor setup: Go to homing mode
      delay(20);
      SDO_write(nodeId,0x2b,0x6040, 0x00, 0x001F);   // 0x6040 Controlword: Start Homing
      // delay(500);
      delay(100);
      digitalWrite(pin_led, LOW);
      delay(100);
      digitalWrite(pin_led, HIGH);
      delay(100);
      digitalWrite(pin_led, LOW);
      delay(100);
      digitalWrite(pin_led, HIGH);
      delay(10);
      while_loop_counter++;

      if (state_homing_1 == 3 || while_loop_counter > 40){
        if (while_loop_counter > 40){
          Serial1.println("Homing timed out");
          robot_error = 4; // Homing failed
        }
        else{
          Serial1.println("Homing completed succesfully ");
        }
        break;
      } 
    
    }
  digitalWrite(pin_led, HIGH);
  }
  else{
  Serial1.println();
  Serial1.print("Machine already homed. Skip homing");
  }


  Serial1.println();
  Serial1.print("actual_homing_state= ");
  Serial1.print(state_homing_1);
  Serial1.println("End homing");
}

void debug_serialprint(){
  Serial1.println();
  Serial1.print("actual_state= ");
  Serial1.print(state_mot_cont_1);
  Serial1.println();
  Serial1.println();
  Serial1.print("operation mode 1 = ");
  Serial1.print(operation_mode_1);
  Serial1.println();
}

void can_data_processing(){
  prev_actual_position_2 = actual_position_2;
  prev_actual_position_3 = actual_position_3;
  actual_position_1 = (int32_t)(data_6064_1[0] | (data_6064_1[1] << 8) | (data_6064_1[2] << 16) | (data_6064_1[3] << 24));
  actual_position_2 = (int32_t)(data_6064_2[0] | (data_6064_2[1] << 8) | (data_6064_2[2] << 16) | (data_6064_2[3] << 24));
  actual_position_3 = (int32_t)(data_6064_3[0] | (data_6064_3[1] << 8) | (data_6064_3[2] << 16) | (data_6064_3[3] << 24));
  actual_velocity_1 = (int16_t)(data_6044_1[0] | (data_6044_1[1] << 8));
  actual_velocity_2 = (int16_t)(data_6044_1[0] | (data_6044_1[1] << 8));
  actual_velocity_3 = (int16_t)(data_6044_1[0] | (data_6044_1[1] << 8));
  state_mot_cont_1  = getState(data_6041_1);
  state_mot_cont_2  = getState(data_6041_2);
  state_mot_cont_3  = getState(data_6041_3);
  state_homing_1    = getHomingState(data_6041_1);
  operation_mode_1  = data_6061_1;
  operation_mode_2  = data_6061_2;
  operation_mode_3  = data_6061_3;
  error_code_1      = (int16_t)(data_603F_1[0] | (data_603F_1[1] << 8));
  error_code_2      = (int16_t)(data_603F_2[0] | (data_603F_2[1] << 8));
  error_code_3      = (int16_t)(data_603F_3[0] | (data_603F_3[1] << 8));
  joystick_active   = can_joy_data[1];
  joystick_speed    = map(can_joy_data[2],0,200,-joystick_max_speed,joystick_max_speed); 
  joystick_steer    = map(can_joy_data[3],0,200,joystick_max_steer,-joystick_max_steer); 
  nuc_speed_desired = ((can_nuc_data[0] << 8) + can_nuc_data[1])-can_speed_offset;
  nuc_steer_desired = (can_nuc_data[2]-can_steer_offset);
  nuc_status        =  can_nuc_data[3];
  nuc_error_reset   =  can_nuc_data[4];
  }

// void calculateWheelSpeeds(double current_steer_angle, int speed_front_desired_func) {
//     // Convert steering angle from degrees to radians for calculation
//     double steer_angle_rad = current_steer_angle * M_PI / 180.0;
    
//     // Prevent division by zero when the steer angle is very small (robot is moving almost straight)
//     if (fabs(steer_angle_rad) < 1e-6) {
//         wheel_speed_left = speed_front_desired_func;
//         wheel_speed_right = speed_front_desired_func;
//         return;
//     }
//     // Calculate the turn radius
//     double turn_radius = wheelbase / tan(steer_angle_rad);

//     double v_rear = speed_front_desired_func * cos(current_steer_angle);

//     // Calculate wheel speeds
//     wheel_speed_right = int(v_rear * (1 + wheeltrack / (2 * turn_radius)));
//     wheel_speed_left  = int(v_rear * (1 - wheeltrack / (2 * turn_radius)));
// }

void calculateWheelSpeeds(double current_steer_angle, int speed_front_desired_func) {

    // Convert steering angle from degrees to radians
    double steer_angle_rad = current_steer_angle * M_PI / 180.0;

    // If the desired front speed is zero, both rear wheels should be stationary
    if (speed_front_desired_func == 0) {
        wheel_speed_left = 0.0;
        wheel_speed_right = 0.0;
        return;
    }

    // If the steering angle is zero, both rear wheels should move at the same speed as the front wheel
    if (steer_angle_rad == 0) {
        wheel_speed_left = speed_front_desired_func;
        wheel_speed_right = speed_front_desired_func;
        return;
    }

  // Calculate the desired yaw rate based on the current steering angle and front wheel speed
    double desired_yaw_rate = speed_front_desired_func * tan(steer_angle_rad) / wheelbase;

if (std::abs(desired_yaw_rate) > max_yaw_rate) {
    // Calculate the limited front wheel speed to ensure yaw rate does not exceed max_yaw_rate
    double limited_speed_front = max_yaw_rate * wheelbase / std::abs(tan(steer_angle_rad));

    // Preserve the sign of the original speed_front_desired_func
    limited_speed_front = std::copysign(limited_speed_front, speed_front_desired_func);

    // Update the front wheel speed to the limited speed
    speed_front_desired_func = limited_speed_front;

    // Update the desired yaw rate to the maximum allowable yaw rate, preserving its original sign
    desired_yaw_rate = std::copysign(max_yaw_rate, desired_yaw_rate);
}

    // Calculate the turning radius based on the current steering angle
    double radius = wheelbase / tan(steer_angle_rad);

    // Calculate the rear wheel speeds for Ackermann steering directly from the front wheel speed and steering angle
    // double v_inner = speed_front_desired_func * (radius - (wheeltrack / 2)) / radius;
    // double v_outer = speed_front_desired_func * (radius + (wheeltrack / 2)) / radius;

    wheel_speed_left = speed_front_desired_func * (radius - (wheeltrack / 2)) / radius;
    wheel_speed_right= speed_front_desired_func * (radius + (wheeltrack / 2)) / radius;

}
void read_error(){
  Serial1.println("Read error");
  bool_603F_1 = false;  // Set recieve flags down
  bool_603F_2 = false;
  bool_603F_3 = false;
  sendSDOReadRequest(1, 0x603F, 0x00);
  delay(50);
  sendSDOReadRequest(2, 0x603F, 0x00);
  delay(10);
  sendSDOReadRequest(3, 0x603F, 0x00);
  delay(10);
  if (bool_603F_1 == false || bool_603F_2 == false || bool_603F_3 == false){
    Serial1.println("Recieve error not set wait for 100 ms");
    delay(100);
    Serial1.println(bool_603F_1);
    Serial1.println(bool_603F_2);
    Serial1.println(bool_603F_3);
  }
  if (bool_603F_1 == false || bool_603F_2 == false || bool_603F_3 == false){
    Serial1.println("Recieve error not set second time");
    }
  
}

void enable_motors_from_error(){
  read_error();
  if (state_mot_cont_1 == 0){
    error_code_1 =      (int16_t)(data_603F_1[0] | (data_603F_1[1] << 8));
    Serial1.println("Error motor controller 1");
    Serial1.print("Error is: ");
    Serial1.println(error_code_1,HEX);
    reset_error(1);
    delay(1);
    SDO_write(1,0x2f,0x6060, 0x00, 0x1);   // 0x6060 Motor setup: 1 Profile position 3 is Profile speed
    delay(1);
    enableMotor(1); 

  }
  if (state_mot_cont_2 == 0){
    error_code_2 =      (int16_t)(data_603F_2[0] | (data_603F_2[1] << 8));
    Serial1.println("Error motor controller 2");
    Serial1.print("Error is: ");
    Serial1.println(error_code_2,HEX);
    reset_error(2);
    delay(1);
    SDO_write(2,0x2f,0x6060, 0x00, 0x3);   // 0x6060 Motor setup: 1 Profile position 3 is Profile speed
    delay(1);
    enableMotor(2);   
  }
  if (state_mot_cont_3 == 0){
    error_code_3 =      (int16_t)(data_603F_3[0] | (data_603F_3[1] << 8));
    Serial1.println("Error motor controller 3");
    Serial1.print("Error is: ");
    Serial1.println(error_code_3,HEX);
    reset_error(3);
    delay(1);
    SDO_write(1,0x2f,0x6060, 0x00, 0x3);   // 0x6060 Motor setup: 1 Profile position 3 is Profile speed
    delay(1);
    enableMotor(3); 
  }
}

void state_machine(){ // Output disered state, disired speed, disered steer angle.
// Check which interface is availeble
  
  if(millis() - can_oldtime_nuc > can_rx_timeout || millis() - can_oldtime_joy > can_rx_timeout){    // Check timeouttime CAN-bus when mode selector is ON
    if(millis() - can_oldtime_nuc > can_rx_timeout){
      can_bus_timeout_nuc = true;
      nuc_speed_desired = 0;
      nuc_speed_desired = 0;
    }

    if(millis() - can_oldtime_joy > can_rx_timeout){
      can_bus_timeout_joy = true;
      joystick_speed = 0;
      joystick_steer = 0;
    }
    
    if(joystick_active == 0){
      joystick_speed = 0;
      joystick_steer = 0;
    }
  }

  if(joystick_active == 0 && can_bus_timeout_nuc == false  && robot_error == 0){            // Robot controlled by NUC | State 1
    steer_desired = nuc_steer_desired;
    speed_desired = nuc_speed_desired;
    state_robot   = 1;
  }
  
  if(joystick_active == 1 && can_bus_timeout_joy == false && robot_error == 0){        // Robot controlled by Joystick | State 2
    steer_desired = joystick_steer*-1;
    speed_desired = joystick_speed;
    state_robot   = 2;
  }
  
  if((abs(prev_actual_position_2 - actual_position_2) > delta_pos_move_treshold-1 || abs(prev_actual_position_3 - actual_position_3) > delta_pos_move_treshold-1) && nuc_speed_desired == 0 && joystick_speed == 0 && (can_bus_timeout_nuc == false ||can_bus_timeout_joy == false)){                                                                                // Robot desired speed is 0 and actual speed = 0
    steer_desired = 0;
    speed_desired = 0;
    state_robot   = 3;
  }   

  if(abs(prev_actual_position_2 - actual_position_2) < delta_pos_move_treshold && abs(prev_actual_position_3 - actual_position_3) < delta_pos_move_treshold && nuc_speed_desired == 0 && joystick_speed == 0 && (can_bus_timeout_nuc == false ||can_bus_timeout_joy == false)){                                                                                // Robot desired speed is 0 and actual speed = 0
    steer_desired = 0;
    speed_desired = 0;
    state_robot   = 4;
  }

  if(can_bus_timeout_nuc == true && can_bus_timeout_joy == true){    
    steer_desired = 0;
    speed_desired = 0;                                                                                // CANbus timeout both occured
    state_robot   = 5;
  }   

  if(digitalRead(pin_bumper_1) == HIGH || digitalRead(pin_bumper_2) == HIGH || millis()-bumper_timer < bumper_reactivated_delay){  
    if(digitalRead(pin_bumper_1) == HIGH || digitalRead(pin_bumper_2) == HIGH){
      bumper_timer = millis();
    }
    Serial1.println("State machine [5]: Bumper is activated");
    steer_desired = 0;
    speed_desired = 0;                                                                                // Bumper is hit
    state_robot   = 6;
    robot_error   = 2;
  }   
  // Serial1.println(digitalRead(pin_em_stop));
  if (digitalRead(pin_em_stop) == HIGH){                                                     // EM_stop is pressed
    Serial1.println("State machine [6]: EM stop pressed");
    steer_desired = 0;
    speed_desired = 0;
    state_robot   = 7;
    robot_error   = 1;
  }  
  if ((digitalRead(pin_em_stop) == LOW && robot_error == 1)||(robot_error == 2 && state_robot != 6 && state_robot != 7 )||(state_homing_1 == 3 && robot_error == 4)||(state_mot_cont_1 == 3 && state_mot_cont_2 == 3 && state_mot_cont_3 == 3 && robot_error == 3)||(robot_error == 2 && (millis()-bumper_timer) > bumper_reactivated_delay)){
    Serial1.println("reset error");
    robot_error   = 0;
  }

  // else{                
  //   Serial1.print("State machine [0]: Stateless");                                            // Robot not controlled stateless
  //   steer_desired = 0;
  //   speed_desired = 0;
  //   state_robot   = 0; 
  // }
  speed_desired = constrain(speed_desired, speed_max*-1 , speed_max);                   // Limit Speed
  steer_desired = constrain(steer_desired, steer_max_angle*-1 , steer_max_angle);       // Limit Angle
 
 if (prev_state_robot != state_robot){
  Serial1.print("Previous State = ");
  Serial1.print(prev_state_robot);
  Serial1.print("New State = ");
  Serial1.print(state_robot);
 }

prev_state_robot = state_robot;


}

void motor_and_relay_state_control(){
 if(state_robot > 0 && state_robot < 4){
  int des_state_motor = 3;
    if(state_mot_cont_1 != des_state_motor || state_mot_cont_2 != des_state_motor || state_mot_cont_3 != des_state_motor){ // Enable motors
      Serial1.println("Motor controller enable");
      // Serial1.println(state_mot_cont_1);
      // Serial1.println(state_mot_cont_2);
      // Serial1.println(state_mot_cont_3);
      // Serial1.println(millis());
      enableMotor(1);
      enableMotor(2);
      enableMotor(3);
      delay(1);
      myCAN.transmit(0x080,0,0);
      delay(100);
      state_mot_cont_1  = getState(data_6041_1);
      state_mot_cont_2  = getState(data_6041_2);
      state_mot_cont_3  = getState(data_6041_3);
    
    if(state_mot_cont_1 != des_state_motor || state_mot_cont_2 != des_state_motor || state_mot_cont_3 != des_state_motor){ // 1st Check if motors are enabled
      // Reset error and enable motors
      Serial1.println("Motor controller error, Attempt to reset");
      delay(1);
      myCAN.transmit(0x080,0,0);
      delay(100);
      state_mot_cont_1  = getState(data_6041_1);
      state_mot_cont_2  = getState(data_6041_2);
      state_mot_cont_3  = getState(data_6041_3);
      enable_motors_from_error();
    }
    if(state_mot_cont_1 != des_state_motor || state_mot_cont_2 != des_state_motor || state_mot_cont_3 != des_state_motor){ // 2nd Check if motors are enabled after error reset
      robot_error = 3; // motorcontroller error
      Serial1.println("Motor controller error, reset impossible");
    }
  }

 }
 else if ((state_robot > 3 && state_robot < 7) || state_robot < 1){
  // Motor need to be de-activated
  // if(state_mot_cont_1 != 2 || state_mot_cont_2 != 2 || state_mot_cont_3 != 2){ // Commented to not turn of steering motor
    if(state_mot_cont_2 != 2 || state_mot_cont_3 != 2){
    Serial1.println("Motor controller ,Standby motor");
    // standbyMotor(1); // Commented to not turn of steering motor
    standbyMotor(2);
    standbyMotor(3);
  }
 }

  // Brake relay state
 if (state_robot == 7 && (abs(prev_actual_position_2 - actual_position_2) > delta_pos_move_treshold || abs(prev_actual_position_3 - actual_position_3) > delta_pos_move_treshold)){
    Serial1.print("EM stop pressed and robot moving");
    steer_desired = 0;
    speed_desired = 0;
    digitalWrite(pin_brake_relay, HIGH);
    brake_relay_state = 1;
  }

  else if (state_robot == 7 && (actual_velocity_2 < 80 || actual_velocity_3 < 80)){
    Serial1.print("Disable brake relay");
    digitalWrite(pin_brake_relay, LOW);
    brake_relay_state = 0;
  }

  else if (state_robot != 7 && brake_relay_state == 1){
    Serial1.print("Disable brake relay");
    digitalWrite(pin_brake_relay, LOW);
  }
  }

void set_pin_mode(){
  pinMode(pin_tipover_1,INPUT_PULLUP);
  pinMode(pin_tipover_2,INPUT_PULLUP);
  pinMode(pin_em_stop, INPUT_PULLUP);
  pinMode(pin_bumper_1,INPUT);
  pinMode(pin_bumper_2,INPUT);
  pinMode(pin_brake_relay,OUTPUT);
  pinMode(pin_led,OUTPUT);
}

void setup() {
  Serial1.begin(115200);
  Serial1.println("begin");

  set_pin_mode();

  digitalWrite(pin_led, HIGH);
  delay(250);
  digitalWrite(pin_led, LOW);
  delay(250);
  digitalWrite(pin_led, HIGH);
  delay(250);
  digitalWrite(pin_led, LOW);
  delay(250);
  digitalWrite(pin_led, HIGH);

  myCAN.attachInterrupt(canISR);
  int delay_time = 10;

  if (bool_6041_1 == 0){  // Check if 6041 is already recieved (mcu reset of cold start). Ohterwise wait for potential start of motorcontrollers
   delay(3000);
   Serial1.print("potential cold start. wait 3 sec");
  }

  sendNMTCommand(CS_STOP_REMOTE_NODE, 0); // 0 for broadcast to all nodes
  delay(delay_time);
  sendNMTCommand(CS_START_REMOTE_NODE, 0); // 0 for broadcast to all nodes
  delay(delay_time);


  // CAN_receive_func();
  Serial1.println("End read error");

  reset_error(1);
  delay(delay_time);
  reset_error(2);
  delay(delay_time);
  reset_error(3);
  delay(delay_time);


  motor_homing_func(1);

  Serial1.println("Setup motor ");
  SDO_write(1,0x23,0x60ff, 0x00, -1);      // Set speed for postion control
  delay(delay_time);
  SDO_write(1,0x2f,0x6060, 0x00, 0x1);   // 0x6060 Motor setup: 1 Profile position 3 is Profile speed
  delay(delay_time);
  Serial1.println("Enable motor");
  enableMotor(1);          // 6040
  Serial1.println("Start main loop ");
  set_motor_postion_working(1,0);

  delay(delay_time);
  Serial1.println("Setup motor 2 ");
  SDO_write(2,0x23,0x60ff, 0x00, 0);      // Set speed for postion control
  delay(delay_time);
  SDO_write(2,0x2f,0x6060, 0x00, 0x3);   // 0x6060 Motor setup: 1 Profile position 3 is Profile speed
  delay(delay_time);
  Serial1.println("Enable motor");
  enableMotor(2);          // 6040
  Serial1.println("Start main loop ");

  delay(delay_time);
  Serial1.println("Setup motor 3 ");
  SDO_write(3,0x23,0x60ff, 0x00, 0);      // Set speed for postion control
  delay(delay_time);
  SDO_write(3,0x2f,0x6060, 0x00, 0x3);   // 0x6060 Motor setup: 1 Profile position 3 is Profile speed
  delay(delay_time);
  Serial1.println("Enable motor");
  enableMotor(3);          // 6040
  Serial1.println("Start main loop ");
  delay(1000);


}

void can_send_func(){
    CAN_frame_t Frame;
    Frame.id = 0x600; //
    Frame.len = 8; // 

    Frame.data[0] = 0x0;
    Frame.data[1] = 0x0;
    Frame.data[2] = robot_error;
    Frame.data[3] = state_robot;

    Frame.data[4] = 0x0;
    Frame.data[5] = 0x0;
    Frame.data[6] = 0x0;
    Frame.data[7] = 0x0;


    // Send the PDO frame
    myCAN.transmit(Frame.id,Frame.data,Frame.len); 
}

void updateErrorLED() {
    unsigned long currentMillis = millis();

    if (robot_error == 0) {
        // Error state 0: LED should be on
        digitalWrite(pin_led, HIGH);
        offPeriod = false;
        currentBlinkCount = 0;
    } else {
        if (offPeriod) {
            if (currentMillis - previousMillis >= offInterval) {
                // End the off period and start blinking again
                offPeriod = false;
                previousMillis = currentMillis;
                currentBlinkCount = 0;
                ledState = LOW; // Ensure LED starts from LOW state
                digitalWrite(pin_led, ledState);
            }
        } else {
            // Blinking period
            if (currentMillis - previousMillis >= blinkInterval) {
                previousMillis = currentMillis;
                ledState = !ledState;
                digitalWrite(pin_led, ledState);

                if (ledState == HIGH) {
                    currentBlinkCount++;
                }

                // Check if we have completed the required number of blinks
                if (currentBlinkCount >= robot_error+1) {
                    offPeriod = true;
                    digitalWrite(pin_led, LOW); // Ensure the LED is off during the off period
                    previousMillis = currentMillis; // Reset the timer for the off period
                }
            }
        }
    }
}

void loop() {
  if (can_stop == false){
    // debug_serialprint();
    can_data_processing();
    state_machine();
    motor_and_relay_state_control();

    delay(1);
    // Send the SYNC message to recieve PDO
    myCAN.transmit(0x080,0,0);

    // Calculate wheel speeds
    int motor_position_pulse = calculatePositionInPulses(steer_desired, steer_pulses_per_rot, steer_pulses_homing_offset,steer_transmission_ratio);
    current_steer_angle = calculatePositionInDegrees(steer_pulses_per_rot,steer_pulses_homing_offset,steer_transmission_ratio,actual_position_1);
    calculateWheelSpeeds(current_steer_angle,speed_desired);

   // Send Nosewheel position
    delay(1);
     if (prev_motor_position_pulse != motor_position_pulse){
      set_motor_postion_working(1,motor_position_pulse);
      prev_motor_position_pulse =motor_position_pulse;
    }
    
    // Wheel control
    delay(1);
    if (prev_wheel_speed_right != wheel_speed_right){
      SDO_write(2,0x23,0x60ff, 0x00, (wheel_speed_right*-1));
      prev_wheel_speed_right = wheel_speed_right;
    }
    delay(1);
    if (prev_wheel_speed_left != wheel_speed_left){
      SDO_write(3,0x23,0x60ff, 0x00, (wheel_speed_left));
      prev_wheel_speed_left = wheel_speed_left;
    }
    can_send_func();
    updateErrorLED();

}



delay(16);
}


