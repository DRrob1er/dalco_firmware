#include <functions.h>
#include <Arduino.h>


// Cheatsheet CANopen:

//  IDX     What                                  Type          Down
//  6060 =  operation motor: 02 is speed control  [int_8]       0x2f
//  6042 =  target velocity                       [int_16]      0x2b
//  6040 =  controlword                           [unsigned16]  0x2b

// 1 byte = 0x2f
// 2 byte = 0x2b
// 3 byte = 0x27
// 4 byte = 0x2


struct CAN_frame_t {
    int id;      
    uint8_t data[8];  
    uint8_t len;
};

int test_func(int a, int b){
    int c = a+b;
    return c;
}

void sendNMTCommand(uint8_t cs, uint8_t nodeId) {
  // Construct NMT command frame
  CAN_frame_t nmtFrame;
  nmtFrame.id = 0x0000; // COB-ID for NMT
  nmtFrame.len = 2;
  nmtFrame.data[0] = cs; // Control State
  nmtFrame.data[1] = nodeId; // Node ID

  // Send frame
  myCAN.transmit(nmtFrame.id,nmtFrame.data,nmtFrame.len);
}

void enableMotor(int nodeId) {
    // Transition to "Ready to Switch On"
    SDO_write(nodeId,0x2b,0x6040, 0x00, 0x0006);   // 0x6040 Controlword: Transition to "Ready to Switch On"
    delay(100); // Wait for the state transition; adjust delay as needed

    // Transition to "Switched On"
    SDO_write(nodeId,0x2b,0x6040, 0x00, 0x0007);   // 0x6040 Controlword: Transition to "Switched On"
    delay(100); // Wait for the state transition; adjust delay as needed

    // Enable Operation
    SDO_write(nodeId,0x2b,0x6040, 0x00, 0x000F);   // 0x6040 Controlword: Enable Operation
}

void sendSDOReadRequest(uint8_t nodeId, uint16_t index, uint8_t subIndex) {
    CAN_frame_t sdoFrame;
    sdoFrame.id = 0x600 + nodeId; // COB-ID for SDO client to server (RxSDO) is 0x600 + node ID
    sdoFrame.len = 8; // SDO frame length is always 8 bytes
    sdoFrame.data[0] = 0x40; // Command byte for read request

    // Index of the object to read (little endian)
    sdoFrame.data[1] = index & 0xFF; // Low byte of index
    sdoFrame.data[2] = (index >> 8) & 0xFF; // High byte of index
    // sdoFrame.data[1] = 0x01; // Low byte of index
    // sdoFrame.data[2] = 0x10; // High byte of index

    // Sub-index of the object to read
    sdoFrame.data[3] = subIndex;

    // The remaining bytes are not used in the request and should be set to 0
    for (int i = 4; i < 8; i++) {
        sdoFrame.data[i] = 0;
    }

    // Send the SDO frame
    myCAN.transmit(sdoFrame.id,sdoFrame.data,sdoFrame.len);
}

void SDO_write(int nodeId,  int down_byte,  int IDX, int IDX_sub, int32_t data) {
  // Construct PDO frame 

  CAN_frame_t sdoFrame;
  sdoFrame.id = 0x600 + nodeId;         // COB-ID for SDO request
  sdoFrame.len = 8;

  sdoFrame.data[0] = down_byte;         // Command specifier for expedited download, 2 bytes of data
  sdoFrame.data[1] = IDX & 0xFF;
  sdoFrame.data[2] = (IDX >> 8) & 0xFF;
  sdoFrame.data[3] = IDX_sub;           // Sub-index

  // Controlword value (little-endian format)
  if  (down_byte == 0x2f){
    sdoFrame.data[4] = data & 0xFF;
    sdoFrame.data[5] = 0x00;
    sdoFrame.data[6] = 0x00;
    sdoFrame.data[7] = 0x00;
  }

  else if (down_byte == 0x2b){
    sdoFrame.data[4] = data & 0xFF;
    sdoFrame.data[5] = (data >> 8) & 0xFF;
    sdoFrame.data[6] = 0x00;
    sdoFrame.data[7] = 0x00;
  }

  else if (down_byte == 0x27){
    sdoFrame.data[4] = data & 0xFF;
    sdoFrame.data[5] = (data >> 8) & 0xFF;
    sdoFrame.data[6] = (data >> 16) & 0xFF;
    sdoFrame.data[7] = 0x00;
  }

  else if (down_byte == 0x23){
    sdoFrame.data[4] = data & 0xFF;
    sdoFrame.data[5] = (data >> 8) & 0xFF;
    sdoFrame.data[6] = (data >> 16) & 0xFF;
    sdoFrame.data[7] = (data >> 24) & 0xFF;
  }
  else{
    Serial1.print("SDO_write error");
  }

  myCAN.transmit(sdoFrame.id,sdoFrame.data,sdoFrame.len);
}

void PDO_write_int32(int nodeId, int PDO_Id, int32_t data_pos, int32_t data_vel){
    CAN_frame_t pdoFrame;
    pdoFrame.id = PDO_Id + nodeId; // COB-ID for PDOs is 0x300 + node ID
    pdoFrame.len = 8; // Assuming the target position is a 32-bit integer

    // Encode the target position into the PDO frame (assuming little-endian format)
    pdoFrame.data[0] = data_pos & 0xFF; // Least significant byte
    pdoFrame.data[1] = (data_pos >> 8) & 0xFF;
    pdoFrame.data[2] = (data_pos >> 16) & 0xFF;
    pdoFrame.data[3] = (data_pos >> 24) & 0xFF; // Most significant byte

    pdoFrame.data[4] = data_vel & 0xFF; // Least significant byte
    pdoFrame.data[5] = (data_vel >> 8) & 0xFF;
    pdoFrame.data[6] = (data_vel >> 16) & 0xFF;
    pdoFrame.data[7] = (data_vel >> 24) & 0xFF; // Most significant byte


    // Send the PDO frame
    myCAN.transmit(pdoFrame.id,pdoFrame.data,pdoFrame.len);
}

void PDO_write_int16(int nodeId, int PDO_Id, int16_t data){
    CAN_frame_t pdoFrame;
    pdoFrame.id = PDO_Id + nodeId; // COB-ID for PDOs is 0x300 + node ID
    pdoFrame.len = 4; // Assuming the target position is a 32-bit integer

    // Encode the target position into the PDO frame (assuming little-endian format)
    pdoFrame.data[0] = data & 0xFF; // Least significant byte
    pdoFrame.data[1] = (data >> 8) & 0xFF;
    pdoFrame.data[2] = (data >> 16) & 0xFF;
    pdoFrame.data[3] = (data >> 24) & 0xFF; // Most significant byte

    // Send the PDO frame
    myCAN.transmit(pdoFrame.id,pdoFrame.data,pdoFrame.len);
}

void CAN_filters(){
  myCAN.filterMask16Init(0, 0X100, 0x7ff);
  myCAN.filterMask16Init(1, 0X110, 0x7ff);
}

int getState(const uint8_t bytes[2]) {
    // Check the bits for each state according to the patterns you provided
    // Convert the 2-byte array into a uint16_t value assuming little-endian format
    
    int state = -1; // Default state when no condition matches    

    uint16_t statusWord = bytes[0] | (bytes[1] << 8);
    uint16_t reversedStatusWord = 0;
    for (int i = 0; i < 16; ++i) {
        reversedStatusWord |= ((statusWord >> i) & 1) << (15 - i);
    }
    
    String generalState;
    if ((statusWord & 0b00000111) == 0b00000000) {
        generalState = "Not ready to switch on";
        state = 0;
    } else if ((statusWord & 0b01010000) == 0b01010000) {
        generalState = "Switch on disabled";
        state = 1;
    } else if ((statusWord & 0b00100011) == 0b00100001) {
        generalState = "Ready to switch on";
        state = 2;
    } else if ((statusWord & 0b00100011) == 0b00100011) {
        generalState = "Switched on";
        state = 3;
    } else if ((statusWord & 0b00100111) == 0b00100111) {
        generalState = "Operation enabled";
        state = 4;
    } else if ((statusWord & 0b00000111) == 0b00000111) {
        generalState = "Quick stop active";
        state = 5;
    } else if ((statusWord & 0b01011000) == 0b00010000) {
        generalState = "Fault reaction active";
        state = 6;
    } else if ((statusWord & 0b01011000) == 0b00010000) {
        generalState = "Fault";
        state = 7;
    } else {
        generalState = "Unknown general state";
        state = 8;
    }

    return state; // If none of the patterns match
}

int getHomingState(const uint8_t bytes[2]){
  int homing_state = -1;

  uint16_t statusWord = bytes[0] | (bytes[1] << 8);
   uint16_t reversedStatusWord = 0;
    for (int i = 0; i < 16; ++i) {
        reversedStatusWord |= ((statusWord >> i) & 1) << (15 - i);
    }

    // Interpret the homing status
    String homingStatus;
    if (!(reversedStatusWord & (1 << 5)) && !(reversedStatusWord & (1 << 3)) && !(reversedStatusWord & (1 << 2))) {
        homingStatus = "Homing is performed";
        homing_state = 1;
    } else if (!(reversedStatusWord & (1 << 5)) && !(reversedStatusWord & (1 << 3)) && (reversedStatusWord & (1 << 2))) {
        homingStatus = "Homing is interrupted or not started";
        homing_state = 2;
    } else if ((reversedStatusWord & (1 << 5)) && (reversedStatusWord & (1 << 3)) && !(reversedStatusWord & (1 << 2))) {
        homingStatus = "Homing confirmed but target not yet reached";
        homing_state = 3;
    } else if ((reversedStatusWord & (1 << 5)) && (reversedStatusWord & (1 << 3)) && (reversedStatusWord & (1 << 2))) {
        homingStatus = "Homing completed";
        homing_state = 4;
    } else if (!(reversedStatusWord & (1 << 5)) && (reversedStatusWord & (1 << 3)) && !(reversedStatusWord & (1 << 2))) {
        homingStatus = "Error during homing, motor still turning";
        homing_state = 5;
    } else if (!(reversedStatusWord & (1 << 5)) && (reversedStatusWord & (1 << 3)) && (reversedStatusWord & (1 << 2))) {
        homingStatus = "Error during homing, motor at standstill";
        homing_state = 6;
    } else {
        homingStatus = "Unknown homing status";
        homing_state = 7;
    }
    return homing_state;  // Return the determined state
}




// ----------- Old Functions ---------------

// void setup_motor_mode(int nodeId, int config) {
//   // Construct PDO frame 
//   CAN_frame_t pdoFrame;
//   pdoFrame.id = 0x600 + nodeId; // Example COB-ID, adjust as per your setup
//   pdoFrame.len = 8;
//   // Data mapping depends on your motor controller's CANopen implementation
//   uint16_t IDX = 0x6060;
//   pdoFrame.data[0] = 0x2f;
//   pdoFrame.data[1] = IDX & 0xFF;
//   pdoFrame.data[2] = (IDX >> 8) & 0xFF;
// //   pdoFrame.data[1] = 0x60;
// //   pdoFrame.data[2] = 0x60;
//   pdoFrame.data[3] = 0x00;

//   pdoFrame.data[4] = config;
//   pdoFrame.data[5] = 0x00;
//   pdoFrame.data[6] = 0x00;
//   pdoFrame.data[7] = 0x00;

//   // Send frame
//   myCAN.transmit(pdoFrame.id,pdoFrame.data,pdoFrame.len);
// }

// void writeControlword(int nodeId, uint16_t type) {
//   // Construct PDO frame 

//   CAN_frame_t sdoFrame;
//   sdoFrame.id = 0x600 + nodeId; // COB-ID for SDO request
//   sdoFrame.len = 8;
//   uint16_t IDX = 0x6040;

//   sdoFrame.data[0] = 0x2B; // Command specifier for expedited download, 2 bytes of data
//   sdoFrame.data[1] = IDX & 0xFF;
//   sdoFrame.data[2] = (IDX >> 8) & 0xFF;
//   sdoFrame.data[3] = 0x00; // Sub-index

//   // Controlword value (little-endian format)
//   sdoFrame.data[4] = type & 0xFF;
//   sdoFrame.data[5] = (type >> 8) & 0xFF;
//   sdoFrame.data[6] = 0x00;
//   sdoFrame.data[7] = 0x00;

//   myCAN.transmit(sdoFrame.id,sdoFrame.data,sdoFrame.len);
// }

// void desired_motor_speed_SDO(int nodeId, int speed) {
//   // Construct PDO frame 
//   CAN_frame_t pdoFrame;
//   pdoFrame.id = 0x600 + nodeId; // Example COB-ID, adjust as per your setup
//   pdoFrame.len = 8;
//   // Data mapping depends on your motor controller's CANopen implementation
//   uint16_t IDX = 0x6042;
//   pdoFrame.data[0] = 0x2b;
//   pdoFrame.data[1] = IDX & 0xFF;
//   pdoFrame.data[2] = (IDX >> 8) & 0xFF;
// //   pdoFrame.data[1] = 0x42;
// //   pdoFrame.data[2] = 0x60;
//   pdoFrame.data[3] = 0x00;

//   pdoFrame.data[4] = speed & 0xFF;
//   pdoFrame.data[5] = (speed >> 8) & 0xFF;
//   pdoFrame.data[6] = 0x00;
//   pdoFrame.data[7] = 0x00;


//   // Send frame
//   myCAN.transmit(pdoFrame.id,pdoFrame.data,pdoFrame.len);

// }