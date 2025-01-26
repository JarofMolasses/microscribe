#include <Arduino.h>
#include "hci.h"
#include "arm.h"
#include "drive.h"

// The order of the includes is important. For reasons I don't understand
// Arduino quirk: if using onboard USB COM port, DTR signal sent by the terminal software trips reset 
// If you want to open and close the terminal software at will, 
//  1.add a capacitor 10uF to reset pin. Remove to flash over USB, or manually enter bootloader on upload.
//  2.use an external USB-TTL on the RX/TX lines directly to implement USB 

arm_rec arm;
long baud = 19200L;   // <<<<<<<<< HCI has an autosynch function, but it seems to only work with 19200
int port = 1;
byte buffer[1]; 

void printXYZ(arm_rec arm);
void home(arm_rec arm);

typedef enum PrintMode{
  PRINT_CONT,
  PRINT_DISCONT,
  PRINT_NONE
} printmode_t;

printmode_t printMode = PRINT_CONT;
arm_result result;

void setup() {
  Serial.begin(9600);
  delay(1000);
  // Initialize and connect Microscribe arm

  arm_init(&arm);
  arm_install_simple(&arm);
  result = arm_connect(&arm, port, baud);
  delay(1000);

  // Home arm at bootup position
  result = arm_home_pos(&arm);
}

void loop() {
  // Always check for a byte command and process it
  if(Serial.available()  )
  {
    Serial.readBytes(buffer, 1);
    switch ((int)buffer[0])
    {
      case 'c':
        // default continuous mode, report XYZ constantly
        printMode = PRINT_CONT;
        Serial.println("Continuous mode");
        break;
      case 'n':
        printMode = PRINT_NONE;
        Serial.println("Printing off");
        break;
      case 'd':
      // discontinuous mode
        printMode = PRINT_DISCONT;
        Serial.println("Discontinuous mode");
        break;
      case 'h':
      // home 
        home(arm);
      default:
        break;
    }
  }


  // get update from Microscribe
  result = arm_stylus_6DOF_update(&arm);
   
	// print coordinates to Serial Monitor
  switch(printMode)
  {
    case PRINT_CONT:
      printXYZ(arm);
    break;
    case PRINT_NONE:
    break;
    case PRINT_DISCONT:
    break;
    default:
    break;
  }

  delay(100); // rudimentary spam limiter. Use timer interrupts for serious applications
}

void printXYZ(arm_rec arm)
{
  Serial.print("XYZ,");
  Serial.print(arm.stylus_tip.x);
  Serial.print(",");
  Serial.print(arm.stylus_tip.y);
  Serial.print(",");
  Serial.print(arm.stylus_tip.z);
  Serial.println("");
}

void home(arm_rec arm)
{
  arm_home_pos(&arm);
  Serial.println("Home arm position");
}