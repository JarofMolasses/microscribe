#include <Arduino.h>
#include "hci.h"
#include "arm.h"
#include "drive.h"

#define ID_PHRASE "MICROSCRIBE SERIAL INTERFACE"
//#define AUTOCONNECT       // define this if you want the interface to automatically connect on startup without needing to send connect command

// The order of the includes is important. For reasons I don't understand
// Arduino quirk: if using onboard USB COM port, DTR signal sent by the terminal software trips reset 
// If you want to open and close the terminal software at will, 
//  1.add a capacitor 10uF to reset pin. Remove to flash over USB, or manually enter bootloader on upload.
//  2.use an external USB-TTL on the RX/TX lines directly to implement USB 

arm_rec arm;
long baud = 19200L;   // <<<<<<<<< HCI has an "autosynch" function, but it seems to only work with 19200
int port = 1;
byte buffer[1]; 

void printXYZ(arm_rec arm);
void home(arm_rec arm);

enum ConnectionState{
  FALSE = 0x0,
  TRUE
};
int arm_connected = FALSE;

typedef enum PrintMode{
  PRINT_CONT,
  PRINT_DISCONT,
  PRINT_NONE,
  PRINT_QUERY
} printmode_t;
printmode_t printMode = PRINT_QUERY;
arm_result result;

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, arm_connected);

  // Initialize and connect Microscribe arm
  arm_init(&arm);
  arm_install_simple(&arm);

  #ifdef AUTOCONNECT
  result = arm_connect(&arm, port, baud);
  arm_connected = TRUE;
  delay(1000);

  Serial.println("READY");
  digitalWrite(13, arm_connected);
  #endif
}

void loop() {
  // Always check for a byte command and process it
  if(Serial.available())
  {
    Serial.readBytes(buffer, 1);
    switch ((int)buffer[0])
    {
      case 'i':
        // identify machine 
        Serial.println(ID_PHRASE);
        break;
      case 'c':
        // default continuous mode, report XYZ constantly
        printMode = PRINT_CONT;
        Serial.println("CONTINUOUS MODE");
        break;
      case 'n':
        printMode = PRINT_NONE;
        Serial.println("PRINTING OFF");
        break;
      case 'd':
      // discontinuous mode
        printMode = PRINT_DISCONT;
        Serial.println("DISCONTINUOUS MODE");
        break;
      case 'q':
      // query mode. Software flow control
        printMode = PRINT_QUERY;
        Serial.println("QUERY MODE");
        break;
      case 'h':
      // home 
        home(arm);
        break;
      case '>':
      // query command
        if(printMode == PRINT_QUERY)
        {
          printXYZ(arm);
        }
        break;
      case 'x':
        arm_disconnect(&arm);
        Serial.println("DISCONNECTED");
        arm_connected = FALSE;      // Consider: what if this function fails? This is pretty crude, we should rather just track whether the serial port assigned to the arm is open.
        digitalWrite(13, arm_connected);
        break;
      case 'r':
        arm_connect(&arm, port, baud);
        Serial.println("READY");
        arm_connected = TRUE;
        digitalWrite(13, arm_connected);
      default:
        break;
    }
  }

  // get update from Microscribe
  if(arm_connected == TRUE) result = arm_stylus_6DOF_update(&arm);
   
	// print coordinates to Serial Monitor
  switch(printMode)
  {
    case PRINT_CONT:
      printXYZ(arm);
      delay(70); // rudimentary spam limiter. Use timer interrupts for serious applications
    break;
    default:
    break;
  }
}

void printXYZ(arm_rec arm)
{
  Serial.print("XYZ,");
  Serial.print(arm.stylus_tip.x);
  Serial.print(",");
  Serial.print(arm.stylus_tip.y);
  Serial.print(",");
  Serial.print(arm.stylus_tip.z);
  Serial.print(",");
  Serial.print(arm.stylus_dir.x);
  Serial.print(",");
  Serial.print(arm.stylus_dir.y);
  Serial.print(",");
  Serial.print(arm.stylus_dir.z);
  Serial.println("");
}

void home(arm_rec arm)
{
  arm_home_pos(&arm);
  Serial.println("RESET HOME LOCATION");
}