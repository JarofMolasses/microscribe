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

void setup() {
  Serial.begin(9600);
  delay(1000);
  // Initialize and connect Microscribe arm

  arm_init(&arm);
  arm_install_simple(&arm);
  arm_result result = arm_connect(&arm, port, baud);
}

void loop() {
   // get update from Microscribe
   arm_result result = arm_stylus_6DOF_update(&arm);
   
	// print coordinates to Serial Monitor
  Serial.print(arm.stylus_tip.x);
  Serial.print("   ");
  Serial.print(arm.stylus_tip.y);
  Serial.print("   ");
  Serial.print(arm.stylus_tip.z);
  Serial.println(" ");

  delay(100); // rudimentary spam limiter. Use timer interrupts for serious applications
}