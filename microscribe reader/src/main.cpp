// The order of the includes is important. For reasons I don't understand
// Arduino quirk: if using onboard USB COM port, DTR signal sent by the terminal software trips reset 
// If you want to open and close the terminal software at will, 
//  1.add a capacitor 10uF to reset pin. Remove to flash over USB, or manually enter bootloader on upload.
//  2.use an external USB-TTL on the RX/TX lines directly to implement USB 

#include <Arduino.h>
#include "hci.h"
#include "arm.h"
#include "drive.h"

#define ID_PHRASE F("MICROSCRIBE SERIAL INTERFACE")
//#define AUTOCONNECT       // define this if you want the interface to automatically connect on startup without needing to send connect command


void printXYZ(arm_rec);
void printendpoints(arm_rec);
void printencoder(arm_rec);
void printangles(arm_rec);
void home(arm_rec);
void print_param_block(arm_rec);
void print_encoder_max(hci_rec);
void print_hci_strings(arm_rec);

arm_rec arm;
long baud = 19200L;   // <<<<<<<<< HCI has an "autosynch" function, but it seems to only work with 19200
int port = 1;
byte buffer[1]; 

// Arduino byte commands
#define IDN           'i'
#define CONT          'c'
#define DISCONT       'd'
#define QUERY         'q'
#define PRINTOFF      'n'
#define HOME          'h'

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
  Serial.begin(115200);
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
  // get update from Microscribe
  if(arm_connected == TRUE) result = arm_stylus_6DOF_update(&arm);

  // Always check for a byte command and process it
  if(Serial.available())
  {
    Serial.readBytes(buffer, 1);
    switch ((int)buffer[0])
    {
      case IDN:
        // identify Arduino
        Serial.println(ID_PHRASE);
        break;
      case CONT:
        // default continuous mode, report XYZ constantly
        printMode = PRINT_CONT;
        Serial.println(F("CONTINUOUS MODE"));
        break;
      case PRINTOFF:
        printMode = PRINT_NONE;
        Serial.println(F("PRINTING OFF"));
        break;
      case DISCONT:
      // discontinuous mode
        printMode = PRINT_DISCONT;
        Serial.println(F("DISCONTINUOUS MODE"));
        break;
      case QUERY:
      // query mode. Software flow control
        printMode = PRINT_QUERY;
        Serial.println(F("QUERY MODE"));
        break;
      case HOME:
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
      // Disconnect arm
        arm_disconnect(&arm);
        Serial.println(F("DISCONNECTED"));
        arm_connected = FALSE;      
        digitalWrite(13, arm_connected);
        break;
      case 'r':
      // Connect/ reconnect arm
        arm_connect(&arm, port, baud);
        Serial.println(F("READY"));
        arm_connected = TRUE;
        digitalWrite(13, arm_connected);
        break;
      case 'g':
      // print calibration constants
        if(arm_connected == TRUE)
        {
          print_param_block(arm);
          print_encoder_max(arm.hci);          
        }
        break;
      case 't':
      // print angles
        if(arm_connected == TRUE)
        {
          // printencoder(arm);
          printangles(arm);
        }
        break;
      case 'e':
      // print endpoints
        if(arm_connected == TRUE)
        {
          printendpoints(arm);
        }
        break;
      case 's':
        //print_hci_strings(arm);         // This locks the system up if we uncomment it. (?? WHY ??)
        break;
      default:
        break;
    }
  }

	// print coordinates to Serial Monitor on loop if requested
  if(printMode == PRINT_CONT)
  {
    printXYZ(arm);
    delay(70); // rudimentary spam limiter. Use timer interrupts for serious applications
  }
}

void printXYZ(arm_rec arm)
{
  Serial.print("XYZ,");
  Serial.print(arm.stylus_tip.x,4);
  Serial.print(",");
  Serial.print(arm.stylus_tip.y,4);
  Serial.print(",");
  Serial.print(arm.stylus_tip.z,4);
  Serial.print(",");
  Serial.print(arm.stylus_dir.x,4);
  Serial.print(",");
  Serial.print(arm.stylus_dir.y,4);
  Serial.print(",");
  Serial.print(arm.stylus_dir.z,4);
  Serial.println("");
}

void printendpoints(arm_rec arm)
{
  // Serial.println("LINK ENDPOINTS: ");
  int i = 0;

  Serial.print("END,");
  Serial.print(arm.endpoint[i].x);
  Serial.print(",");
  Serial.print(arm.endpoint[i].y);
  Serial.print(",");
  Serial.print(arm.endpoint[i++].z);
  Serial.print(",");

  Serial.print(arm.endpoint[i].x);
  Serial.print(",");
  Serial.print(arm.endpoint[i].y);
  Serial.print(",");
  Serial.print(arm.endpoint[i++].z);
  Serial.print(",");

  Serial.print(arm.endpoint[i].x);
  Serial.print(",");
  Serial.print(arm.endpoint[i].y);
  Serial.print(",");
  Serial.print(arm.endpoint[i++].z);
  Serial.print(",");

  Serial.print(arm.endpoint[i].x);
  Serial.print(",");
  Serial.print(arm.endpoint[i].y);
  Serial.print(",");
  Serial.print(arm.endpoint[i++].z);
  Serial.print(",");

  Serial.print(arm.endpoint[i].x);
  Serial.print(",");
  Serial.print(arm.endpoint[i].y);
  Serial.print(",");
  Serial.print(arm.endpoint[i++].z);
  Serial.print(",");

  Serial.print(arm.endpoint[i].x);
  Serial.print(",");
  Serial.print(arm.endpoint[i].y);
  Serial.print(",");
  Serial.println(arm.endpoint[i].z);
}

void printencoder(arm_rec arm)
{
  int i = 0;
  Serial.print("ENC,");
  Serial.print(arm.hci.encoder[i++]);
  Serial.print(",");
  Serial.print(arm.hci.encoder[i++]);
  Serial.print(",");
  Serial.print(arm.hci.encoder[i++]);
  Serial.print(",");
  Serial.print(arm.hci.encoder[i++]);
  Serial.print(",");
  Serial.println(arm.hci.encoder[i++]);

}

void printangles(arm_rec arm)
{
  // Serial.println("JOINT ANGLES: ");
  int i = 0;

  // This block with repeated print works
  Serial.print("THETA,");
  Serial.print(arm.joint_rad[i++], 5);
  Serial.print(",");
  Serial.print(arm.joint_rad[i++], 5);
  Serial.print(",");
  Serial.print(arm.joint_rad[i++], 5);
  Serial.print(",");
  Serial.print(arm.joint_rad[i++], 5);
  Serial.print(",");
  Serial.println(arm.joint_rad[i++], 5);

  // The exact same thing doesn't work in a for loop. 
  // for(i = 0; i < 6; i++)
  // {
  //   Serial.print("X"); Serial.print(i); Serial.print(": "); Serial.println(arm.endpoint[i].x);
  //   Serial.print("Y"); Serial.print(i); Serial.print(": "); Serial.println(arm.endpoint[i].y);
  //   Serial.print("Z"); Serial.print(i); Serial.print(": "); Serial.println(arm.endpoint[i].z);
  //   Serial.print("Theta"); Serial.print(i); Serial.print(": ");  Serial.println(arm.joint_deg[i]);
  // }
}

void home(arm_rec arm)
{
  arm_home_pos(&arm);
  Serial.println("RESET HOME LOCATION");
}


void print_encoder_max(hci_rec hci)
{
  Serial.println("ENCODER MAX COUNTS:");
  int i = 0;
  Serial.println(hci.max_encoder[i++]);
  Serial.println(hci.max_encoder[i++]);
  Serial.println(hci.max_encoder[i++]);
  Serial.println(hci.max_encoder[i++]);
  Serial.println(hci.max_encoder[i++]);
  Serial.println(hci.max_encoder[i++]);

  // It does not like this if it has a for loop in it.
  // for(i = 0; i < NUM_DOF; i++)
  // {
  //   Serial.println(hci.max_encoder[i]);
  // }

  Serial.println("");
}

void print_param_block(arm_rec arm)
{
  Serial.println("PARAM BLOCK:");
  int i;
  for(i = 0; i< PARAM_BLOCK_SIZE; i++)
  {
    Serial.print(arm.param_block[i]);
    Serial.println("");
  }
  Serial.println("");
  Serial.println("OFFSETS D:");
  for(i = 0; i<NUM_DOF; i++)
  {
    Serial.print(arm.D[i],6);
    Serial.println("");
  }
  Serial.println("");
    Serial.println("OFFSETS A:");
  for(i = 0; i<NUM_DOF; i++)
  {
    Serial.print(arm.A[i],6);
    Serial.println("");
  }
  Serial.println("");
  Serial.println("SKEW ALPHA:");
  for(i = 0; i<NUM_DOF; i++)
  {
    Serial.print(arm.ALPHA[i],6);
    Serial.println("");
  }
  Serial.println("");
  Serial.println("BETA: ");
  Serial.println(arm.BETA,6);
  Serial.println("");
  Serial.println("HOME ENCODER POSITIONS:");
  for(i = 0; i<NUM_ENCODERS; i++)
  {
    Serial.print(arm.hci.home_pos[i]);
    Serial.println("");
  }
  Serial.println("");
  Serial.println("HOME ENCODER REFERENCE:");
  for(i = 0; i<NUM_ENCODERS; i++)
  {
    Serial.print(arm.hci.home_ref[i]);
    Serial.println("");
  }
  Serial.println("");
}


// Print HCI information 
// This function will run and retrieve the correct data, 
// however other functions break if this code is included in the main loop switch case
// e.g. cannot print XYZ data
void print_hci_strings(arm_rec arm)
{
  hci_get_strings(&arm.hci);    
  Serial.println(F("HCI STRINGS: ")); 
  Serial.print("SERIAL NUMBER: ");
  Serial.println(arm.hci.serial_number);
  Serial.print("PRODUCT NAME: ");
  Serial.println(arm.hci.product_name);
  Serial.print("PRODUCT ID: ");
  Serial.println(arm.hci.product_id);
  Serial.print("MODEL NAME: ");
  Serial.println(arm.hci.model_name);
  Serial.print("COMMENT: ");
  Serial.println(arm.hci.comment);
  Serial.print("PARAM FORMAT: ");
  Serial.println(arm.hci.param_format);
  Serial.print("VERSION: ");
  Serial.println(arm.hci.version);
  Serial.println("");
}