#include <ArduPID.h>

// Pin Config
#define                 SHAFT_HALL_PICKUP_PIN             2
#define                 INLET_SERVO_PIN                   5
#define                 OUTLET_SERVO_PIN                  6
#define                 OUTLET_TEMP_ANALOG_PIN            0
#define                 LOADCELL_CLOCK_PIN                7
#define                 LOADCELL_DATA_PIN                 8

// internal data formatting config
#define                 INCOMING_PACKET_SIZE_BYTES        6

// Shaft Speed config
unsigned int            shaftRpmMaximum                   = 9000;
unsigned int            shaftRpmMaximumHyst               = 300;
byte                    shaftRpmAvgConst                  = 3;
byte                    shaftRpmRounding                  = 50;
unsigned int            shaftRpmRunningThreshold          = shaftRpmMaximum;
unsigned long           shaftRpmTimeout                   = 2000000;

// Inlet Valve Config
double                  inletPidKp                        = 0;
double                  inletPidKi                        = 0;
double                  inletPidKd                        = 0;
byte                    inletMinDuty                      = 10;
byte                    inletMaxDuty                      = 90;

// Outlet Valve Config
double                  outletPidKp                       = 0;
double                  outletPidKi                       = 0;
double                  outletPidKd                       = 0;
byte                    outletMinDuty                     = 10;
byte                    outletMaxDuty                     = 90;
byte                    outletMaxTemperature              = 50;

// Measurement Config
byte                    loadCellAvgConst                  = 3;
byte                    loadCellResolution                = 128;
unsigned int            loadCellFactor                    = 0;

// Shaft Speed Control
unsigned int            shaftRpmCurrent                   = 0;
unsigned int            shaftRpmDesired                   = shaftRpmMaximum;
volatile unsigned long  shaftHallMicrosCurrent            = 0;
volatile unsigned long  shaftHallMicrosLast               = 0;
volatile bool           shaftRpmUpdateReady               = false;
unsigned long           shaftRpmTimeDelta                 = 0;
// unsigned long        shaftRpmReadings;                 // TODO: Implement this

// Inlet Valve Control
byte                    inletDutyCurrent                  = 0;
byte                    inletDutyDesired                  = inletMinDuty;
bool                    inletOverrideActive               = false;

// Outlet Valve Control
byte                    outletDutyCurrent                 = 0;
byte                    outletDutyDesired                 = outletMinDuty;
bool                    outletOverrideActive              = false;
byte                    outletTemperatureCurrent          = 0;

// Load Measurement
unsigned int            loadCellForceCurrent              = 0;

void setup() {
  
  Serial.begin(115200);

  while (!Serial) { ; } // wait for connection

  attachInterrupt(digitalPinToInterrupt(SHAFT_HALL_PICKUP_PIN), hallInterrupt, FALLING);

}

void loop() {

  // Check for serial comms + respond  + perform any special request + update variables

  if (Serial.available() >= INCOMING_PACKET_SIZE_BYTES) { parseIncomingSerial(); }

  // Read/compute sensor data

  // Check for failure cases

  // Recalculate pid

  // 

}

void parseIncomingSerial() {

  // first byte is the command byte
  byte commandByte = Serial.read();

  // remaining 5 bytes on buffer are the data bytes
  // what we do with them depends on the command
  switch (commandByte) {

    case 0:     // set shaft RPM max (16 bit) (unsigned int)
      shaftRpmMaximum = serialDataToUnsignedInt();
      sendTelemetry(true, false);
      break;

    case 1:     // set shaft RPM max hysteresis (16 bit) (unsigned int)
      shaftRpmMaximumHyst = serialDataToUnsignedInt();
      sendTelemetry(true, false);
      break;

    case 2:      // set shaft RPM rounding (8 bit) (byte)
      shaftRpmRounding = serialDataToByte();
      sendTelemetry(true, false);
      break;

    case 3:     // set shaft rpm timeout (32 bit) (unsigned long)
      ;
      break;

    case 4:     // set shaft rpm target (16 bit) (unsigned)
      ;
      break;

    case 5:     // set inlet kp (32 bit) (double)
      ;
      break;

    case 6:     // set inlet ki (32 bit) (double)
      ;
      break;

    case 7:     // set inlet kd (32 bit (double)
      ;
      break;

    case 8:     // set inlet min duty (8 bit) (byte)
      ;
      break;

    case 9:     // set inlet max duty (8 bit) (byte)
      ;
      break;

    case 10:     // enable/disable inlet override (8 bit) (boolean)
      ;
      break;

    case 11:    // set inlet override duty cycle (8 bit) (byte)
      ;
      break;

    case 12:    // set outlet kp (32 bit) (double)
      ;
      break;

    case 13:    // set outlet ki (32 bit) (double)
      ;
      break;

    case 14:    // set outlet kd (32 bit) (double)
      ;
      break;

    case 15:    // set outlet min duty (8 bit) (byte)
      ;
      break;

    case 16:    // set outlet max duty (8 bit) (byte)
      ;
      break;

    case 17:    // set outlet target temperature (8 bit) (byte)
      ;
      break;
    
    case 18:    // enable/disable outlet override (8 bit) (boolean)
      ;
      break;

    case 19:    // set outlet override duty cycle (8 bit) (byte)
      ;
      break;

    case 20:    // set load cell resolution (8 bit) (byte)
      ;
      break;

    case 21:    // set load cell factor (16 bit) (unsigned int)
      ;
      break;

    case 22:  // telemetry request (no data)
      sendTelemetry(false, false);
      break;

  }

}

// just read and drop specified amount of data
void shredSerialData(unsigned int count) {

  for (int i = 0; i < count; i++) {
    Serial.read();
  }

}

// 8 bit - use last byte of inc. data
boolean serialDataToBoolean() {

  shredSerialData(4);   // shred all but last byte of data
  boolean returnData = Serial.read();
  return returnData;

}

// 8 bit - use last byte of inc. data
byte serialDataToByte() {

  shredSerialData(4);   // shred all but last byte of data
  byte returnData = Serial.read();
  return returnData;


}

// 16 bit - use last 2 bytes of inc. data
int serialDataToSignedInt() {

  int returnData;

  shredSerialData(3);   // shred all but last 2 bytes of data

  byte upper = Serial.read();
  byte lower = Serial.read();

  uint8_t * returnDataLower = (uint8_t *)&returnData;
  uint8_t * returnDataUpper = (uint8_t *)&returnData + 1;

  *returnDataUpper = upper;
  *returnDataLower = lower;

  return returnData;

}

// 16 bit - use last 2 bytes of inc. data
unsigned int serialDataToUnsignedInt() {

  unsigned int returnData;

  shredSerialData(3);

  byte upper = Serial.read();
  byte lower = Serial.read();

  uint8_t * returnDataLower = (uint8_t *)&returnData;
  uint8_t * returnDataUpper = (uint8_t *)&returnData + 1;

  *returnDataUpper = upper;
  *returnDataLower = lower;

  return returnData;

}

// 32 bit - use last 4 bytes of inc. data
double serialDataToDouble() {

  double returnData;

  shredSerialData(1);

  byte incD3 = Serial.read();
  byte incD2 = Serial.read();
  byte incD1 = Serial.read();
  byte incD0 = Serial.read();

  uint8_t * d0 = (uint8_t *)&returnData;
  uint8_t * d1 = (uint8_t *)&returnData + 1;
  uint8_t * d2 = (uint8_t *)&returnData + 2;
  uint8_t * d3 = (uint8_t *)&returnData + 3;

  *d3 = incD3;
  *d2 = incD2;
  *d1 = incD1;
  *d0 = incD0;

  return returnData;

}

void sendTelemetry(boolean pass, boolean fail) {

}

// returns true if we had a successful calculation, false if not
boolean calculateRpm() {

  if (!shaftRpmUpdateReady) { return false; }   // immediately return false if we're not ready to update

  shaftRpmTimeDelta = shaftHallMicrosCurrent - shaftHallMicrosLast;
  shaftRpmCurrent = 60000000 / shaftRpmTimeDelta;
  return true;

}

void hallInterrupt() {

  shaftHallMicrosLast = shaftHallMicrosCurrent;
  shaftHallMicrosCurrent = micros();

  if (shaftHallMicrosLast != 0 && shaftHallMicrosCurrent != 0) {
    shaftRpmUpdateReady = true;
  } else {
    shaftRpmUpdateReady = false;
  }

}

bool shaftRpmOverspeed() {

  if (shaftRpmCurrent > shaftRpmMaximum) {
    return true;
  } else {
    return false;
  }

}
