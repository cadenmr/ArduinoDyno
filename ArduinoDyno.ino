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
      shaftRpmTimeout = serialDataToUnsignedLong();
      sendTelemetry(true, false);
      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 4:     // set shaft rpm target (16 bit) (unsigned)
      shaftRpmDesired = serialDataToUnsignedInt();
      sendTelemetry(true, false);
      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 5:     // set inlet kp (32 bit) (double)
      inletPidKp = serialDataToDouble();
      sendTelemetry(true, false);
      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 6:     // set inlet ki (32 bit) (double)
      inletPidKi = serialDataToDouble();
      sendTelemetry(true, false);
      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 7:     // set inlet kd (32 bit (double)
      inletPidKd = serialDataToDouble();
      sendTelemetry(true, false);
      break;

    case 8:     // set inlet min duty (8 bit) (byte)

      byte tempInletMinDuty = serialDataToByte();

      if (tempInletMinDuty > 100) {
        sendTelemetry(false, true);
      } else {
        inletMinDuty = tempInletMinDuty;
        sendTelemetry(true, false);
      }

      break;

    case 9:     // set inlet max duty (8 bit) (byte)

      byte tempInletMaxDuty = serialDataToByte();

      if (tempInletMaxDuty > 100) {
        sendTelemetry(false, true);
      } else {
        inletMaxDuty = tempInletMaxDuty;
        sendTelemetry(true, false);
      }

      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 10:     // enable/disable inlet override (8 bit) (boolean)
      inletOverrideActive = serialDataToBoolean();
      sendTelemetry(true, false);
      break;

    case 11:    // set inlet override duty cycle (8 bit) (byte)
      
      if (inletOverrideActive) {
        inletDutyDesired = serialDataToByte();
        sendTelemetry(true, false);
      } else {
        sendTelemetry(false, true);
      }

      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 12:    // set outlet kp (32 bit) (double)
      outletPidKp = serialDataToDouble();
      sendTelemetry(true, false);
      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 13:    // set outlet ki (32 bit) (double)
      outletPidKi = serialDataToDouble();
      sendTelemetry(true, false);
      break;

    // TODO: UPDATE PID TOO WHEN THIS HAPPENS
    case 14:    // set outlet kd (32 bit) (double)
      outletPidKd = serialDataToDouble();
      sendTelemetry(true, false);
      break;

    case 15:    // set outlet min duty (8 bit) (byte)

      byte tempOutletMinDuty = serialDataToByte();

      if (tempOutletMinDuty > 100) {
        sendTelemetry(false, true);
      } else {
        outletMinDuty = tempOutletMinDuty;
        sendTelemetry(true, false);
      }

      break;

    case 16:    // set outlet max duty (8 bit) (byte)

      byte tempOutletMaxDuty = serialDataToByte();

      if (tempOutletMaxDuty > 100) {
        sendTelemetry(false, true);
      } else {
        outletMaxDuty = tempOutletMaxDuty;
        sendTelemetry(true, false);
      }

      break;

    case 17:    // set outlet target temperature (8 bit) (byte)

      outletMaxTemperature = serialDataToByte();
      sendTelemetry(true, false);
      break;
    
    case 18:    // enable/disable outlet override (8 bit) (boolean)

      outletOverrideActive = serialDataToBoolean();
      sendTelemetry(true, false);
      break;

    case 19:    // set outlet override duty cycle (8 bit) (byte)

      if (outletOverrideActive) {
        outletDutyDesired = serialDataToByte();
        sendTelemetry(true, false);
      } else {
        sendTelemetry(false, true);
      }

      break;

    // TODO: UPDATE LOAD CELL WHEN THIS CHANGES
    case 20:    // set load cell resolution (8 bit) (byte)

      byte tempResolution = serialDataToByte();

      if (tempResolution == 64 || tempResolution == 128 ) {

        loadCellResolution = tempResolution;

      } else {
        sendTelemetry(false, true);
      }

      sendTelemetry(true, false);
      break;

    // TODO: UPDATE LOAD CELL WHEN THIS HAPPENS
    case 21:    // set load cell factor (16 bit) (unsigned int)

      loadCellFactor = serialDataToUnsignedInt();

      sendTelemetry(true, false);
      break;

    case 22:  // telemetry request (no data)
      sendTelemetry(false, false);
      break;

  }

}

void sendTelemetry(bool pass, bool fail) {

}

// returns true if we had a successful calculation, false if not
bool calculateRpm() {

  if (!shaftRpmUpdateReady) { return false; }   // immediately return false if we're not ready to update

  shaftRpmTimeDelta = shaftHallMicrosCurrent - shaftHallMicrosLast;
  shaftRpmCurrent = 60000000 / shaftRpmTimeDelta;
  return true;

}

void hallInterrupt() {

  shaftHallMicrosLast = shaftHallMicrosCurrent;
  shaftHallMicrosCurrent = micros();

  // check if both values are loaded
  if (shaftHallMicrosLast == 0 || shaftHallMicrosCurrent == 0) {

    // we are not ready to recalculate if we have one of these equaling zero
    shaftRpmUpdateReady = false;

  } else {  // if both values are loaded

    // make sure the current micros count is higher than the last micros count
    // this prevents a bug where micros() will overflow about every ~70 minutes
    if (shaftHallMicrosLast > shaftHallMicrosCurrent) {
      shaftRpmUpdateReady = false;
    } else {
      // if both checks pass, we are good to calculate the RPM
      shaftRpmUpdateReady = true;
    }
  }

}

bool shaftRpmOverspeed() {

  if (shaftRpmCurrent > shaftRpmMaximum) {
    return true;
  } else {
    return false;
  }

}
