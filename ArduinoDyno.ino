#include <ArduPID.h>

// Pin Config
#define                 SHAFT_HALL_PICKUP_PIN             2
#define                 INLET_SERVO_PIN                   5
#define                 OUTLET_SERVO_PIN                  6
#define                 OUTLET_TEMP_ANALOG_PIN            0
#define                 LOADCELL_CLOCK_PIN                7
#define                 LOADCELL_DATA_PIN                 8

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
unsigned int            shaftRpmCurrent         = 0;
unsigned int            shaftRpmDesired         = shaftRpmMaximum;
volatile unsigned long  shaftHallMicrosCurrent  = 0;
volatile unsigned long  shaftHallMicrosLast     = 0;
volatile boolean        shaftRpmUpdateReady     = false;
unsigned long           shaftRpmTimeDelta       = 0;
// unsigned long        shaftRpmReadings;       // TODO: Implement this

// Inlet Valve Control
byte                    inletDutyCurrent        = 0;
byte                    inletDutyDesired        = inletMinDuty;
boolean                 inletOverrideActive     = false;

// Outlet Valve Control
byte                    outletDutyCurrent       = 0;
byte                    outletDutyDesired       = outletMinDuty;
boolean                 outletOverrideActive    = false;
byte                    outletTemperatureCurrent= 0;

// Measurement
unsigned int            loadCellForceCurrent    = 0;

void setup() {
  
  Serial.begin(115200);

  while (!Serial.available()) { ; } // wait for connection
  startupDataParse();               // parse all configuration info from host PC

  attachInterrupt(digitalPinToInterrupt(SHAFT_HALL_PICKUP_PIN), hallInterrupt, FALLING);

}

void loop() {

  // Check for serial comms + respond  + perform any special request + update variables

  // Read/compute sensor data

  // Check for failure cases

  // Recalculate pid

  // 

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
