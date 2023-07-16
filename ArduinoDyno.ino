// Pin Config
#define                 SHAFT_HALL_PICKUP_PIN             2
#define                 WATER_PUMP_SPEED_PIN              3   // future use
#define                 OUTLET_TEMP_PIN                   4
#define                 INLET_SERVO_PIN                   5
#define                 OUTLET_SERVO_PIN                  6
#define                 LOADCELL_CLOCK_PIN                7
#define                 LOADCELL_DATA_PIN                 8

// internal data formatting config
#define                 INCOMING_PACKET_SIZE_BYTES        6

// internal PID config
#define                 LOADCELL_SAMPLE_RATE_MICROS       30000 //20000 // 14286 //12821 // 78Hz load cell sample rate
#define                 OUTLET_TEMP_SAMPLE_RATE_MICROS    0 // TODO: SET ME
#define                 PID_SAMPLE_RATE_MICROS            10000 // 100Hz pid rate

// Shaft Speed config
double                  shaftRpmMaximum                   = 9000;
double                  shaftRpmMaximumHyst               = 300;
double                  shaftRpmRounding                  = 50;
double                  shaftRpmRunningThreshold          = shaftRpmMaximum;
unsigned long           shaftRpmTimeout                   = 2000000;
// Inlet Valve Config
double                  inletPidKp                        = 0;
double                  inletPidKi                        = 0;
double                  inletPidKd                        = 0;
double                  inletPidIMin                      = 0;
double                  inletPidIMax                      = 0;
byte                    inletMinDuty                      = 10;
byte                    inletMaxDuty                      = 90;
// Outlet Valve Config
double                  outletPidKp                       = 0;
double                  outletPidKi                       = 0;
double                  outletPidKd                       = 0;
double                  outletPidIMin                     = 0;
double                  outletPidIMax                     = 0;
byte                    outletMinDuty                     = 10;
byte                    outletMaxDuty                     = 90;
double                  outletTemperatureDesired          = 50;
// Measurement Config
byte                    loadCellAvgConst                  = 3;
byte                    loadCellResolution                = 128;
unsigned long           loadCellOffset                    = 0;
double                  loadCellScale                     = 0;
// Shaft Speed Control
double                  shaftRpmCurrent                   = 9999;
double                  shaftRpmCurrentRounded            = 9999;
double                  shaftRpmDesired                   = shaftRpmMaximum;
volatile unsigned long  shaftHallMicrosCurrent            = 0;
volatile unsigned long  shaftHallMicrosLast               = 0;
volatile bool           shaftRpmUpdateReady               = false;
// Inlet Valve Control
double                  inletDutyDesired                  = inletMinDuty;
bool                    inletOverrideActive               = false;
// Outlet Valve Control
double                  outletDutyDesired                 = outletMinDuty;
bool                    outletOverrideActive              = false;
double                  outletTemperatureCurrent         = 255.99;
// Load Measurement
double                  loadCellForceCurrent              = 255.99;
// Internal Objects
bool configured = false;
bool critical = true;

unsigned long currentMicros = micros();
unsigned long lastMicros = 0;
unsigned long lastLoopTimeDelta = 0;

unsigned long lastLoadCellMicros = 0;
unsigned long lastPidMicros = 0;

bool microsOverflowed = false;

// imports
#include <ArduPID.h>
#include <HX711.h>  // use robtillaart library
#include <OneWire.h>
#include <DallasTemperature.h>

ArduPID inletController;
ArduPID outletController;

HX711 torqueSensor;

OneWire oneWire(OUTLET_TEMP_PIN);
DallasTemperature dallasTempSensors(&oneWire);
DeviceAddress outletTempSensorAddress;

void setup() {
  
  // initialize comms 
  Serial.begin(115200);

  // initialize all sensors
  torqueSensor.begin(LOADCELL_DATA_PIN, LOADCELL_CLOCK_PIN);
  torqueSensor.set_raw_mode();
  // TODO: handle sensor not found error
  dallasTempSensors.begin();
  if (!dallasTempSensors.getAddress(outletTempSensorAddress, 0)) {
    // TODO: handle sensor not found error
  }

  // initialize outputs
  setInlet(inletMinDuty);
  setOutlet(outletMinDuty);

  // initialize PID
  inletController.begin(&shaftRpmCurrent, &inletDutyDesired, &shaftRpmDesired, inletPidKp, inletPidKi, inletPidKd);
  outletController.begin(&outletTemperatureCurrent, &outletDutyDesired, &outletTemperatureDesired, outletPidKp, outletPidKi, outletPidKd);
  inletController.setOutputLimits(inletMinDuty, inletMaxDuty);
  outletController.setOutputLimits(outletMinDuty, outletMaxDuty);
  inletController.setWindUpLimits(inletPidIMin, inletPidIMax);
  outletController.setWindUpLimits(outletPidIMin, outletPidIMax);
  inletController.reverse();
  outletController.reverse();

  // load all config values from pc until we are ready to start up
  while (!configured) {

    if (Serial.available() >= INCOMING_PACKET_SIZE_BYTES) { parseIncomingSerial(); }

  }

  critical = false;

  // start reading RPM
  attachInterrupt(digitalPinToInterrupt(SHAFT_HALL_PICKUP_PIN), hallInterrupt, FALLING);

}

void loop() {

  // Read/compute sensor data -- FUNCTIONS DISABLED FOR TESTING
  calculateRpm();
  // Temperature
  // dallasTempSensors.requestTemperatures();
  // outletTemperatureCurrent = dallasTempSensors.getTempC(outletTempSensorAddress);
  // if (outletTemperatureCurrent == DEVICE_DISCONNECTED_C) {
  //   // TODO: handle this error
  // }

  // Load Cell

  if ((micros() >= (lastLoadCellMicros + LOADCELL_SAMPLE_RATE_MICROS - lastLoopTimeDelta - 100))) {

    loadCellForceCurrent = torqueSensor.get_units();
    lastLoadCellMicros = micros();

  }

  // Recalculate pid (only works if enabled)
  if (!inletOverrideActive || !outletOverrideActive && !microsOverflowed) {

    if (micros() >= (lastPidMicros + PID_SAMPLE_RATE_MICROS - lastLoopTimeDelta - 100)) {
      inletController.compute();
      outletController.compute();
      lastPidMicros = micros();
    }

  }

  // TODO: Check for failure cases

  // Set outputs
  setInlet(inletDutyDesired);
  setOutlet(outletDutyDesired);

  // Check for serial comms + respond  + perform any special request + update variables
  if (Serial.available() >= INCOMING_PACKET_SIZE_BYTES) { parseIncomingSerial(); }

  // calculate loop timing
  lastMicros = currentMicros;
  currentMicros = micros();
  lastLoopTimeDelta = currentMicros - lastMicros;

  // flag overflow of micros()
  if (lastMicros > currentMicros) {
    microsOverflowed = true;
  } else {
    microsOverflowed = false;
  }

}

void parseIncomingSerial() {

  // first byte is the command byte
  byte commandByte = Serial.read();

  // remaining 5 bytes on buffer are the data bytes
  // what we do with them depends on the command

  if (commandByte == 0x00) {          // set shaft rpm maximum

    shaftRpmMaximum = serialDataToUnsignedInt();
    sendTelemetry(true, false);

  } else if (commandByte == 0x01) {   // set shaft rpm maximum hysteresis

    shaftRpmMaximumHyst = serialDataToUnsignedInt();
    sendTelemetry(true, false);

  } else if (commandByte == 0x02) {   // set shaft rpm rounding

    shaftRpmRounding = serialDataToByte();
    sendTelemetry(true, false);

  } else if (commandByte == 0x03) {   // set shaft rpm timeout

    shaftRpmTimeout = serialDataToUnsignedLong();
    sendTelemetry(true, false);

  } else if (commandByte == 0x04) {   // set shaft rpm target

    shaftRpmDesired = serialDataToUnsignedInt();
    sendTelemetry(true, false);

  } else if (commandByte == 0x05) {   // set inlet kp

    inletPidKp = serialDataToDouble();
    inletController.setCoefficients(inletPidKp, inletPidKi, inletPidKd);
    sendTelemetry(true, false);

  } else if (commandByte == 0x06) {   // set inlet ki

    inletPidKi = serialDataToDouble();
    inletController.setCoefficients(inletPidKp, inletPidKi, inletPidKd);
    sendTelemetry(true, false);

  } else if (commandByte == 0x07) {   // set inlet kd

    inletPidKd = serialDataToDouble();
    inletController.setCoefficients(inletPidKp, inletPidKi, inletPidKd);
    sendTelemetry(true, false);

  } else if (commandByte == 0x08) {   // set inlet integrator limit minimum

    inletPidIMin = serialDataToDouble();
    inletController.setWindUpLimits(inletPidIMin, inletPidIMax);
    sendTelemetry(true, false);

  } else if (commandByte == 0x09) {   // set inlet integrator limit maximum

    inletPidIMax = serialDataToDouble();
    inletController.setWindUpLimits(inletPidIMin, inletPidIMax);
    sendTelemetry(true, false);

  } else if (commandByte == 0x0A) {   // set inlet min duty

    byte tempInletMinDuty = serialDataToByte();

    if (tempInletMinDuty > 100) {
      sendTelemetry(false, true);
    } else {
      inletMinDuty = tempInletMinDuty;
      inletController.setOutputLimits((double)inletMinDuty, (double)inletMaxDuty);
      sendTelemetry(true, false);
    }

  } else if (commandByte == 0x0B) {   // set inlet max duty

    byte tempInletMaxDuty = serialDataToByte();

    if (tempInletMaxDuty > 100) {
      sendTelemetry(false, true);
    } else {
      inletMaxDuty = tempInletMaxDuty;
      inletController.setOutputLimits((double)inletMinDuty, (double)inletMaxDuty);
      sendTelemetry(true, false);
    }

  } else if (commandByte == 0x0C) {   // enable/disable inlet override

    inletOverrideActive = serialDataToBoolean();
    
    if (inletOverrideActive) {
      inletController.stop();
    } else {
      inletController.start();
    }

    sendTelemetry(true, false);

  } else if (commandByte == 0x0D) {   // set inlet override duty cycle

    if (inletOverrideActive) {
      inletDutyDesired = serialDataToByte();
      sendTelemetry(true, false);
    } else {
      sendTelemetry(false, true);
    }

  } else if (commandByte == 0x0E) {   // set outlet kp

    outletPidKp = serialDataToDouble();
    outletController.setCoefficients(outletPidKp, outletPidKi, outletPidKd);
    sendTelemetry(true, false);

  } else if (commandByte == 0x0F) {   // set outlet ki

    outletPidKi = serialDataToDouble();
    outletController.setCoefficients(outletPidKp, outletPidKi, outletPidKd);
    sendTelemetry(true, false);

  } else if (commandByte == 0x10) {   // set outlet kd

    outletPidKd = serialDataToDouble();
    outletController.setCoefficients(outletPidKp, outletPidKi, outletPidKd);
    sendTelemetry(true, false);

  } else if (commandByte == 0x11) {   // set outlet integrator minimum

    outletPidIMin = serialDataToDouble();
    outletController.setWindUpLimits(outletPidIMin, outletPidIMax);
    sendTelemetry(true, false);

  } else if (commandByte == 0x12) {   // set outlet integrator maximum

  outletPidIMax = serialDataToDouble();
  outletController.setWindUpLimits(outletPidIMin, outletPidIMax);
  sendTelemetry(true, false);


  } else if (commandByte == 0x13) {   // set outlet min duty

    byte tempOutletMinDuty = serialDataToByte();

    if (tempOutletMinDuty > 100) {
      sendTelemetry(false, true);
    } else {
      outletMinDuty = tempOutletMinDuty;
      outletController.setOutputLimits(outletMinDuty, outletMaxDuty);
      sendTelemetry(true, false);
    }

  } else if (commandByte == 0x14) {   // set outlet max duty

    byte tempOutletMaxDuty = serialDataToByte();

    if (tempOutletMaxDuty > 100) {
      sendTelemetry(false, true);
    } else {
      outletMaxDuty = tempOutletMaxDuty;
      outletController.setOutputLimits(outletMinDuty, outletMaxDuty);
      sendTelemetry(true, false);
    }

  } else if (commandByte == 0x15) {   // set outlet target temperature

    outletTemperatureDesired = serialDataToDouble();
    sendTelemetry(true, false);

  } else if (commandByte == 0x16) {   // enable/disable outlet override

    outletOverrideActive = serialDataToBoolean();

    if (outletOverrideActive) {
      outletController.stop();
    } else {
      outletController.start();
    }

    sendTelemetry(true, false);

  } else if (commandByte == 0x17) {   // set outlet override duty cycle

    if (outletOverrideActive) {
      outletDutyDesired = serialDataToByte();
      sendTelemetry(true, false);
    } else {
      sendTelemetry(false, true);
    }

  } else if (commandByte == 0x18) {   // set load cell resolution

    byte tempResolution = serialDataToByte();

    if (tempResolution == 64 || tempResolution == 128 ) {

      loadCellResolution = tempResolution;
      torqueSensor.set_gain(loadCellResolution, false);
      sendTelemetry(true, false);

    } else {
      sendTelemetry(false, true);
    }

  } else if (commandByte == 0x19) {   // set load cell offset

    loadCellOffset = serialDataToUnsignedLong();
    torqueSensor.set_offset(loadCellOffset);
    sendTelemetry(true, false);

  } else if (commandByte == 0x1A) {   // set load cell scale

    loadCellScale = serialDataToDouble();
    torqueSensor.set_scale(loadCellScale);
    sendTelemetry(true, false);

  } else if (commandByte == 0x1B) {   // telemetry request (no data)

    shredSerialData(5);
    sendTelemetry(false, false);

  } else if (commandByte == 0x1C) {   // set configured bit

    shredSerialData(5);
    configured = true;
    sendTelemetry(true, false);

  }

}

// 21 bytes
void sendTelemetry(bool pass, bool fail) {

  // status
  // pass/fail
  byte status = 0b00000000;
  if (pass) {
    bitSet(status, 5);
  } 
  
  if (fail) {
    bitSet(status, 6);
  }

  if (critical) {
    bitSet(status, 4);
  }

  // TODO: add error telemetry here

  if (!configured) {
    status += 0x01;
  }
  
  Serial.write(status);

  // shaft rpm
  byte * rpmPtr = (byte *)&shaftRpmCurrent;
  Serial.write(rpmPtr, 4);

  // measured load cell force
  byte * forcePtr = (byte *)&loadCellForceCurrent;
  Serial.write(forcePtr, 4);

  // inlet duty cycle
  byte * inletDutyPtr = (byte *)&inletDutyDesired;
  Serial.write(inletDutyPtr, 4);

  // outlet duty cycle
  byte * outletDutyPtr = (byte *)&outletDutyDesired;
  Serial.write(outletDutyPtr, 4);

  // outlet water temperatur
  byte * outletTempCurrentPtr = (byte *)&outletTemperatureCurrent;
  Serial.write(outletTempCurrentPtr, 4);

}

// returns true if we had a successful calculation, false if not
bool calculateRpm() {

  // timeout check
  if (shaftRpmUpdateReady && ((micros() - shaftHallMicrosCurrent) > shaftRpmTimeout)) {
    shaftRpmUpdateReady = false;
    shaftRpmCurrent = 0;
    shaftRpmCurrentRounded = 0;
  }

  if (!shaftRpmUpdateReady) { return false; }   // immediately return false if we're not ready to update

  // update rpm
  shaftRpmCurrent = 60000000 / (shaftHallMicrosCurrent - shaftHallMicrosLast);
  // shaftRpmCurrentRounded = (shaftRpmCurrent / shaftRpmRounding + (shaftRpmCurrent % shaftRpmRounding > 2)) * shaftRpmRounding;

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

void setInlet(double duty) {

  int setDuty = (int)duty;
  if (setDuty < inletMinDuty ) { setDuty = inletMinDuty; }
  if (setDuty > inletMaxDuty) { setDuty = inletMaxDuty; }

  setDuty = map(setDuty, 0, 100, 0, 255);
  analogWrite(INLET_SERVO_PIN, setDuty);

}

void setOutlet(double duty) {

  int setDuty = (int)duty;
  if (setDuty < outletMinDuty ) { setDuty = outletMinDuty; }
  if (setDuty > outletMaxDuty) { setDuty = outletMaxDuty; }

  setDuty = map(setDuty, 0, 100, 0, 255);
  analogWrite(OUTLET_SERVO_PIN, setDuty);

}
