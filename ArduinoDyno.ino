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

// Shaft Speed config
unsigned int            shaftRpmMaximum                   = 9000;
unsigned int            shaftRpmMaximumHyst               = 300;
// byte                    shaftRpmAvgConst                  = 3;
byte                    shaftRpmRounding                  = 50;
unsigned int            shaftRpmRunningThreshold          = shaftRpmMaximum;
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
byte                    outletTemperatureDesired              = 50;

// Measurement Config
byte                    loadCellAvgConst                  = 3;
byte                    loadCellResolution                = 128;
unsigned long           loadCellOffset                    = 0;
double                  loadCellScale                     = 0;

// Shaft Speed Control
unsigned int            shaftRpmCurrent                   = 256;
unsigned int            shaftRpmCurrentRounded            = 256;
byte * rpmPtr                                             = (byte *) &shaftRpmCurrentRounded;
unsigned int            shaftRpmDesired                   = shaftRpmMaximum;
volatile unsigned long  shaftHallMicrosCurrent            = 0;
volatile unsigned long  shaftHallMicrosLast               = 0;
volatile bool           shaftRpmUpdateReady               = false;
// unsigned long           shaftRpmTimeDelta                 = 0;
// unsigned long        shaftRpmReadings;                 // TODO: Implement this

// Inlet Valve Control
// byte                    inletDutyCurrent                  = 0;
byte                    inletDutyDesired                  = inletMinDuty;
bool                    inletOverrideActive               = false;

// Outlet Valve Control
// byte                    outletDutyCurrent                 = 0;
byte                    outletDutyDesired                 = outletMinDuty;
bool                    outletOverrideActive              = false;
float                    outletTemperatureCurrent          = 255.99;
byte * outletTempCurrentPtr                               = (byte *) &outletTemperatureCurrent;

// Load Measurement
double                  loadCellForceCurrent              = 255.99;
byte * forcePtr = (byte *) &loadCellForceCurrent;

// Internal Objects
bool configured = false;

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
  // TODO: handle sensor not found error
  dallasTempSensors.begin();
  if (!dallasTempSensors.getAddress(outletTempSensorAddress, 0)) {
    // TODO: handle sensor not found error
  }

  // initialize outputs
  analogWrite(INLET_SERVO_PIN, inletMinDuty);
  analogWrite(OUTLET_SERVO_PIN, outletMinDuty);

  // initialize PID
  inletController.begin((double*)&shaftRpmCurrent, (double*)&inletDutyDesired, (double*)&shaftRpmDesired, inletPidKp, inletPidKi, inletPidKd);
  outletController.begin((double*)&outletTemperatureCurrent, (double*)&outletDutyDesired, (double*)&outletTemperatureDesired, outletPidKp, outletPidKi, outletPidKd);
  inletController.reverse();
  outletController.reverse();

  // load all config values from pc until we are ready to start up
  while (!configured) {

    if (Serial.available() >= INCOMING_PACKET_SIZE_BYTES) { parseIncomingSerial(); }

  }

  // start reading RPM
  attachInterrupt(digitalPinToInterrupt(SHAFT_HALL_PICKUP_PIN), hallInterrupt, FALLING);

}

void loop() {

  // Check for serial comms + respond  + perform any special request + update variables
  if (Serial.available() >= INCOMING_PACKET_SIZE_BYTES) { parseIncomingSerial(); }

  // Read/compute sensor data
  calculateRpm();

  // Temperature
  dallasTempSensors.requestTemperatures();
  outletTemperatureCurrent = dallasTempSensors.getTempC(outletTempSensorAddress);
  if (outletTemperatureCurrent == DEVICE_DISCONNECTED_C) {
    // TODO: handle this error
  }

  // Load Cell
  if (torqueSensor.wait_ready_timeout(6)) {
    loadCellForceCurrent = torqueSensor.get_units(3);   // todo: see if this is a good value to avg
  } else {  // sensor not found
    loadCellForceCurrent = 0;
    // TODO: error handling
  }

  // Check for failure cases

  // Recalculate pid

  if (!inletOverrideActive) {

  }

  if (!outletOverrideActive) {

  }

  // Set outputs

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

    outletTemperatureDesired = serialDataToByte();
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

// 10 bytes
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
  // TODO: add error telemetry here

  if (!configured) {
    status += 0x01;
  }
  
  Serial.write(status);

  // shaft rpm
  Serial.write(rpmPtr, 2);

  // measured load cell force
  Serial.write(forcePtr, 4);

  // inlet duty cycle
  Serial.write(inletDutyDesired);

  // outlet duty cycle
  Serial.write(outletDutyDesired);

  // outlet water temperatur
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
  shaftRpmCurrentRounded = (shaftRpmCurrent / shaftRpmRounding + (shaftRpmCurrent % shaftRpmRounding > 2)) * shaftRpmRounding;

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

void setInlet(int duty) {

  int setDuty = map(duty, 0, 100, 0, 255);

  if (setDuty < inletMinDuty ) {

    setDuty = inletMinDuty;

  }

  if (setDuty > inletMaxDuty) {

    setDuty = inletMaxDuty;

  }

  analogWrite(INLET_SERVO_PIN, setDuty);

}

void setOutlet(int duty) {

  int setDuty = map(duty, 0, 100, 0, 255);

  if (setDuty < outletMinDuty ) {

    setDuty = outletMinDuty;

  }

  if (setDuty > outletMaxDuty) {

    setDuty = outletMaxDuty;

  }

  analogWrite(OUTLET_SERVO_PIN, setDuty);

}
