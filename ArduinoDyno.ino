// Pin Config
#define                 SHAFT_HALL_PICKUP_PIN             2
// #define                 WATER_PUMP_SPEED_PIN              3   // future use
#define                 OUTLET_TEMP_PIN                   4
#define                 INLET_SERVO_PIN                   6
#define                 OUTLET_SERVO_PIN                  5
#define                 LOADCELL_CLOCK_PIN                7
#define                 LOADCELL_DATA_PIN                 3

// internal data formatting config
#define                 SERIAL_RATE                       250000
#define                 INCOMING_PACKET_SIZE_BYTES        6

// sample rates
#define                 MAINLOOP_RATE_MICROS              5000    // 200hz master limiter
// load cell is sampled as data is available, non blocking
#define                 OUTLET_TEMP_SAMPLE_RATE_MICROS    1000000  // 5hz temp sensor rate
#define                 PID_SAMPLE_RATE_MICROS            10000   // 100hz pid rate

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
byte                    inletMinDuty                      = 0;
byte                    inletMaxDuty                      = 100;
// Outlet Valve Config
double                  outletPidKp                       = 0;
double                  outletPidKi                       = 0;
double                  outletPidKd                       = 0;
double                  outletPidIMin                     = 0;
double                  outletPidIMax                     = 0;
byte                    outletMinDuty                     = 0;
byte                    outletMaxDuty                     = 100;
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
double                  outletTemperatureCurrent         = 0;
// Load Measurement
long                    loadCellForceCurrent              = 256;
// Internal Objects
bool configured = false;
bool critical = false;
// failure states
bool tempSensorIsDisconnected = false;
bool mainLoopBrokeRealtime = false;
bool shaftRpmTooFast = false;
// telem ptrs
byte * rpmPtr = (byte *)&shaftRpmCurrent;
byte * forcePtr = (byte *)&loadCellForceCurrent;
byte * inletDutyPtr = (byte *)&inletDutyDesired;
byte * outletDutyPtr = (byte *)&outletDutyDesired;
byte * outletTempCurrentPtr = (byte *)&outletTemperatureCurrent;

unsigned long loopStartingMicros = 0;
unsigned long lastPidMicros = 0;
unsigned long lastTempMicros = 0;

// imports
#include <ArduPID.h>
#include <HX711_light.h>  // use robtillaart library
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NonBlockingDallas.h>

ArduPID inletController;
ArduPID outletController;

HX711_light torqueSensor(LOADCELL_DATA_PIN, LOADCELL_CLOCK_PIN);

OneWire oneWire(OUTLET_TEMP_PIN);
DallasTemperature dallasTempSensors(&oneWire);
DeviceAddress outletTempSensorAddress;
NonBlockingDallas nonBlockingTempSensor(&dallasTempSensors);

void setup() {

  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00001100; // x256 fast pwm
  
  // initialize comms 
  Serial.begin(SERIAL_RATE);

  // initialize all sensors
  torqueSensor.begin();
  // TODO: handle sensor not found error

  nonBlockingTempSensor.begin(NonBlockingDallas::resolution_12, NonBlockingDallas::unit_C, OUTLET_TEMP_SAMPLE_RATE_MICROS / 1000);
  nonBlockingTempSensor.onIntervalElapsed(handleTempTimeout);

  // dallasTempSensors.begin();
  // if (!dallasTempSensors.getAddress(outletTempSensorAddress, 0)) {
  //   // TODO: handle sensor not found error
  // }

  // initialize outputs
  setInlet(50);
  setOutlet(50);

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

  // start reading RPM
  attachInterrupt(digitalPinToInterrupt(SHAFT_HALL_PICKUP_PIN), hallInterrupt, FALLING);

}

void loop() {

  // record current time upon beginning loop
  loopStartingMicros = micros();

  calculateRpm();

  // Temperature

  nonBlockingTempSensor.update();

  // prevent bug on overflow
  // if (micros() < lastTempMicros) {
  //   lastTempMicros = 0;
  // }

  // if (!tempSensorIsDisconnected && (micros() >= lastTempMicros + OUTLET_TEMP_SAMPLE_RATE_MICROS - MAINLOOP_RATE_MICROS)) {

  //   dallasTempSensors.requestTemperatures();
  //   double outletTemperatureTemp = dallasTempSensors.getTempC(outletTempSensorAddress);

  //   if (outletTemperatureTemp == DEVICE_DISCONNECTED_C) {
  //     tempSensorIsDisconnected = true;
  //   } else {
  //     outletTemperatureCurrent = outletTemperatureTemp;
  //   }

  //   lastTempMicros = micros();

  // }

  // Load Cell
  if (torqueSensor.dataReady()) {
    loadCellForceCurrent = torqueSensor.readData();
  }

  // Recalculate pid (only works if enabled)
  // prevent bug on overflow
  if (micros() < lastPidMicros) {
    lastPidMicros = 0;
  }

  if (micros() >= (lastPidMicros + PID_SAMPLE_RATE_MICROS - MAINLOOP_RATE_MICROS)) {
    inletController.compute();
    outletController.compute();
    lastPidMicros = micros();
  }

  // TODO: Check for more failure cases
  shaftRpmTooFast = shaftRpmOverspeed();

  // Set outputs
  setInlet(inletDutyDesired);
  setOutlet(outletDutyDesired);

  // Check for serial comms + respond  + perform any special request + update variables
  if (Serial.available() == INCOMING_PACKET_SIZE_BYTES) { parseIncomingSerial(); }

  mainLoopBrokeRealtime = false;
  if ((micros() - loopStartingMicros) > MAINLOOP_RATE_MICROS) {
    mainLoopBrokeRealtime = true;
  }

  // hold here if main loop completes early
  while (micros() < loopStartingMicros + MAINLOOP_RATE_MICROS) {
    // prevent hang if micros() overflows
    if (micros() <= loopStartingMicros) { break; }
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

    byte inletDutyTemp = serialDataToByte();

    if (inletOverrideActive) {
      inletDutyDesired = inletDutyTemp;
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

    byte outletDutyTemp = serialDataToByte();

    if (outletOverrideActive) {
      outletDutyDesired = outletDutyTemp;
      sendTelemetry(true, false);
    } else {
      sendTelemetry(false, true);
    }

  } else if (commandByte == 0x18) {   // set load cell resolution TODO: FIX THIS

    unsigned int tempResolution = serialDataToUnsignedInt();

    // if (tempResolution == 64 || tempResolution == 128 ) {

    //   loadCellResolution = tempResolution;
    //   torqueSensor.set_gain(loadCellResolution, false);
    //   sendTelemetry(true, false);

    // } else {
    //   sendTelemetry(false, true);
    // }

    sendTelemetry(false, true);

  } else if (commandByte == 0x19) {   // set load cell offset DEPRECIATED

    unsigned long shred = serialDataToUnsignedLong();
    // torqueSensor.set_offset(loadCellOffset);
    sendTelemetry(false, true);

  } else if (commandByte == 0x1A) {   // set load cell scale DEPRECIATED

    double shred = serialDataToDouble();
    // torqueSensor.set_scale(loadCellScale);
    sendTelemetry(false, true);

  } else if (commandByte == 0x1B) {   // telemetry request (no data)

    shredSerialData(5);
    if (configured) {
      sendTelemetry(false, false);
    } else {
      sendTelemetry(false, true);
    }

  } else if (commandByte == 0x1C) {   // set configured bit

    shredSerialData(5);
    configured = true;
    sendTelemetry(true, false);

  }

}

// 21 bytes
void sendTelemetry(bool pass, bool fail) {

  // reset critical flag before re-checking for it
  critical = false;

  // status
  byte status = 0b00000000;
  
  if (pass) {
    bitSet(status, 5);
  } 
  if (fail) {
    bitSet(status, 6);
  }

  // TODO: Add more failure codes
  if (!configured) {
    status += 0x1;
    critical = true;
  } else if (mainLoopBrokeRealtime) {
    status += 0x2;
    critical = true;
  } else if (shaftRpmTooFast) {
    status += 0x3;
    critical = true;
  } else if (tempSensorIsDisconnected) {
    status += 0x04;
    critical = true;
  }

  if (critical) {
    bitSet(status, 4);
  }
  
  Serial.write(status);

  // shaft rpm
  Serial.write(rpmPtr, 4);

  // measured load cell force
  Serial.write(forcePtr, 4);

  // inlet duty cycle
  Serial.write(inletDutyPtr, 4);

  // outlet duty cycle
  Serial.write(outletDutyPtr, 4);

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

  if (shaftRpmCurrent >= shaftRpmMaximum - shaftRpmMaximumHyst) {
    return true;
  } else {
    return false;
  }

}

void setInlet(double duty) {

  int setDuty = (int)duty;
  // if (setDuty < inletMinDuty ) { setDuty = inletMinDuty; }
  // if (setDuty > inletMaxDuty) { setDuty = inletMaxDuty; }

  setDuty = map(setDuty, 0, 100, 53, 100);
  analogWrite(INLET_SERVO_PIN, setDuty);

}

void setOutlet(double duty) {

  int setDuty = (int)duty;
  // if (setDuty < outletMinDuty ) { setDuty = outletMinDuty; }
  // if (setDuty > outletMaxDuty) { setDuty = outletMaxDuty; }

  setDuty = map(setDuty, 0, 100, 53, 100);
  analogWrite(OUTLET_SERVO_PIN, setDuty);

}

void handleTempTimeout(float temperature, bool valid, int deviceIndex) {
  outletTemperatureCurrent = temperature;
}
