// just read and drop specified amount of data
void shredSerialData(unsigned int count) {

  for (int i = 0; i < count; i++) {
    Serial.read();
  }

}

// 8 bit - use last byte of inc. data
bool serialDataToBoolean() {

  shredSerialData(4);   // shred all but last byte of data
  bool returnData = Serial.read();
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

  uint8_t * returnDataLower = (uint8_t *)&returnData;
  uint8_t * returnDataUpper = (uint8_t *)&returnData + 1;

  *returnDataLower = Serial.read();
  *returnDataUpper = Serial.read();

  return returnData;

}

// 16 bit - use last 2 bytes of inc. data
unsigned int serialDataToUnsignedInt() {

  unsigned int returnData;

  shredSerialData(3);

  uint8_t * returnDataLower = (uint8_t *)&returnData;
  uint8_t * returnDataUpper = (uint8_t *)&returnData + 1;

  *returnDataLower = Serial.read();
  *returnDataUpper = Serial.read();

  return returnData;

}

// 32 bit - use last 4 bytes of inc. data
double serialDataToDouble() {

  double returnData;

  shredSerialData(1);

  uint8_t * d0 = (uint8_t *)&returnData;
  uint8_t * d1 = (uint8_t *)&returnData + 1;
  uint8_t * d2 = (uint8_t *)&returnData + 2;
  uint8_t * d3 = (uint8_t *)&returnData + 3;

  *d0 = Serial.read();
  *d1 = Serial.read();
  *d2 = Serial.read();
  *d3 = Serial.read();

  return returnData;

}

// 32 bit - use last 4 bytes of inc. data
unsigned long serialDataToUnsignedLong() {

  unsigned long returnData;

  shredSerialData(1);

  uint8_t * d0 = (uint8_t *)&returnData;
  uint8_t * d1 = (uint8_t *)&returnData + 1;
  uint8_t * d2 = (uint8_t *)&returnData + 2;
  uint8_t * d3 = (uint8_t *)&returnData + 3;

  *d0 = Serial.read();
  *d1 = Serial.read();
  *d2 = Serial.read();
  *d3 = Serial.read();

  return returnData;

}