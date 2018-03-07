// Adafruit AM2320 Temperature & Humidity Unified Sensor Library
// Copyright (c) 2018 Adafruit Industries
// Author: Limor Fried

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include "Adafruit_AM2320.h"

Adafruit_AM2320::Adafruit_AM2320(TwoWire *theI2C, int32_t tempSensorId, int32_t humiditySensorId):
  _temp(this, tempSensorId),
  _humidity(this, humiditySensorId),
  _i2c(theI2C)
{}

bool Adafruit_AM2320::begin() {
  _i2caddr = 0x5C;  // fixed addr
  _i2c->begin();
  return true;
}

float Adafruit_AM2320::readTemperature() {
  uint16_t t = readRegister16(AM2320_REG_TEMP_H);
  if (t == 0xFFFF) return NAN;

  float ft = (int16_t)t;
  return ft / 10.0;
}

float Adafruit_AM2320::readHumidity() {
  uint16_t h = readRegister16(AM2320_REG_HUM_H);
  if (h == 0xFFFF) return NAN;

  float fh = h;
  return fh / 10.0;
}

uint16_t Adafruit_AM2320::readRegister16(uint8_t reg) {
  // wake up
  Wire.beginTransmission(_i2caddr);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10); // wait 10 ms

  // send a command to read register
  Wire.beginTransmission(_i2caddr);
  Wire.write(AM2320_CMD_READREG);
  Wire.write(reg);
  Wire.write(2);  // 2 bytes
  Wire.endTransmission();

  delay(2);  // wait 2 ms
  
  // 2 bytes preamble, 2 bytes data, 2 bytes CRC
  Wire.requestFrom(_i2caddr, (uint8_t)6);
  if (Wire.available() != 6)
    return 0xFFFF;
  
  uint8_t buffer[6];
  for (int i=0; i<6; i++) {
    buffer[i] = Wire.read();
    //Serial.print("byte #"); Serial.print(i); Serial.print(" = 0x"); Serial.println(buffer[i], HEX);
  }

  if (buffer[0] != 0x03)   return 0xFFFF; // must be 0x03 modbus reply
  if (buffer[1] != 2)      return 0xFFFF; // must be 2 bytes reply
  
  uint16_t the_crc = buffer[5];
  the_crc <<= 8;
  the_crc |= buffer[4];
  uint16_t calc_crc = crc16(buffer, 4); // preamble + data
  //Serial.print("CRC: 0x"); Serial.println(calc_crc, HEX);
  if (the_crc != calc_crc)
    return 0xFFFF;

  // All good!
  uint16_t ret = buffer[2];
  ret <<= 8;
  ret |= buffer[3];

  return ret;
}


uint16_t Adafruit_AM2320::crc16(uint8_t *buffer, uint8_t nbytes) {
  uint16_t crc = 0xffff;
  for (int i=0; i<nbytes; i++) {
    uint8_t b = buffer[i];
    crc ^= b;
    for (int x=0; x<8; x++) {
      if (crc & 0x0001) {
	crc >>= 1;
	crc ^= 0xA001;
      } else {
	crc >>= 1;
      }
    }
  }
  return crc;
}

void Adafruit_AM2320::setName(sensor_t* sensor) {
  strncpy(sensor->name, "AM2320", sizeof(sensor->name) - 1);
}

void Adafruit_AM2320::setMinDelay(sensor_t* sensor) {
  sensor->min_delay = 2000000L;  // 2 seconds (in microseconds)
}

Adafruit_AM2320::Temperature::Temperature(Adafruit_AM2320* parent, int32_t id):
  _parent(parent),
  _id(id)
{}

bool Adafruit_AM2320::Temperature::getEvent(sensors_event_t* event) {
  // Clear event definition.
  memset(event, 0, sizeof(sensors_event_t));
  // Populate sensor reading values.
  event->version     = sizeof(sensors_event_t);
  event->sensor_id   = _id;
  event->type        = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  event->timestamp   = millis();
  event->temperature = _parent->readTemperature();
  
  return true;
}

void Adafruit_AM2320::Temperature::getSensor(sensor_t* sensor) {
  // Clear sensor definition.
  memset(sensor, 0, sizeof(sensor_t));
  // Set sensor name.
  _parent->setName(sensor);
  // Set version and ID
  sensor->version         = AM2320_SENSOR_VERSION;
  sensor->sensor_id       = _id;
  // Set type and characteristics.
  sensor->type            = SENSOR_TYPE_AMBIENT_TEMPERATURE;
  _parent->setMinDelay(sensor);

  // This isn't documented, of course
  sensor->max_value   = 80.0F;
  sensor->min_value   = -20.0F;
  sensor->resolution  = 2.0F;
}

Adafruit_AM2320::Humidity::Humidity(Adafruit_AM2320* parent, int32_t id):
  _parent(parent),
  _id(id)
{}

bool Adafruit_AM2320::Humidity::getEvent(sensors_event_t* event) {
  // Clear event definition.
  memset(event, 0, sizeof(sensors_event_t));
  // Populate sensor reading values.
  event->version           = sizeof(sensors_event_t);
  event->sensor_id         = _id;
  event->type              = SENSOR_TYPE_RELATIVE_HUMIDITY;
  event->timestamp         = millis();
  event->relative_humidity = _parent->readHumidity();
  
  return true;
}

void Adafruit_AM2320::Humidity::getSensor(sensor_t* sensor) {
  // Clear sensor definition.
  memset(sensor, 0, sizeof(sensor_t));
  // Set sensor name.
  _parent->setName(sensor);
  // Set version and ID
  sensor->version         = AM2320_SENSOR_VERSION;
  sensor->sensor_id       = _id;
  // Set type and characteristics.
  sensor->type            = SENSOR_TYPE_RELATIVE_HUMIDITY;
  _parent->setMinDelay(sensor);

  // This isn't documented, of course
  sensor->max_value   = 100.0F;
  sensor->min_value   = 0.0F;
  sensor->resolution  = 1.0F;
}
