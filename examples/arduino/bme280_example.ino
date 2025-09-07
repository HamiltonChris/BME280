#include <Wire.h>
#include "bme280.h"

void i2c_send(uint8_t address, uint8_t* data, uint8_t length);
void i2c_receive(uint8_t address, uint8_t* data, uint8_t length);

bme280_t bme280;
uint32_t pressure = 0;
uint32_t humidity = 0;
int32_t temperature = 0;
char user_data[100];
uint8_t chip_id = 0;
int8_t status = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);
  
  Wire.begin();

	bme280.send = &i2c_send;
	bme280.receive = &i2c_receive;
	bme280.temperature_oversampling = OVERSAMPLING_x1;
	bme280.pressure_oversampling = OVERSAMPLING_x1;
	bme280.humidity_oversampling = OVERSAMPLING_x1;
	bme280.standby = SB_1000MS;
	bme280.filter = FILTER_OFF;
	bme280.mode = NORMAL;

  status = bme280_init(&bme280);
  if (status)
  {
    Serial.print("Error initializing driver\n");
  }
  else
  {
    Serial.print("Initialization Complete\n");
    Serial.print("Test BME280\n");
  }
}

void loop() {
  if (!status)
  {
    status = bme280_read_all(&bme280, &pressure, &humidity, &temperature);
    if (!status)
    {
      Serial.print("Error with config structure.");
    }
    else
    {
      sprintf((char*)user_data, "Pressure: %d.%02d Pa, Temperature: %d.%02d degrees C, Humidity %d.%03d%%\r\n",
        pressure >> 8, (pressure & 0xFF) * 100 >> 8, temperature / 100, temperature % 100,
        humidity >> 10, (humidity &0x3FF) * 1000 >> 10);

      Serial.print(user_data);
    }
  }
  delay(1000);
}

// i2c callback function for writes
void i2c_send(uint8_t address, uint8_t* data, uint8_t length)
{
  Wire.beginTransmission(address);
  Wire.write(data, length);
  Wire.endTransmission();
}

// i2c callback function for reads
void i2c_receive(uint8_t address, uint8_t* data, uint8_t length)
{
  Wire.requestFrom(address, length);
  for (uint8_t i = 0; i < length; i++)
  {
    *data = Wire.read();
    data++;
  }
}