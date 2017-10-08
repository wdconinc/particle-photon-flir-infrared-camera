/*
Copyright (c) 2014, Pure Engineering LLC
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

// Debugging flags
int debug = 0;

// Pin assignments for SPI connection
// A2 SS, A3 CLK, A4 MISO, A5 MOSI
#define FLIR_SPI_SS A2
#define FLIR_SPI_CLK A3
#define FLIR_SPI_MISO A4
#define FLIR_SPI_MOSI A5

// Pin assignments for I2C connection
#define FLIR_I2C_SDA 0
#define FLIR_I2C_SCK 1

// I2C address
#define FLIR_I2C_ADDRESS (0x2A)

// FLIR I2C module types
#define AGC (0x01)
#define SYS (0x02)
#define VID (0x03)
#define OEM (0x08)

// FLIR I2C command types
#define GET (0x00)
#define SET (0x01)
#define RUN (0x02)

#define VOSPI_FRAME_SIZE (164)

#include "mdns.h"

void setup()
{
  // Initialize serial connection
  Serial.begin(115200);

  // Initialize I2C connection
  Wire.setSpeed(1000); // 1 MHz
  Wire.begin();

  // Initialize SPI connection
  pinMode(FLIR_SPI_SS, OUTPUT);
  SPI.setClockDivider(SPI_CLOCK_DIV4); // 60 MHz / 4 = 15 MHz
  SPI.setBitOrder(MSBFIRST); // Data is read and written MSB first.
  SPI.setDataMode(SPI_MODE3);
  SPI.begin();

  Serial.println("setup complete");
}

int spi_read_word(int data)
{
  // take the SS pin low to select the chip:
  digitalWrite(FLIR_SPI_SS, LOW);

  //  send in the address and value via SPI:
  int read_data;
  read_data = SPI.transfer(data >> 8) << 8;
  read_data |= SPI.transfer(data);

  // take the SS pin high to de-select the chip:
  digitalWrite(FLIR_SPI_SS, HIGH);

  return read_data;
}

byte lepton_frame_packet[VOSPI_FRAME_SIZE];

#define IMAGE_SIZE (800)
byte image[IMAGE_SIZE];
int image_index;
void read_lepton_frame(void)
{
  for (int i = 0; i < (VOSPI_FRAME_SIZE / 2); i++)
  {
    // take the SS pin low to select the chip:
    digitalWrite(FLIR_SPI_SS, LOW);

    //  send in the address and value via SPI:
    lepton_frame_packet[2 * i] = SPI.transfer(0x00);
    lepton_frame_packet[2 * i + 1] = SPI.transfer(0x00);

    // take the SS pin high to de-select the chip:
    digitalWrite(FLIR_SPI_SS, HIGH);
  }
}

void lepton_sync(void)
{
  int i;
  int data = 0x0f;

  // take the SS pin high to de-select the chip:
  digitalWrite(FLIR_SPI_SS, HIGH);

  delay(185);
  while ((data & 0x0f) == 0x0f)
  {
    // take the SS pin low to select the chip:
    digitalWrite(FLIR_SPI_SS, LOW);

    data = SPI.transfer(0x00) << 8;
    data |= SPI.transfer(0x00);

    // take the SS pin high to de-select the chip:
    digitalWrite(FLIR_SPI_SS, HIGH);

    for (i = 0; i < ((VOSPI_FRAME_SIZE - 2) / 2); i++)
    {
      // take the SS pin low to select the chip:
      digitalWrite(FLIR_SPI_SS, LOW);

      SPI.transfer(0x00);
      SPI.transfer(0x00);

      // take the SS pin high to de-select the chip:
      digitalWrite(FLIR_SPI_SS, HIGH);
    }
  }

}

void print_lepton_frame(void)
{
  for (int i = 0; i < (VOSPI_FRAME_SIZE); i++)
  {
    Serial.print(lepton_frame_packet[i], HEX);
    Serial.print(",");
  }
  Serial.println(" ");
}

void print_image(void)
{
  for (int i = 0; i < (IMAGE_SIZE); i++)
  {
    Serial.print(image[i], HEX);
    Serial.print(",");
  }
  Serial.println(" ");
}

void lepton_command(unsigned int moduleID, unsigned int commandID, unsigned int command)
{
  Wire.beginTransmission(FLIR_I2C_ADDRESS);

  // Command Register is a 16-bit register located at Register Address 0x0004
  Wire.write(0x00);
  Wire.write(0x04);

  if (moduleID == 0x08) //OEM module ID
  {
    Wire.write(0x48);
  }
  else
  {
    Wire.write(moduleID & 0x0f);
  }
  Wire.write( ((commandID << 2 ) & 0xfc) | (command & 0x3));

  byte error = Wire.endTransmission();    // stop transmitting
  if (error != 0)
  {
    Serial.print("I2C error=");
    Serial.println(error);
  }
}

void agc_enable()
{
  Wire.beginTransmission(FLIR_I2C_ADDRESS); // transmit to device #4
  Wire.write(0x01);
  Wire.write(0x05);
  Wire.write(0x00);
  Wire.write(0x01);

  byte error = Wire.endTransmission();    // stop transmitting
  if (error != 0)
  {
    Serial.print("I2C error=");
    Serial.println(error);
  }
}

void set_reg(unsigned int reg)
{
  Wire.beginTransmission(FLIR_I2C_ADDRESS); // transmit to device #4
  Wire.write(reg >> 8 & 0xff);
  Wire.write(reg & 0xff);            // sends one byte

  byte error = Wire.endTransmission();    // stop transmitting
  if (error != 0)
  {
    Serial.print("I2C error=");
    Serial.println(error);
  }
}

//Status reg 15:8 Error Code  7:3 Reserved 2:Boot Status 1:Boot Mode 0:busy

int read_reg(unsigned int reg)
{
  set_reg(reg);

  Wire.requestFrom(FLIR_I2C_ADDRESS, 2);

  int reading = 0;
  reading = Wire.read();  // receive high byte (overwrites previous reading)
  //Serial.println(reading);
  reading = reading << 8;    // shift high byte to be high 8 bits

  reading |= Wire.read(); // receive low byte as lower 8 bits

  if (debug > 0) {
    Serial.print("reg 0x");
    Serial.print(reg, HEX);
    Serial.print(" = 0x");
    Serial.print(reading, HEX);
    Serial.print(" = 0b");
    Serial.println(reading, BIN);
  }

  return reading;
}

int read_data_V(unsigned int n = 1, unsigned int* data = 0)
{
  while (read_reg(0x2) & 0x01)
  {
    Serial.println("busy");
  }

  int payload_length = read_reg(0x6);
  if (debug > 0) {
    Serial.print("payload_length=");
    Serial.println(payload_length);
  }

  Wire.requestFrom(FLIR_I2C_ADDRESS, payload_length);
  //set_reg(0x08);
  for (int i = 0; i < (payload_length / 2); i++)
  {
    data[i] = Wire.read() << 8;
    data[i] |= Wire.read();
    if (debug > 0) {
      Serial.print("0x");
      Serial.println(data[i], HEX);
    }
  }

  return data[0];
}

int read_data()
{
  while (read_reg(0x2) & 0x01)
  {
    Serial.println("busy");
  }

  int payload_length = read_reg(0x6);
  if (debug > 0) {
    Serial.print("payload_length=");
    Serial.println(payload_length);
  }

  Wire.requestFrom(FLIR_I2C_ADDRESS, payload_length);
  //set_reg(0x08);
  int data;
  for (int i = 0; i < (payload_length / 2); i++)
  {
    data = Wire.read() << 8;
    data |= Wire.read();
    if (debug > 0) {
      Serial.print("0x");
      Serial.println(data, HEX);
    }
  }

  return data;
}


void loop()
{
  Serial.println("beginTransmission");

  int data;

  read_reg(0x2);

  // Print system information
  Serial.println("SYS Camera Customer Serial Number");
  lepton_command(SYS, 0x28 >> 2 , GET);
  data = read_data();

  Serial.println("SYS Flir Serial Number");
  lepton_command(SYS, 0x2 , GET);
  data = read_data();
  Serial.print("SYS Flir Serial Number = ");
  Serial.println(data);

  Serial.println("SYS Camera Uptime");
  lepton_command(SYS, 0x0C >> 2 , GET);
  data = read_data();
  Serial.print("SYS Camera Uptime = [ms] ");
  Serial.println(data);

  Serial.println("SYS Fpa Temperature Kelvin");
  lepton_command(SYS, 0x14 >> 2 , GET);
  data = read_data();
  Serial.print("SYS Fpa Temperature Kelvin = ");
  Serial.println(((float)data)/100);

  Serial.println("SYS Aux Temperature Kelvin");
  lepton_command(SYS, 0x10 >> 2 , GET);
  data = read_data();
  Serial.print("SYS Aux Temperature Kelvin = ");
  Serial.println(((float)data)/100);

  Serial.println("OEM Chip Mask Revision");
  lepton_command(OEM, 0x14 >> 2 , GET);
  data = read_data();

  Serial.println("OEM Part Number");
  lepton_command(OEM, 0x1C >> 2 , GET);
  data = read_data();

  Serial.println("OEM Camera Software Revision");
  lepton_command(OEM, 0x20 >> 2 , GET);
  data = read_data();

  Serial.println("AGC Enable");
  //lepton_command(AGC, 0x01  , SET);
  agc_enable();
  data = read_data();

  Serial.println("AGC READ");
  lepton_command(AGC, 0x00  , GET);
  data = read_data();

  Serial.println("SYS Telemetry Enable State");
  lepton_command(SYS, 0x19>>2 ,GET);
  data = read_data();

  //while (1)
  //{
    lepton_sync();
    read_lepton_frame();
    if(lepton_frame_packet[0]&0x0f != 0x0f )
    {
      print_lepton_frame();
    }
  //}

  delay(5000);
}
