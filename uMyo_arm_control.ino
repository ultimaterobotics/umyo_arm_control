#include <Wire.h>
#include <Arduino.h>
#include <uMyo_RF24.h>

int rf_cen = 10; //nRF24 chip enable pin
int rf_cs = 9; //nRF24 CS pin

uint8_t servo_states[5];

uint8_t pca_addr = 0x40;
uint8_t pca_enable_pin = 5;

//========PCA9685 driver==============

#define PCA_ALLOFF 0xFD
#define PCA_PRESCALE 0xFE
#define PCA_LED0 0x06
#define PCA_MODE1 0
#define PCA_MODE2 0

void pca_write8(uint8_t reg, uint8_t val)
{
  Wire.beginTransmission(pca_addr);
  Wire.write(reg);
  Wire.write(val);
  int err = Wire.endTransmission();
}

uint8_t pca_read8(uint8_t reg)
{    
  Wire.beginTransmission(pca_addr);
  Wire.write(reg);
  int err = Wire.endTransmission();
  Wire.requestFrom(pca_addr, 1);
  uint8_t res = Wire.read();
  return res;
}

void pca_write16(uint8_t reg, uint16_t val)
{
  Wire.beginTransmission(pca_addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.write(val>>8);
  int err = Wire.endTransmission();
}

void pca_write_buf(uint8_t reg, uint8_t *buf, int len)
{
  Wire.beginTransmission(pca_addr);
  Wire.write(reg);
  for (int x = 0; x < len; x++)
    Wire.write(buf[x]);
  int err = Wire.endTransmission();
}

void pca_set_prescaler(uint16_t val)
{
  uint8_t oldmode = pca_read8(PCA_MODE1);
  uint8_t newmode = (oldmode & 0x7F) | 0x10; // sleep
  pca_write8(PCA_MODE1, newmode);            // go to sleep
  pca_write8(PCA_PRESCALE, val);        // set the prescaler
  pca_write8(PCA_MODE1, oldmode);
  delay(5);
  pca_write8(PCA_MODE1, oldmode | 0xa0); //  This sets the MODE1 register to turn on auto increment.
}

void pca_set_chan(int chan, uint16_t on_time, uint16_t off_time)
{
  int reg = PCA_LED0 + chan*4;
  uint8_t buf[4];
  buf[0] = on_time;
  buf[1] = on_time>>8;
  buf[2] = off_time;
  buf[3] = off_time>>8;
  pca_write_buf(reg, buf, 4);
}

int freq = 50;
uint32_t period_us = 1000000 / freq;

void pca_set_servos(uint8_t *servo_buf)
{
  uint8_t buf[20]; //16 bytes per transaction
  for(int n = 0; n < 5; n++)
  {
    int s = n;
    uint16_t val = servo_states[s];
    uint16_t val_us = 850 + val*4;
    uint16_t tim = ((uint32_t)val_us*4096) / period_us;
    buf[n*4] = 0; //on time zero
    buf[n*4+1] = 0; 
    buf[n*4+2] = tim; 
    buf[n*4+3] = tim>>8; 
  }
  pca_write_buf(PCA_LED0, buf, 20);
}

//========End of PCA9685 driver==============

void setup()
{  
  Serial.begin(115200);
  //init uMyo nRF24 library
  uMyo.begin(rf_cs, rf_cen);
  //init Wire
  digitalWrite(A4, HIGH);
  digitalWrite(A5, HIGH);
  Wire.begin();

  //init PCA9685
  pca_write8(PCA_MODE1, 0x80);

  pinMode(pca_enable_pin, OUTPUT);
  digitalWrite(pca_enable_pin, 0);

  pca_set_prescaler(6103 / freq - 1);

  //set servos in the middle position
  for(int s = 0; s < 5; s++)
    servo_states[s] = 128;
  pca_set_servos(servo_states);
}

uint32_t last_out_ms = 0; //time when we last updated servos

void loop()
{
  uint32_t ms = millis();
  
  uMyo.run(); //it's necessary to call it every few milliseconds or data will be lost
  int dev_count = uMyo.getDeviceCount(); //for this project we wait until 3 uMyo units appear
  if(dev_count < 3) return;

  //channels are:
  //0 - thumb muscles
  //1 - 3rd finger
  //2 - 4th and 5th fingers combined
  //uMyo units must be turned on in this order after this
  //program was started, so that first device that appeared (index 0)
  //would be used for thumb, second for 3rd finger, etc
  float ch0 = uMyo.getMuscleLevel(0);
  float ch1 = uMyo.getMuscleLevel(1);
  float ch2 = uMyo.getMuscleLevel(2);

  //now we want to ignore muscle levels below certain
  //threshold - bias_chX variables - and with certain scale
  //defined with scale_chX variables
  float v0, v1, v2;
  float bias_ch0 = 200;
  float bias_ch1 = 250;
  float bias_ch2 = 200;
  float scale_ch0 = 150;
  float scale_ch1 = 250;
  float scale_ch2 = 400;

  v0 = (ch0 - bias_ch0)/scale_ch0;
  v1 = (ch1 - bias_ch1)/scale_ch1;
  v2 = (ch2 - bias_ch2)/scale_ch2;
  //limit to 0...1 to further get correct servo positions
  if(v0 < 0) v0 = 0;
  if(v0 > 1) v0 = 1;
  if(v1 < 0) v1 = 0;
  if(v1 > 1) v1 = 1;
  if(v2 < 0) v2 = 0;
  if(v2 > 1) v2 = 1;

  static float avg_v0 = 0;
  static float avg_v1 = 0;
  static float avg_v2 = 0;
  //now we perform exponential averaging so that servos are less shaky
  float acf = 0.97;
  avg_v0 = acf*avg_v0 + (1-acf)*v0;
  avg_v1 = acf*avg_v1 + (1-acf)*v1;
  avg_v2 = acf*avg_v2 + (1-acf)*v2;

  //and here we set servo states. Servo output function
  //is defined in a way that 0...255 is mapped into
  //min...max servo angle
  servo_states[0] = 255*avg_v0;
  servo_states[1] = 255*avg_v1;
  servo_states[2] = 255*avg_v1;
  servo_states[3] = 255*avg_v2;
  servo_states[4] = 255*avg_v2;

  //and now we update servos and print info for debug,
  //but only if it's been more than 20ms since the last update
  if(ms - last_out_ms > 20)
  {
    last_out_ms = ms;
    pca_set_servos(servo_states);
    char buf[128];
    int len = 0;
    len = sprintf(buf, "%d %d %d\n", (int)ch0, (int)ch1, (int)ch2);
    Serial.write(buf, len);
  }
}
