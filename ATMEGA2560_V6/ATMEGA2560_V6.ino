
/*
  COPYRIGHT 2020 LEDGER ASSETS PTY LTD
  Buffer size was modified by GVE in wire.h and TWI.h to 255 chars
  for SPS30 to provide particle count in using I2C interface
*/

#include <Arduino.h>
#include <DHT.h>
#include <sps30.h>
#include <CDM7160.h>
#include <Wire.h>

#define LOG_OUT 1 // use the log output function
#define FHT_N 128 // set to 128 point fht (gives 64 freq bins)
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#include <FHT.h> // include the library

unsigned long StartTime = 0;
unsigned long EndTime = 0;

char arrayCamera[64];

int deltalinetotal = 0;
int lastblocktotal = 0;
int linetotal = 0;
int prevlinetotal = 0;
int countergov = 1;
float blocktotal = 0;
float blocktotaldensity = 0;
int lastblocktotaldensity = 0;
int temold;
int humold;
int cont = 1;
int varDelay = 300;
int mainfan_on_counter = 1;
byte gridtempLow;
byte gridtempHigh;
//test for this address first
char addr = 0x68; //Grid Eye I2C Address
float gridtempCelsius;
float averagetemp;
int irqPin = 2; // IRQ Pin
int pwmPin = 3; // PWM Pin


//dht DHT;
CDM7160 co2;
SPS30 sps30;

/* I2C coms for SPS30 */
#define SP30_COMMS  SERIALPORT2

/* define RX and TX pins for SPS30  now it is running in UART not I2C*/
//needed, but in digital sda and scl so this overridden later
#define TX_PIN 16
#define RX_PIN 17

/* define DIGITAL for Temp and Humidity Digital Sensor        */
//#define DHT11_PIN 8
#define DHTPIN 8      // what pin we're connected to
#define DHTTYPE DHT22   // DHT 22  (AM2302)
DHT dht(DHTPIN, DHTTYPE); //// Initialize DHT sensor for normal 16mhz Arduino

#define DEBUG 0
/* define driver debug
   0 : no messages
   1 : request sending and receiving
   2 : request sending and receiving + show protocol errors */

// function prototypes (sometimes the pre-processor does not create prototypes themself on ESPxx)
void serialTrigger(char * mess);
void ErrtoMess(char *mess, uint8_t r);
void Errorloop(char *mess, uint8_t r);
void GetDeviceInfo();
bool read_sps30();

void setup() {
  //Serial.println(" ");
  //Serial.print("\n");
  Serial.begin(115200);
  Serial3.begin(115200);
  dht.begin();

  //set up main fan pin
  pinMode(9, OUTPUT);

  //set prescaler to 32 for a sample rate of 38.4kHz (BW = 19.2kHz)
  sbi(ADCSRA, ADPS2);
  cbi(ADCSRA, ADPS1);
  sbi(ADCSRA, ADPS0);

  ADCSRA = 0xe5; // set the adc to free running mode AND pre Scaler to 32
  ADMUX = 0x40; // use adc0
  DIDR0 = 0x01; // turn off the digital input for adc0

  scani2cbus();
  //delay(300);
  Serial.println("Connect I2C bus to CDM7160 and GridEye slaves \n");
  Wire.begin();
  pinMode(irqPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  co2.begin();

  // set pins to use for softserial and Serial1 sps30 on ESP32
  // if (TX_PIN != 0 && RX_PIN != 0) sps30.SetSerialPin(RX_PIN,TX_PIN);

  // Begin communication channel;
  if (sps30.begin(SP30_COMMS) == false) {
    Errorloop((char *) "could not initialize communication channel.", 0);
  }

  // check for SPS30 connection
  if (sps30.probe() == false) {
    Errorloop((char *) "could not probe / connect with SPS30.", 0);
  }
  else
    Serial.println(F("Detected SPS30 successfully"));
  //Serial.print("\n");

  // reset SPS30 connection
  if (sps30.reset() == false) {
    Errorloop((char *) "could not reset.", 0);
  }

  // read device info
  GetDeviceInfo();

  // start measurement
  if (sps30.start() == true)
    Serial.println(F("Measurement starting now..."));

  else
    Errorloop((char *) "Could NOT start measurement", 0);

  if (SP30_COMMS == I2C_COMMS) {
    if (sps30.I2C_expect() == 4)
      Serial.println(F(" !!! Due to I2C buffersize only the SPS30 MASS concentration is available !!! \n"));
  }
  Serial.print("\n");
}

void loop() {

  read_sps30();
  read_dht11();
  co2.readData();
  Serial.print(",");
  Serial.print(co2.getCo2());
  Serial.print(",,");

  // Send CO2 value to the wifi module
  Serial3.print(co2.getCo2());
  Serial3.write('\r');

  cont = 1;
  //Serial.println();
  sps30.stop();
  //Serial.println("stop issued and delay");
  // 1000 ms delay to silence sps 30 fan so recording can start
  delay(1000);
  //Serial.println("silent?. now enter key to cont");
  //waitForSerial();
  //Serial.println("about to start mic data collection...press any character plus Return to continue....note next char printed is the key you hit.. ");
  //waitForSerial();
  runmicrophone();
  // now restart sps30
  sps30.start();
  delay(4000);
  //Serial.println("Microphone data aggregation complete, restarted sps30, press any character plus Return to continue....note next char printed is the key you hit.. ");
  //waitForSerial();
  //a 4000ms dely is enough to spool it up...
  //Serial.println("started and delayed, continue ? ");
  //waitForSerial();
  //delay(varDelay);

  if (mainfan_on_counter > 5)  {
    //turn on main fan relay
    digitalWrite(9, HIGH);
    delay(30000);
    digitalWrite(9, LOW);
    mainfan_on_counter = 0;
  }
  else {
    //  Serial.println ("here");
    //  Serial.println (mainfan_on_counter);
  }

  runthermalcamera();

  mainfan_on_counter = mainfan_on_counter + 1;
}


void read_dht11() {
  int tem;
  int hum;
  //int chk = DHT.read11(DHT11_PIN);
  tem = dht.readTemperature();
  hum = dht.readHumidity();

  if (tem > 0 ) {
    Serial.print(",");
    Serial.print(tem);
    Serial.print(",");
    Serial.print(hum);
    temold = dht.readTemperature();
    humold = dht.readHumidity();

    // Parse the int to string and send it to the ESP8266 wifi module via the Serial 3
    char humString[5];         //the ASCII of the integer will be stored in this char array
    itoa(hum, humString, 10);
    char temString[5];
    itoa(tem, temString, 10);
    Serial3.write(humString);
    Serial3.write('\r');
    Serial3.write(temString);
    Serial3.write('\r');

  } else {

    Serial.print(",");
    Serial.print(temold);
    Serial.print(",");
    Serial.print(humold);

    // Parse the int to string and send it to the ESP8266 wifi module via the Serial 3
    char humOldString[5];         //the ASCII of the integer will be stored in this char array
    itoa(humold, humOldString, 10);
    char temOldString[5];
    itoa(temold, temOldString, 10);
    Serial3.write(humOldString);
    Serial3.write('\r');
    Serial3.write(temOldString);
    Serial3.write('\r');
  }

}

void runmicrophone () {

  //i number on next line dictates lenght microphone records for, extensively tested and 4000 is good
  for (int i = 0 ; i < 4000 ; i++) {
    for (int i = 0 ; i < FHT_N ; i++) { // save 256 samples
      while (!(ADCSRA & 0x10)); // wait for adc to be ready

      ADCSRA = 0xe5;

      byte m = ADCL; // fetch adc data
      byte j = ADCH;
      int k = (j << 8) | m; // form into an int
      k -= 0x0200; // form into a signed int
      k <<= 6; // form into a 16b signed int
      fht_input[i] = k; // put real data into bins

    }
    fht_window(); // window the data for better frequency response
    fht_reorder(); // reorder the data before doing the fht
    fht_run(); // process the data in the fht
    fht_mag_log(); // take the output of the fht

    //    Serial.print("Start:  ");


    for (byte i = 0 ; i < FHT_N / 2 ; i++) {

      //note FHT_N/2  so HALF the freq bins only fits Nyquist thereom
      // the abs() function MALFUNCTIONS - C is crap ! - if Blockttoal gets to about 30,000 hence the divide by 100

      linetotal = linetotal + ((fht_log_out[i]) / 50);

      //  Serial.print(fht_log_out[i]); // send out the data
      //  Serial.print(" , ");

    }
    // here the difference in the sum of the Bins for the line, with the previosu line are added up
    // blocktotal is the total delta for a [1 min] block of data
    // countergov tests show 125 lines per 5 sec of data
    // therefore a [1 min] Block is 1500 lines
    // CAUTION : abs FUNCTIN FAILS IF BLOCKCOUNT GOES OVER ABOUT 35,000
    deltalinetotal = abs((prevlinetotal) - (linetotal));
    blocktotal = blocktotal + (deltalinetotal) / 50;
    prevlinetotal = linetotal;
    blocktotaldensity = blocktotaldensity + (linetotal) / 50;

    //Serial.print(" linetotal ");
    //   Serial.print(linetotal);
    //Serial.print(" , ");
    //Serial.print(" deltalinetotal ");
    //     Serial.print(deltalinetotal);
    //Serial.println("  ");

    countergov = countergov + 1;
    //reset counters
    linetotal = 0;
    deltalinetotal = 0;

    // this end of the while i count
  }

  lastblocktotal = blocktotal;
  lastblocktotaldensity = blocktotaldensity;

  //   Serial.println();
  //   Serial.println();
  //   Serial.println("MIC DATA ...Lastblocktotal  ");
  Serial.print(lastblocktotal);
  Serial.print(",");
  //   Serial.print("MIC DATA ...Lastblocktotaldensity  ");
  Serial.println(lastblocktotaldensity);
  Serial.print(" ,, ");

  // Parse the int to string and send it to the ESP8266 wifi module via the Serial 3
  char lastblocktotalString[5];         //the ASCII of the integer will be stored in this char array
  itoa(lastblocktotal, lastblocktotalString, 10);
  char lastblocktotaldensityString[5];
  itoa(lastblocktotaldensity, lastblocktotaldensityString, 10);
  Serial3.write(lastblocktotalString);
  Serial3.write('\r');
  Serial3.write(lastblocktotaldensityString);
  Serial3.write('\r');

  //waitForSerial();

  prevlinetotal = 0;
  countergov = 1;
  blocktotal = 0;
  blocktotaldensity = 0;
  deltalinetotal = 0;
}

void GetDeviceInfo()
{
  char buf[32];
  uint8_t ret;
  SPS30_version v;

  //try to read serial number
  ret = sps30.GetSerialNumber(buf, 32);
  if (ret == ERR_OK) {
    Serial.print(F("Serial number : "));
    if (strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get serial number", ret);

  // try to get product name
  ret = sps30.GetProductName(buf, 32);
  if (ret == ERR_OK)  {
    Serial.print(F("Product name  : "));

    if (strlen(buf) > 0)  Serial.println(buf);
    else Serial.println(F("not available"));
  }
  else
    ErrtoMess((char *) "could not get product name.", ret);

  // try to get version info
  ret = sps30.GetVersion(&v);
  if (ret != ERR_OK) {
    Serial.println(F("Can not read version info"));
    return;
  }

  Serial.print("Firmware level: ");
  Serial.print(v.major);
  Serial.print(".");
  Serial.println(v.minor);

  if (SP30_COMMS != I2C_COMMS) {
    Serial.print("Hardware level: ");
    Serial.println(v.HW_version);
    Serial.print("SHDLC protocol: ");
    Serial.print(v.SHDLC_major);
    Serial.print(".");
    Serial.println(v.SHDLC_minor);
  }

  Serial.print("Library level : ");
  Serial.print(v.DRV_major);
  Serial.print(".");
  Serial.println(v.DRV_minor);
}

/**
   @brief : read and display all values
*/
bool read_sps30()
{
  static bool header = true;
  uint8_t ret, error_cnt = 0;
  struct sps_values val;

  // loop to get data
  do {

    ret = sps30.GetValues(&val);

    // data might not have been ready
    if (ret == ERR_DATALENGTH) {

      if (error_cnt++ > 3) {
        ErrtoMess((char *) "Error during reading values: ", ret);
        return (false);
      }
      delay(1000);
    }

    // if other error
    else if (ret != ERR_OK) {
      ErrtoMess((char *) "Error during reading values: ", ret);
      return (false);
    }

  } while (ret != ERR_OK);

  // only print header first time
  if (header) {

    //Serial.print("Temp, Hum%, C02 ");
    Serial.println("-------------Mass -----------    ------------- Number --------------,,   -Average-    Temp C, Hum %, C02 ppm,, MicBlockTOTAL, MicBlockDensity,, Thermal data Array deg C ");
    Serial.println(F("        Concentration [μg/m3]             Concentration [#/cm3]             [μm]"));
    Serial.println(F("        P1.0\tP2.5\tP4.0\tP10\tP0.5\tP1.0\tP2.5\tP4.0\tP10\tPartSize\n"));
    Serial.println("  ");

    header = false;
  }

  // Read the data from the SPS30
  float MassPM1 = val.MassPM1;
  float MassPM2 = val.MassPM2;
  float MassPM4 = val.MassPM4;
  float MassPM10 = val.MassPM10;
  float NumPM0 = val.NumPM0;
  float NumPM1 = val.NumPM1;
  float NumPM2 = val.NumPM2;
  float NumPM4 = val.NumPM4;
  float NumPM10 = val.NumPM10;
  float PartSize = val.PartSize;

  Serial.print(" ");
  Serial.print(MassPM1);
  Serial.print(",");
  Serial.print(MassPM2);
  Serial.print(",");
  Serial.print(MassPM4);
  Serial.print(",");
  Serial.print(MassPM10);
  Serial.print(",");
  Serial.print(NumPM0);
  Serial.print(",");
  Serial.print(NumPM1);
  Serial.print(",");
  Serial.print(NumPM2);
  Serial.print(",");
  Serial.print(NumPM4);
  Serial.print(",");
  Serial.print(NumPM10);
  Serial.print(",");
  Serial.print(PartSize);
  Serial.print(",");


  // transform the float to string and send it to ESP8266 wifi module via Serial 3
  int MassPM1_int = MassPM1 * 100;
  int MassPM2_int = MassPM2 * 100;
  int MassPM4_int = MassPM4 * 100;
  int MassPM10_int = MassPM10 * 100;
  int NumPM0_int = NumPM0 * 100;
  int NumPM1_int = NumPM1 * 100;
  int NumPM2_int = NumPM2 * 100;
  int NumPM4_int = NumPM4 * 100;
  int NumPM10_int = NumPM10 * 100;
  int PartSize_int = PartSize * 100;

  // Create the array of string
  char MassPM1ToString[5];
  char MassPM2ToString[5];
  char MassPM4ToString[5];
  char MassPM10ToString[5];
  char NumPM0ToString[5];
  char NumPM1ToString[5];
  char NumPM2ToString[5];
  char NumPM4ToString[5];
  char NumPM10ToString[5];
  char PartSizeToString[5];

  // parse the integer values to string
  itoa(MassPM1_int, MassPM1ToString, 10);
  itoa(MassPM2_int, MassPM2ToString, 10);
  itoa(MassPM4_int, MassPM4ToString, 10);
  itoa(MassPM10_int, MassPM10ToString, 10);
  itoa(NumPM0_int, NumPM0ToString, 10);
  itoa(NumPM1_int, NumPM1ToString, 10);
  itoa(NumPM2_int, NumPM2ToString, 10);
  itoa(NumPM4_int, NumPM4ToString, 10);
  itoa(NumPM10_int, NumPM10ToString, 10);
  itoa(PartSize_int, PartSizeToString, 10);

  // Send it to the wifi module
  Serial3.write(MassPM1ToString);
  Serial3.write('\r');
  Serial3.write(MassPM2ToString);
  Serial3.write('\r');
  Serial3.write(MassPM4ToString);
  Serial3.write('\r');
  Serial3.write(MassPM10ToString);
  Serial3.write('\r');
  Serial3.write(NumPM0ToString);
  Serial3.write('\r');
  Serial3.write(NumPM1ToString);
  Serial3.write('\r');
  Serial3.write(NumPM2ToString);
  Serial3.write('\r');
  Serial3.write(NumPM4ToString);
  Serial3.write('\r');
  Serial3.write(NumPM10ToString);
  Serial3.write('\r');
  Serial3.write(PartSizeToString);
  Serial3.write('\r');

  return (true);
}

/**
    @brief : continued loop after fatal error
    @param mess : message to display
    @param r : error code

    if r is zero, it will only display the message
*/
void Errorloop(char *mess, uint8_t r)
{
  if (r) ErrtoMess(mess, r);
  else Serial.println(mess);
  Serial.println(F("Program on hold"));
  for (;;) delay(100000);
}

/**
    @brief : display error message
    @param mess : message to display
    @param r : error code

*/
void ErrtoMess(char *mess, uint8_t r)
{
  char buf[80];

  Serial.print(mess);

  sps30.GetErrDescription(r, buf, 80);
  Serial.println(buf);
}

/**
   serialTrigger prints repeated message, then waits for enter
   to come in from the serial port.
*/
void serialTrigger(char * mess)
{
  Serial.println();

  while (!Serial.available()) {
    Serial.println(mess);
    delay(2000);
  }

  while (Serial.available())
    Serial.read();

}

void MicTest ()
{
  // read the input on analog pin 0:
  int sensorValue = analogRead(A0);
  // print out the value you read:
  Serial.println(sensorValue);
  delay(20);        // delay in between reads for stability
}


void runthermalcamera() {
  //Serial.println("GRID EYE IR TEMP >");
  gridtempLow = 0x80;
  averagetemp = 0;

  for (int pixel = 0; pixel < 64; pixel++) {
    Wire.beginTransmission(addr);
    Wire.write(gridtempLow);
    Wire.endTransmission();
    Wire.requestFrom(addr, 2);
    byte lowerLevel = Wire.read();
    byte upperLevel = Wire.read();

    /*
        Wire.beginTransmission(addr);
        Wire.write(gridtempHigh);
        Wire.endTransmission();
        Wire.requestFrom(addr, 1);
        byte upperLevel = Wire.read();
    */

    int temperature = ((upperLevel << 8) | lowerLevel); //  int temperature = temp - 2048;
    if (temperature > 2047) {
      temperature = temperature - 4096; // temperature = -(2048 - temperature);
    }

    gridtempCelsius = temperature * 0.25;
    //Serial.print("Grid Temp in Celsius > ");
    Serial.print(gridtempCelsius);
    Serial.print(",");

    Serial3.print(gridtempCelsius);
    if (pixel != 63) {
      Serial3.print(",");
    }


    // char buff_temp_camera[6];
    // sprintf(buff_temp_camera, 6, "%f, ",gridtempCelsius);
    // arrayCamera[pixel] = buff_temp_camera;


    //    // send to Wifi module
    //    char valString[5];
    //    // parse the float to integer
    //    int tempCam = gridtempCelsius * 100;
    //    // parse integer to string
    //    itoa(tempCam, valString, 10);
    //    Serial3.write(valString);
    //    Serial3.write('\r');

    if ((pixel + 1) % 8 == 0) {
      Serial.print("\r\n");
    }
    switch (pixel) {
      case 27: //pixels
      case 28: //pixels
      case 35: //pixels
      case 36: //pixels
        averagetemp += gridtempCelsius;
        break;
      default:
        break;
    }

    gridtempLow = gridtempLow + 2;
  }

  Wire.beginTransmission(addr);
  Wire.write(0x0E);
  Wire.endTransmission();
  Wire.requestFrom(addr, 2);
  byte uplevel = Wire.read();
  byte lowerLevel = Wire.read();

  /*
    Wire.beginTransmission(addr);
    Wire.write(0x0F);
    Wire.endTransmission();
    Wire.requestFrom(addr, 1);
    byte lowerLevel = Wire.read();
  */
  //   Serial3.write(arrayCamera);
  Serial3.write('\r');

  int tempst = ((lowerLevel << 8) | uplevel);
  float celsiusst = tempst * 0.0625;
  // Serial.print("Celsius > ");
  // Serial.println(celsiusst);
  // Serial.print("\r\n\r\n");
  averagetemp *= 0.25;

  // delay(1000);

}

void waitForSerial() {


  while (!Serial.available()) {
  }
  Serial.println(Serial.read());
}

void scani2cbus() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      //Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

}
