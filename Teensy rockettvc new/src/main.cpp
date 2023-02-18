// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
//      2019-07-08 - Added Auto Calibration and offset generator
//		   - and altered FIFO retrieval sequence to avoid using blocking code
//      2016-04-18 - Eliminated a potential infinite loop
//      2013-05-08 - added seamless Fastwire support
//                 - added note about gyro calibration
//      2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
//      2012-06-20 - improved FIFO overflow handling and simplified read process
//      2012-06-19 - completely rearranged DMP initialization code and simplification
//      2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
//      2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
//      2012-06-05 - add gravity-compensated initial reference frame acceleration output
//                 - add 3D math helper file to DMP6 example sketch
//                 - add Euler output and Yaw/Pitch/Roll output formats
//      2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
//      2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
//      2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "Servo.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include <SPI.h>              // include libraries
#include <RH_RF95.h>


#include <ArduinoJson.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:
   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

//Start custom stuff for the tvc. 
//uncomment #define USE_TVC if you want to use
#define USE_TVC
#define USE_TVCLOOP
#define ROCKET_PROCEDURES
#define ACTUAL_LAUNCH


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


//Servo Setup vars
Servo yawServo;
Servo pitchServo;
int posYaw = 0;
int posPitch = 0;

//PID Loop setup stuff
double SetpointY, InputY, OutputY;
double SetpointP, InputP, OutputP;
float consKp = 4, consKi = 0.2, consKd = 0.1;
PID yawPID(&InputY, &OutputY, &SetpointY, consKp, consKi, consKd, DIRECT);
PID pitchPID(&InputP, &OutputP, &SetpointP, consKp, consKi, consKd, DIRECT);
int yawOffset = 50; //offset of pos from 90 degrees
int pitchOffset = 0; //offset of pos from 90 degrees


//LoRa setup
#define RFM95_CS 10
#define RFM95_RST 9
#define RFM95_INT digitalPinToInterrupt(8)

#define RF95_FREQ 915.0

#define LED 13

RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Singleton instance of the radio driver
//RH_RF95 rf95(RFM95_CS, RFM95_INT);



//altimeter
Adafruit_BMP3XX bmp;
#define SEALEVELPRESSURE_HPA (1013.25)
//#define USELORA


#define PYRO_PIN 3

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



int16_t packetnum = 0;  // packet counter, we increment per xmission



void sendData(String message) {

  Serial.println("Sending to rf95_server");
  // Send a message to rf95_server
  
  //char radiopacket[32] = message;
  //itoa(packetnum++, radiopacket+13, 10);
  //Serial.print("Sending "); Serial.println(radiopacket);
  //radiopacket[31] = 0;
  //message =(char*) message.c_str();
  Serial.println("Sending...");
  rf95.send((uint8_t *)(char*) message.c_str(), message.length());
  
  digitalWrite(LED, HIGH);

  Serial.println("Waiting for packet to complete...");
  //rf95.waitPacketSent();
  // Now wait for a reply
//   uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
//   uint8_t len = sizeof(buf);

  //Serial.println("Waiting for reply...");
//   if (rf95.waitAvailableTimeout(20))
//   { 
//     // Should be a reply message for us now   
//     if (rf95.recv(buf, &len))
//    {
//       Serial.print("Got reply: ");
//       Serial.println((char*)buf);
//       Serial.print("RSSI: ");
//       Serial.println(rf95.lastRssi(), DEC);    
//     }
//     else
//     {
//       Serial.println("Receive failed");
//     }
//   }
//   else
//   {
//     Serial.println("No reply, is there a listener around?");
//   }
  
  
    digitalWrite(LED, LOW);


}


// ================================================================
// ===                      CALIBRATION STUP                    ===
// ================================================================

const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;   // empirical, to hold sampling to 200 Hz
const int NFast =  1000;    // the bigger, the better (but slower)
const int NSlow = 10000;    // ..
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;
      
void ForceHeader()
  { LinesOut = 99; }
    
void GetSmoothed()
  { int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
        mpu.getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz], 
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);
        if ((i % 500) == 0)
          Serial.print(PERIOD);
        delayMicroseconds(usDelay);
        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];
      } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
      { Smoothed[i] = (Sums[i] + N/2) / N ; }
  } // GetSmoothed

void Initialize()
  {
    

    // initialize device
    Serial.println("Initializing I2C devices...");
    mpu.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
    Serial.println("PID tuning Each Dot = 100 readings");
  /*A tidbit on how PID (PI actually) tuning works. 
    When we change the offset in the MPU6050 we can get instant results. This allows us to use Proportional and 
    integral of the PID to discover the ideal offsets. Integral is the key to discovering these offsets, Integral 
    uses the error from set-point (set-point is zero), it takes a fraction of this error (error * ki) and adds it 
    to the integral value. Each reading narrows the error down to the desired offset. The greater the error from 
    set-point, the more we adjust the integral value. The proportional does its part by hiding the noise from the 
    integral math. The Derivative is not used because of the noise and because the sensor is stationary. With the 
    noise removed the integral value lands on a solid offset after just 600 readings. At the end of each set of 100 
    readings, the integral value is used for the actual offsets and the last proportional reading is ignored due to 
    the fact it reacts to any noise.
  */
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println("\nat 600 Readings");
        mpu.PrintActiveOffsets();
        Serial.println();
        mpu.CalibrateAccel(1);
        mpu.CalibrateGyro(1);
        Serial.println("700 Total Readings");
        mpu.PrintActiveOffsets();
        Serial.println();
        mpu.CalibrateAccel(1);
        mpu.CalibrateGyro(1);
        Serial.println("800 Total Readings");
        mpu.PrintActiveOffsets();
        Serial.println();
        mpu.CalibrateAccel(1);
        mpu.CalibrateGyro(1);
        Serial.println("900 Total Readings");
        mpu.PrintActiveOffsets();
        Serial.println();    
        mpu.CalibrateAccel(1);
        mpu.CalibrateGyro(1);
        Serial.println("1000 Total Readings");
        mpu.PrintActiveOffsets();
     Serial.println("\n\n Any of the above offsets will work nice \n\n Lets proof the PID tuning using another method:"); 
  } // Initialize

void SetOffsets(int TheOffsets[6])
  { mpu.setXAccelOffset(TheOffsets [iAx]);
    mpu.setYAccelOffset(TheOffsets [iAy]);
    mpu.setZAccelOffset(TheOffsets [iAz]);
    mpu.setXGyroOffset (TheOffsets [iGx]);
    mpu.setYGyroOffset (TheOffsets [iGy]);
    mpu.setZGyroOffset (TheOffsets [iGz]);
  } // SetOffsets

void ShowProgress()
  { if (LinesOut >= LinesBetweenHeaders)
      { // show header
        Serial.println("\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
      } // show header
    Serial.print(BLANK);
    for (int i = iAx; i <= iGz; i++)
      { Serial.print(LBRACKET);
        Serial.print(LowOffset[i]),
        Serial.print(COMMA);
        Serial.print(HighOffset[i]);
        Serial.print("] --> [");
        Serial.print(LowValue[i]);
        Serial.print(COMMA);
        Serial.print(HighValue[i]);
        if (i == iGz)
          { Serial.println(RBRACKET); }
        else
          { Serial.print("]\t"); }
      }
    LinesOut++;
  } // ShowProgress

void SetAveraging(int NewN)
  { N = NewN;
    Serial.print("averaging ");
    Serial.print(N);
    Serial.println(" readings each time");
   } // SetAveraging


void PullBracketsIn()
  { boolean AllBracketsNarrow;
    boolean StillWorking;
    int NewOffset[6];
  
    Serial.println("\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking) 
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast))
          { SetAveraging(NSlow); }
        else
          { AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working
   
  } // PullBracketsIn

void PullBracketsOut()
  { boolean Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

    Serial.println("expanding:");
    ForceHeader();
 
    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values
      
        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut




















// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately


    #ifdef ROCKET_PROCEDURES

        //altimeter
        if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
        //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode  
        //if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
            Serial.println("Could not find a valid BMP3 sensor, check wiring!");
            //while (1);
        } else {

            bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
        bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_50_HZ);
        if (! bmp.performReading()) {
            Serial.println("Failed to perform reading :(");
            
        } else {
            Serial.print("Temperature = ");
        Serial.print(bmp.temperature);
        Serial.println(" *C");

        Serial.print("Pressure = ");
        Serial.print(bmp.pressure / 100.0);
        Serial.println(" hPa");

        Serial.print("Approx. Altitude = ");
        Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
        Serial.println(" m");

        Serial.println();



        }
        

        }
        


        
        



        StaticJsonDocument<500> doc;
        doc["yaw"] = 0 ;
        doc["pitch"] = 0 ;
        doc["roll"] = 0 ;
        doc["xccel"] = 0 ;
        doc["yaccel"] = 0 ;
        doc["zaccel"] = 0 ;
        doc["yawservo"] = 0 ;
        doc["pitchservo"] = 0 ;
        doc["altitude"] = 0;
        doc["tempurature"] = 0;
        doc["message"] = "None" ;
        serializeJson(doc, Serial);

    #endif



    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    

    

    
    //config pyro
    pinMode(PYRO_PIN, OUTPUT);


    
    //config servos
    #ifdef USE_TVC
      
        // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
    delay(2000);

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    // calibrated 12-21-22
    mpu.setXGyroOffset(-600);
    mpu.setYGyroOffset(-1);
    mpu.setZGyroOffset(45);
    mpu.setXAccelOffset(-705); // 1
    mpu.setYAccelOffset(1229); // 
    mpu.setZAccelOffset(2077); // 

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        Serial.println(ypr[2]);
        Serial.println(ypr[0]);
        Serial.println(ypr[1]);

    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);




      yawServo.attach(20);
      pitchServo.attach(21);
      yawServo.write(140);  // 140 is 90
      pitchServo.write(90);
      //SetpointY = 0;
      //SetpointP = 0;
      yawPID.SetOutputLimits(0, 130);
      pitchPID.SetOutputLimits(30, 130);
      yawPID.SetMode(AUTOMATIC);
      pitchPID.SetMode(AUTOMATIC);

    #endif




    pinMode(LED, OUTPUT);
        digitalWrite(LED,HIGH);   
        pinMode(RFM95_RST, OUTPUT);
        digitalWrite(RFM95_RST, HIGH);
        Serial.begin(9600);
        delay(100);

        Serial.println();

        Serial.println("Gateway Module starting…");

        digitalWrite(RFM95_RST, LOW);
        delay(10);
        digitalWrite(RFM95_RST, HIGH);
        delay(10);

        while (!rf95.init()) {

            Serial.println("LoRa radio init failed");

            while (1);

        }

        Serial.println("LoRa radio init OK!");

        if (!rf95.setFrequency(RF95_FREQ)) {

            Serial.println("setFrequency failed");

            while (1);

        }

        Serial.print("Set Freq to: ");  
        Serial.println(RF95_FREQ);  
        rf95.setTxPower(23, false);
        sendData("asdfsdffffff");






    
}


int lastSendTime = 0;
int interval = 1000;

void testPyro(){
    sendData("PYRO TEST in 5 seconds");
    delay(5000);
    digitalWrite(PYRO_PIN, HIGH);
    delay(500);
    digitalWrite(PYRO_PIN, LOW);


}

void calibrateAccel(){
    Initialize();
    for (int i = iAx; i <= iGz; i++)
      { // set targets and initial guesses
        Target[i] = 0; // must fix for ZAccel 
        HighOffset[i] = 0;
        LowOffset[i] = 0;
      } // set targets and initial guesses
    Target[iAz] = 16384;
    SetAveraging(NFast);
    
    PullBracketsOut();
    PullBracketsIn();
    
    Serial.println("-------------- done --------------");

}

void calibrateTVC(){


}


void sendSecond(String outgoing)
{
	if (millis() - lastSendTime > interval) {
    int firsttime = millis();
    
    
    sendData(outgoing);
    int secondtime = millis();
    Serial.println("Time taken is: ");
    Serial.println((secondtime-firsttime));
    lastSendTime = millis();            // timestamp the message

    //check for incomings
    if (rf95.available())
  {
    // Should be a message for us now   
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    
    if (rf95.recv(buf, &len))
    {
      
      //RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      // Serial.print("RSSI: ");
      //Serial.println(rf95.lastRssi(), DEC);
      char* recieved = (char*)(buf);
      if (strcmp(recieved, (char*)("testPyro")) == 0)
      {
        testPyro();
      } else if (strcmp(recieved, (char*)("calibrateAccel")) == 0)
      {
        calibrateAccel();
      } else if (strcmp(recieved, (char*)("calibrateTVC")) == 0)
      {
        calibrateTVC();
      }
      
      
      
      

      
    }
    else
    {
      Serial.println("Receive failed");
    }
  }
    
    
  }
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //ACTUAL LAUNCH TIME
    #ifdef ACTUAL_LAUNCH
        
        bool launch = false;
        while (launch == false)
        {
            StaticJsonDocument<500> doc;
            //get data
            if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
                 mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                if (! bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    doc["altitude"] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                    doc["tempurature"] = bmp.temperature;
                    doc["message"] = "Ready for Launch" ;

                }
                // Serial.print("ypr\t");
                // Serial.print(ypr[0] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[1] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[2] * 180/M_PI);

                //Serial.print("ypr\t");
                float yaw = ypr[2] * 180/M_PI;
                
                float pitch = ypr[1] * 180/M_PI;
                
                float roll = ypr[1] * 180/M_PI;


                doc["yaw"] = yaw ;
                doc["pitch"] = pitch;
                doc["roll"] = roll;
                doc["xccel"] = aaWorld.x ;
                doc["yaccel"] = aaWorld.y ;
                doc["zaccel"] = aaWorld.z ;
                String preoutput = "";
                serializeJson(doc, preoutput);
                sendData(preoutput);
            }
            
            if (rf95.available())
            {
                // Should be a message for us now   
                uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
                uint8_t len = sizeof(buf);
                
                if (rf95.recv(buf, &len))
                {
                
                //RH_RF95::printBuffer("Received: ", buf, len);
                Serial.print("Got: ");
                Serial.println((char*)buf);
                // Serial.print("RSSI: ");
                //Serial.println(rf95.lastRssi(), DEC);
                char* recieved = (char*)(buf);
                if (strcmp(recieved, (char*)("testPyro")) == 0)
                {
                    testPyro();
                } else if (strcmp(recieved, (char*)("calibrateAccel")) == 0)
                {
                    calibrateAccel();
                } else if (strcmp(recieved, (char*)("calibrateTVC")) == 0)
                {
                    calibrateTVC();
                } else if (strcmp(recieved, (char*)("launch")))
                {
                    launch = true;
                }
                
                
                
                
                

                
                }
                else
                {
                Serial.println("Receive failed");
                }
            }

            delay(100);
        }
        
        
    #endif



    StaticJsonDocument<500> doc;
    //Serial.print("start loop 2-2");
    // if programming failed, don't try to do anything
    //if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        //Serial.println("In loop");
        #ifdef OUTPUT_READABLE_QUATERNION
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            Serial.print("quat\t");
            Serial.print(q.w);
            Serial.print("\t");
            Serial.print(q.x);
            Serial.print("\t");
            Serial.print(q.y);
            Serial.print("\t");
            Serial.println(q.z);
        #endif

        #ifdef OUTPUT_READABLE_EULER
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetEuler(euler, &q);
            Serial.print("euler\t");
            Serial.print(euler[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(euler[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(euler[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        #ifdef OUTPUT_READABLE_REALACCEL
            // display real acceleration, adjusted to remove gravity
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            Serial.print("areal\t");
            Serial.print(aaReal.x);
            Serial.print("\t");
            Serial.print(aaReal.y);
            Serial.print("\t");
            Serial.println(aaReal.z);
        #endif

        #ifdef OUTPUT_READABLE_WORLDACCEL
            // display initial world-frame acceleration, adjusted to remove gravity
            // and rotated based on known orientation from quaternion
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
            Serial.print("aworld\t");
            Serial.print(aaWorld.x);
            Serial.print("\t");
            Serial.print(aaWorld.y);
            Serial.print("\t");
            Serial.println(aaWorld.z);
        #endif
    
        #ifdef OUTPUT_TEAPOT
            // display quaternion values in InvenSense Teapot demo format:
            teapotPacket[2] = fifoBuffer[0];
            teapotPacket[3] = fifoBuffer[1];
            teapotPacket[4] = fifoBuffer[4];
            teapotPacket[5] = fifoBuffer[5];
            teapotPacket[6] = fifoBuffer[8];
            teapotPacket[7] = fifoBuffer[9];
            teapotPacket[8] = fifoBuffer[12];
            teapotPacket[9] = fifoBuffer[13];
            Serial.write(teapotPacket, 14);
            teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
        #endif
        

        #ifdef USE_TVCLOOP
                
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                
                // Serial.print("ypr\t");
                // Serial.print(ypr[0] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[1] * 180/M_PI);
                // Serial.print("\t");
                // Serial.print(ypr[2] * 180/M_PI);

                //Serial.print("ypr\t");
                float yaw = ypr[2] * 180/M_PI;
                
                float pitch = ypr[1] * 180/M_PI;
                
                float roll = ypr[1] * 180/M_PI;

                float yservopos = yawOffset +90-yaw;
                if ((yservopos < 100) | (yservopos >153))
                {
                    Serial.print("Yaw out of range!!");
                } else
                {
                    yawServo.write(yservopos);
                }
                
                //Serial.print("yaw"); // limits 153 front and 100 back
                //Serial.print(yservopos);
                //Serial.print("\n");
                
                float pservopos = pitchOffset +90+pitch;
                if ((pservopos < 81) | (pservopos >109))
                {
                    Serial.print("Pitch out of range!!");
                } else
                {
                    pitchServo.write(pservopos);
                }
                
                //Serial.print("pitch"); // limits 81 left and 109 right
                //Serial.print(pservopos);
                //Serial.print("\n");
                doc["yaw"] = yaw ;
                doc["pitch"] = pitch;
                doc["roll"] = roll;
                doc["xccel"] = aaWorld.x ;
                doc["yaccel"] = aaWorld.y ;
                doc["zaccel"] = aaWorld.z ;
                doc["yawservo"] = yservopos ;
                doc["pitchservo"] = pservopos ;
                String loraoutput = "fff";
                serializeJson(doc, Serial);
                //serializeJson(doc, loraoutput);
                //sendData("Helloooo");
                


                //onReceive(LoRa.parsePacket());
                #endif
        
    } else {
        Serial.println("no good data");
        //sendData("Helloooo");
    }
    

    #ifdef ROCKET_PROCEDURES
                //bmp setup
                
                 if (! bmp.performReading()) {
                    Serial.println("Failed to perform reading :(");
                    
                } else {
                    doc["altitude"] = bmp.readAltitude(SEALEVELPRESSURE_HPA);
                    doc["tempurature"] = bmp.temperature;
                    doc["message"] = "Nominal" ;

                }
                
            

                // blink LED to indicate activity
                Serial.println("start procedure");
                blinkState = !blinkState;
                digitalWrite(LED_PIN, blinkState);
                String loraoutput = "";
                serializeJson(doc, loraoutput);
                Serial.println(loraoutput);
                sendSecond(loraoutput);
    #endif


}

