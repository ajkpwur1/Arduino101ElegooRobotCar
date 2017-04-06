#include <BMI160.h>
#include <CurieIMU.h>
#include <MadgwickAHRS.h>
#include <Servo.h> //servo library

#include <CurieBLE.h>

Servo obstacleAvoidServo; // create servo object to control servo
//Obstacle Avoidance
int Echo = A0;  
int Trig = A5; 
volatile float Distance = 100;
volatile int echopulseStart;
volatile int echopulseEnd = 0;
volatile int echopulseLength = 0;
volatile int obstAvoidState = 0; //0=Ready 1=Wait Echo Pulse Rise 2=Wait Echo Pulse Fall

const int ledPin = 13; // set ledPin to use on-board LED
const char F = '2';//Forward
const char B = '6';//Back
const char L = '4';//Left
const char r = '3';//right
const char l = '1';//left
const char R = '5';//Right
const char S = '0';//Stop
const char M = '7';//Memory
const char G = '9';//Go
const char I = 'I';//Idle
const char P = 'P';//Pivot
const char attainingLeftAngle = 'L';
const char attainingRightAngle = 'R';
const boolean LEFT = 0;
const boolean RIGHT =1;
char event = S;   // variable to hold a transmitted byte set to 0 
String state = "Idle";  

char recallState = I;

long loopCount ;
static float targetHeading;
static float headingCmd;
static float speedCmd;
static float servoCmd;
static float leftMotor;
static float rightMotor;
static float leftMotorCmd;
static float rightMotorCmd;
static float headCmdIErrorUpv = 0;
static float headCmdIErrorYpv = 0;
static float headCmdDErrorXpv = 0;

static float driftCompensationUpv;
static float driftCompensationYpv;
static float headingCmdFiltered ;

static float headingCmdFilteredpv = 0;
static float headingCmdFilterAlpha;
static float driftFilterAlpha;
static float baseHeading;
static float headingpv;
static float headingCorrected;
static float headingDriftFiltered;
static float headingDriftFilteredpv;
boolean feedBackEnable; 

BLEPeripheral blePeripheral;  // BLE Peripheral Device (the board you're programming)

// ====  create Nordic Semiconductor UART service =========
BLEService uartService = BLEService("6E400001B5A3F393E0A9E50E24DCCA9E");
// create characteristics
BLECharacteristic rxCharacteristic = BLECharacteristic("6E400002B5A3F393E0A9E50E24DCCA9E", BLEWriteWithoutResponse, 20);  // == TX on central (android app)
BLECharacteristic txCharacteristic = BLECharacteristic("6E400003B5A3F393E0A9E50E24DCCA9E", BLENotify , 20); // == RX on central (android app)


Madgwick filter;
unsigned long microsPerReading, microsPrevious;
float accelScale, gyroScale;

//     headingDesired = modAdd( baseHeading , headingCmdFiltered, 360 );
float modAdd( float op1 , float op2, float mod )
//first value is always 0 or more.
//second number may be negative.

{ float temp;
  int nonFrac;
  if ( op2 < 0 ) {
     // Convert the subtract into a addition
     // Mixed number  
     temp = (0 - op2) / 360;
      // get the real.
     nonFrac = (int) temp;
     // get fraction and get to negative degrees
     op2 = (temp - nonFrac) * mod;
     // convert the negative op2 to positive
     op2 = mod - op2;  
  } 
   // Execute positive algorithm.
    temp = op1 + op2 ;
    temp = temp / mod ;
    nonFrac = (int) temp;
    return (temp-nonFrac)*mod ;
  
}
  
float ComputeError( float heading , float headingDesired) {
float  headCmdErrorCCW;
float headCmdErrorCW;
float headCmdError;
          // Things are reversed on angle... the Curie IMU must be mounted upside down on the board.
    
          // See Book1 Page 10 of Maker Notes for graphics
    
          if (heading > headingDesired ){
            headCmdErrorCCW = abs((360 - heading) + headingDesired);
            headCmdErrorCW = abs(heading -headingDesired);
            headCmdError = min(headCmdErrorCW, headCmdErrorCCW);
            if (headCmdError == headCmdErrorCW) { headCmdError = -1 * headCmdError;}
          } else { 
            headCmdErrorCCW = abs((360 - headingDesired ) + heading);
            headCmdErrorCW = abs( headingDesired - heading);
            headCmdError = min(headCmdErrorCW, headCmdErrorCCW);
            if (headCmdError == headCmdErrorCCW) { headCmdError = -1 *headCmdError;}
          }   
         /*Serial.print(" headCmdErrorCW :");
         Serial.print( headCmdErrorCW);  
         Serial.print(" headCmdErrorCCW:");
         Serial.print( headCmdErrorCCW);
         Serial.print(" heading :");
         Serial.print( heading);
         */
     return headCmdError;
}
void _mDrive(int  MotorRight, int MotorLeft )
{

//enA Green
//in1 Blue
//in2 Brown 
//enB Black
//in3 Gray
//in4 White

//Motor Right
int enA = 6; ////Changed from 10 to 11  Rewired. 
int in1 = 7;
int in2 = 8;
//motor Left
int enB = 5;
int in3 = 10;
int in4 = 11;
int AB = 200;
int OFF = 0;
// Limit the Motor commands.

if ( MotorRight >= 250 ) { MotorRight = 250;
} else  if ( MotorRight <= -250 ) {MotorRight = -250;
}
if ( MotorLeft >= 250 ) {MotorLeft = 250;
} else if (MotorLeft <= -250 ) {MotorLeft = -250;}

Serial.print(" MotorLeft:");
Serial.print(MotorLeft);
Serial.print(" MotorRight:");
Serial.println(MotorRight); 
//Forward Only
 if ((MotorRight > 0) && (MotorLeft > 0 )) 
 {
 digitalWrite(in1, LOW);
 digitalWrite(in2, HIGH);
 digitalWrite(in3, LOW);
 digitalWrite(in4, HIGH);
 analogWrite(enA, MotorRight);
 analogWrite(enB, MotorLeft);
// Serial.println("going forward!");
 
 //Right Turn
 } else if ((MotorRight > 0) && (MotorLeft < 0 ) )
 {
 digitalWrite(in1,LOW);
 digitalWrite(in2,HIGH);
 digitalWrite(in3,HIGH);
 digitalWrite(in4,LOW);
 analogWrite(enA, MotorRight);
 analogWrite(enB, (0 - MotorLeft));
 Serial.println("Aggresive right!");
 //Left Turn
 } else if ((MotorRight < 0) && (MotorLeft > 0 ) )
 {
 digitalWrite(in1,HIGH);
 digitalWrite(in2,LOW);
 digitalWrite(in3,LOW);
 digitalWrite(in4,HIGH);
 analogWrite(enA, (0-MotorRight));
 analogWrite(enB, MotorLeft);
 Serial.println("Aggresive left!");

 } else if ((MotorRight < 0) && (MotorLeft < 0 ) )
 {
 digitalWrite(in1,HIGH);
 digitalWrite(in2,LOW);
 digitalWrite(in3,HIGH);
 digitalWrite(in4,LOW);
 analogWrite(enA, (0-MotorRight));
 analogWrite(enB, (0-MotorLeft));
Serial.println("going Back!");

 } else {
  analogWrite(enA,OFF);
  analogWrite(enB,OFF);
  Serial.println("Stopped!");
}  
}


 /*Ultrasonic distance measurement Sub function*/
int Distance_Measure_Start() 
// This runs to kicks off a measurement of Distance if Ready
{
  if ( obstAvoidState > 0 ) {
     Distance = 10000;
  }
  digitalWrite(Trig, LOW);   
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);  
  delayMicroseconds(20);
  digitalWrite(Trig, LOW); 
  attachInterrupt(digitalPinToInterrupt(Echo), Distance_EchoPulse_Start, RISING);  
  obstAvoidState = 1;
}  

 /*Ultrasonic distance measurement Sub function*/
void  Distance_EchoPulse_Start()
//This detects the Echo Pulse Going High   
{
  if ( obstAvoidState == 1 ) {
    echopulseStart = micros();
    attachInterrupt(digitalPinToInterrupt(Echo), Distance_Echo_Compute, FALLING);
   obstAvoidState = 2;
  }
}  

 /*Ultrasonic distance measurement Sub function*/
void Distance_Echo_Compute()   
{
  if ( obstAvoidState == 2 ) {
    echopulseEnd = micros();
    if (echopulseEnd > echopulseStart ) { 
        echopulseLength = echopulseEnd- echopulseStart;  
        Distance =  echopulseLength /58; 
    }
    obstAvoidState = 0;  
  } 
} 


void setup() {
  Serial.begin(19200);
  pinMode(ledPin, OUTPUT); // use the LED on pin 13 as an output
  obstacleAvoidServo.attach(3);// attach servo on pin 3 to servo object

  
  // set advertised local name and service UUID:
  blePeripheral.setLocalName("BLE_ROV");
  blePeripheral.setAdvertisedServiceUuid(uartService.uuid());

  // add service, rx and tx characteristics:
  blePeripheral.addAttribute(uartService);
  blePeripheral.addAttribute(rxCharacteristic);
  blePeripheral.addAttribute(txCharacteristic);

  // assign event handlers for connected, disconnected to peripheral
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);

  // assign event handler for characteristic
  rxCharacteristic.setEventHandler(BLEWritten, rxCharacteristicWritten);

  // advertise the service
  blePeripheral.begin();


      // start the IMU and filter
   CurieIMU.begin();
   CurieIMU.setGyroRate(25);
   CurieIMU.setAccelerometerRate(25);
   filter.begin(25);

   // Set the accelerometer range to 2G
   CurieIMU.setAccelerometerRange(2);
   // Set the gyroscope range to 250 degrees/second
   CurieIMU.setGyroRange(250);



   // initialize variables to pace updates to correct rate
   microsPerReading = 1000000 / 25; 
   microsPrevious = micros();
   
   feedBackEnable = true; 
  //Obstacle Avoidance
  pinMode(Echo, INPUT);    
  pinMode(Trig, OUTPUT);  
  
}

void loop() {
  int aix, aiy, aiz;
   int gix, giy, giz;
   float ax, ay, az;
   float gx, gy, gz;
   float roll, pitch, heading;
   float headingDrift;
   unsigned long microsNow;


   float headCmdErrorIntegralGain;
   float headCmdErrorProportionalGain;
   float headCmdErrorDerivativeGain;
   float integratorControlGain;
 
   float Period;
   float headingCmdFilterTC;
   float driftFilterTC;

   float  headingDesired;
   float headingCmdHeadingDiff;
   float headingCmdDiff;
   float headingCmdHeadingDiffNegComp;
   float headingCmdDiffNegComp;  
   float headCmdError;
   float headingCmdLimited;
   float directionVal;


   float headCmdIErrorX;
   float headCmdIErrorU;
   float headCmdIErrorY;
   float headCmdPErrorY;
   float headCmdDErrorY;
   float driftCompensation;
   float servoAngle;
 

 
  // poll ble peripheral
  blePeripheral.poll();  

   // check if it's time to read data and update the filter
    microsNow = micros();
   if (microsNow - microsPrevious >= microsPerReading) {

     // read raw data from CurieIMU
     CurieIMU.readMotionSensor(aix, aiy, aiz, gix, giy, giz);

     // convert from raw data to gravity and degrees/second units
     ax = convertRawAcceleration(aix);
     ay = convertRawAcceleration(aiy);
     az = convertRawAcceleration(aiz);
     gx = convertRawGyro(gix);
     gy = convertRawGyro(giy);
     gz = convertRawGyro(giz);

     // update the filter, which computes orientation
     filter.updateIMU(gx, gy, gz, ax, ay, az);

     // print the heading, pitch and roll
     roll = filter.getRoll();
     pitch = filter.getPitch();
     heading = filter.getYaw();
 //    headingCompensated = driftCompensation 


 Serial.print("E:");
 Serial.print(event); 
 Serial.print(" S:");
 Serial.print(state); 

  
 // Obstacle Avoidance    
      if(state == "Forward" )
      {
         if (Distance < 30 ) { event = S ; }          
     }

    
    Serial.print(" obstAvoidState=");
    Serial.print(obstAvoidState);
    Serial.print(" echopulseStart=");
    Serial.print(echopulseStart);
    Serial.print(" echopulseEnd=");
    Serial.print(echopulseEnd);
    Serial.print(" echopulseLength=");
    Serial.print(echopulseLength);
    Serial.print(" Distance=");
    Serial.print(Distance);
   
  
  if (event == F)
    {  
       speedCmd = 10; //cmsec
       headingCmdFilteredpv = 0;
       headingCmd =  0; 
       servoCmd = 0;
       baseHeading = heading;
       state = "Forward";
     
    } else if( event == R  ) 
    {         
         
         headingCmd = -90; 
         servoCmd = -45;
         baseHeading = heading;
         headingCmdFilteredpv = 0; 
         Serial.println("Right");
         if ( state == "Stop" ) { state = "Pivot"; };
    
    } else if (event == L ) 
    {
          headingCmd =  90; 
          servoCmd = 45;
          baseHeading = heading;
          headingCmdFilteredpv = 0;
          Serial.println("Left"); 
          if ( state == "Stop") { state = "Pivot"; };
    } else if ( event == r  ) 
    {   

         headingCmd =  -30;
         servoCmd = -30;
         baseHeading = heading;
         headingCmdFilteredpv = 0;
         Serial.println("Right");
         if ( state == "Stop") { state = "Pivot"; };
    } else if ( event == l ) 
    {
        headingCmd =  30; 
        servoCmd = 30;
        baseHeading = heading;
        headingCmdFilteredpv = 0;
        Serial.println("left"); 
        if ( state == "Stop") { state = "Pivot"; };
    }else if (event == B) 
    {
         //_mBack();
           headingCmd =  0;
           baseHeading = heading; 
           speedCmd = -10; //cm per sec
          headingCmdFilteredpv = 0;
          state = "Back";
    }  else if (event == S  )
    {
         //_mStop();
         headingCmd =  0;
         baseHeading = heading;
         headingpv = heading; 
         headingDriftFilteredpv = 0;
         headingDriftFiltered = 0;
         speedCmd = 0;     
         headCmdIErrorUpv = 0;
         headCmdIErrorYpv = 0;
         headingCmdFilteredpv = 0; 
         state = "Stop";      
    } 


    if(state == "Stop" )
    {
         headingCmd =  0;
         baseHeading = heading;
         headingCorrected = heading;      
         speedCmd = 0;  
         headCmdIErrorUpv = 0;
         headCmdIErrorYpv = 0;
         headingCmdFilteredpv = 0;
         headingDrift = ComputeError(heading,headingpv);
         //Low pass Filter for Drift.
         Period = 0.04;
           // y[i] := y[i-1] + α * (x[i] - y[i-1])
            // This processing is active until headingCmd is equal to it output. 
            //  α = Period / (Timeconstant + Period)
            //Cmd filter
         driftFilterTC = 1;
         driftFilterAlpha = Period / ( driftFilterTC + Period);
         headingDriftFiltered = headingDriftFilteredpv + driftFilterAlpha * ( headingDrift  - headingDriftFilteredpv) ;
         headingDriftFilteredpv =   headingDriftFiltered;
         headingpv = heading;
         Serial.print(" headingDrift :");
         Serial.print( headingDrift);   
      

                   
    } else if (state == "Pivot") {
      if (headingCmd < 0 ) { servoCmd = -90; }
      else {  servoCmd = 90; }
   } else if (state != "Stop") {

    headingCorrected=  modAdd(  headingCorrected ,headingDriftFiltered , 360 );
   }
    Serial.print(" headingDriftFiltered :");
    Serial.print( headingDriftFiltered); 
    Serial.print(" headingCorrected :");
    Serial.print( headingCorrected);
   

    // Second state machine for Memorizing and Playing Being used for feedbackEnable 
    if ((event  == M) )
    {  

         feedBackEnable = true;
         recallState = M;
         Serial.println("Activate Memory");

    }  else if ((event == G))
    {  
    
         feedBackEnable = false;
         recallState = G;
         Serial.println("Activate Go");

    }     
  
     event = I; // Event is set to Idle after processed. 

     Serial.print(" heading :");
     Serial.print( heading);   
     Serial.print(" headingCmd :");
     Serial.print( headingCmd);
     Period = 0.04;
       // y[i] := y[i-1] + α * (x[i] - y[i-1])
        // This processing is active until headingCmd is equal to it output. 
        //  α = Period / (Timeconstant + Period)
        //Cmd filter
     headingCmdFilterTC = .5;
     headingCmdFilterAlpha = Period / ( headingCmdFilterTC + Period);
     headingCmdFiltered = headingCmdFilteredpv + headingCmdFilterAlpha * ( headingCmd  - headingCmdFilteredpv) ;
     headingCmdFilteredpv =   headingCmdFiltered;
     //Serial.print(" headingCmdFilterAlpha :");
     //Serial.print( headingCmdFilterAlpha);
     Serial.print(" headingCmdFiltered :");
     Serial.print( headingCmdFiltered);  
     Serial.print(" baseHeading:");
     Serial.print( baseHeading);
     

     // Do a Mod 360 Add    
     headingDesired = modAdd( baseHeading , headingCmdFiltered, 360 );    
     Serial.print(" headingDesired:");
     Serial.print( headingDesired);
     if (feedBackEnable) {
           Serial.print("Fbk On ");
           headCmdError = ComputeError(heading,headingDesired);
           servoCmd = headCmdError;    

         // x(n) = y(n-1) + K*T/2 * u(n-1)  K=1  T=.04sec    
         //varX = varYpv + (varGain*(Period/2) * varUpv)  
         //varUpv = varU
         //                                                 
         // y(n) = x(n) + K*T/2*u(n) 
         //varY = varX + (varGain*(Period/2)*varU)                          
         //varYpv = varY 
         //if ( varYpv > varUpLim ) {
         // varYpv = varUpLim;
         //} elseif (varYpv < varLowLim ){
         // varYpv = varLowLim;
         //}
    
         // x(n) = y(n-1) + K*T/2 * u(n-1)  K=1  T=.04sec  
    
         
         headCmdErrorIntegralGain = 1;
         headCmdErrorProportionalGain = 5;
         headCmdErrorDerivativeGain = .2;
         Period = 0.04;
         Serial.print(" headCmdIEreXUpv :");
         Serial.print( headCmdIErrorUpv);  
         headCmdIErrorX = headCmdIErrorYpv + (headCmdErrorIntegralGain*(Period/2) * headCmdIErrorUpv);  
         headCmdIErrorUpv = headCmdError;
         
         /*Serial.print(" headCmdIEreX :");
         Serial.print( headCmdIErrorX);
         Serial.print(" headCmdIErrYpv :");
         Serial.print( headCmdIErrorYpv); 
         */
          
         //                                                 
         // y(n) = x(n) + K*T/2*u(n) 
         headCmdIErrorY = headCmdIErrorX + (headCmdErrorIntegralGain*(Period/2)*headCmdError);                       
         headCmdIErrorYpv = headCmdIErrorY; 
         // Limint between -250 and 250
         if ( headCmdIErrorYpv > 250 ) {
             headCmdIErrorYpv = 250;}
         else if (headCmdIErrorYpv < -250 ) {
             headCmdIErrorYpv = -250;
         }
         Serial.print(" headCmdIErrYpv :");
         Serial.print( headCmdIErrorYpv);  


          // Proportional Compensation
         headCmdPErrorY = headCmdErrorProportionalGain * headCmdError;


         // Derivative Compensation
         //y(n) = K*x(n)/T – K*x(n-1)/T;  K=0

         headCmdDErrorY = (headCmdErrorDerivativeGain * headCmdError / Period )-  (headCmdErrorDerivativeGain * headCmdDErrorXpv / Period );
         headCmdDErrorXpv = headCmdError;
         
         headingCmdLimited = headCmdIErrorY + headCmdPErrorY + headCmdDErrorY;
         /*Serial.print("headCmdIErrorY :");
         Serial.print( headCmdIErrorY);
         Serial.print("headCmdPErrorY :");
         Serial.print( headCmdPErrorY);
         Serial.print("headCmdDErrorY :");
         Serial.print( headCmdDErrorY);
         Serial.print("headCmdLim :");
         Serial.print( headingCmdLimited);
         */
     } else {
         Serial.print("Fbk Off ");
         headingCmdLimited = headingCmd;  
     }

     servoAngle = map ( (servoCmd), -90, 90, 0, 175);
     obstacleAvoidServo.write(servoAngle);//setservo position according to scaled value 

     leftMotor = speedCmd * 20;  //convert cm per sec to DA value
     rightMotor = speedCmd *20;   //convert cm per sec to DA value
     leftMotorCmd = leftMotor + headingCmdLimited;
     rightMotorCmd = rightMotor  - headingCmdLimited;

    _mDrive(leftMotorCmd,rightMotorCmd );

     Distance_Measure_Start();



     
     // increment previous time, so we keep proper pace
     microsPrevious = microsPrevious + microsPerReading;

     // Determine the drift in one second while system is stationary

     //Serial.print("heading: ");
     //Serial.print(heading);
     //Serial.print("driftComp: ");
     //Serial.print(driftCompensation);
     //Serial.print("headingCmd:");
     //Serial.print(headingCmd);

  
 } else {  Serial.print("Timing not met!");};
 
}

void blePeripheralConnectHandler(BLECentral& central) {
  // central connected event handler
  Serial.print("Connected event, central: ");
  Serial.println(central.address());
  //Serial.println("LED on");
  digitalWrite(ledPin, HIGH);
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  // central disconnected event handler
  Serial.print("Disconnected event, central: ");
  Serial.println(central.address());
  //Serial.println("LED off");
  digitalWrite(ledPin, LOW);
}

void rxCharacteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  // central wrote new value to characteristic, update LED
  Serial.print("Characteristic event, written: ");
  
  if (characteristic.value()) {       //null pointer check
    event = *characteristic.value();  //set state to be the value written from the phone/tablet to the Arduino 101
    Serial.println(char(event));      //print out the character to the serial monitor
    
  } 
}

float convertRawAcceleration(int aRaw) {
   // since we are using 2G range
   // -2g maps to a raw value of -32768
   // +2g maps to a raw value of 32767
   
   float a = (aRaw * 2.0) / 32768.0;
   return a;
}

float convertRawGyro(int gRaw) {
   // since we are using 250 degrees/seconds range
   // -250 maps to a raw value of -32768
   // +250 maps to a raw value of 32767
   
   float g = (gRaw * 250.0) / 32768.0;
   return g;
}
