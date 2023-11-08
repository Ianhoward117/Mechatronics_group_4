
#include <AStar32U4Motors.h>
#include <Encoder.h>

AStar32U4Motors m; //read the documentation of this library to understand what functions to use to drive the motors and how to use them

#define PI 3.141592653589

int leftMotor; // COMMANDED MOTOR SPEEDS
int rightMotor;

double leftMotorMax = 22.85; // **students should find this variable themselves**
double rightMotorMax = 21.44;

const int encoderRightPinA = 11;
const int encoderRightPinB = 8;

const int encoderLeftPinA = 15; 
const int encoderLeftPinB = 16;

Encoder encoderRight(encoderRightPinA,encoderRightPinB);
Encoder encoderLeft(encoderLeftPinA,encoderLeftPinB);

double encoderResolution = 1440; // counts per rev
double d = 2.7559055; //wheel diameter in inches

int posLeftCount = 0;
int posRightCount = 0;
int posLeftCountLast = 0;
int posRightCountLast = 0;
double posLeftRad = 0.0; // this will need to be converted to rad/sec
double posRightRad = 0.0; // this will need to be converted to rad/sec
double velLeft = 0; // this will be omegaLeft*d/2;
double velRight = 0; // this will be omegaRight*d/2 will be in inches per sec;
double newVelLeft = 0; // this will be omegaLeft*d/2;
double newVelRight = 0; // this will be omegaRight*d/2 will be in inches per sec;
double rev_per_sec_right = 0;
double rev_per_sec_left = 0;

// MOTOR LOOP CONSTANTS
double interval = 5.0; // 5 ms means 200Hz loop
unsigned long previousMillis = 0;
unsigned long priorTimeL,priorTimeR; // We need to keep track of time for each PID controller separately
double lastSpeedErrorL,lastSpeedErrorR; //same with error
double cumErrorL, cumErrorR;
double maxErr = 20; // chosen arbitrarily for now, students can tune. 
double desVelL; // will be in inches per sec
double desVelR;

// PID CONSTANTS
// LEFT MOTOR - you need to find values. FYI I found good responses with Kp ~ 10x bigger than Ki, and ~3x bigger than Kd. My biggest value was <2.
double kpL = 2.4;
double kiL = .7;
double kdL = .9;
// Right MOTOR - assumes we need to tune them differently
double kpR = 2.5;
double kiR = .8;
double kdR = 1;                                                                                                                                                                                                                                ;

const byte numChars = 32;
char receivedChars[numChars];
char tempChar[numChars]; // temporary array used for parsing
boolean newData = false;

//=====================================================

void setup() {
  Serial.begin(115200); 
 

}

void loop() {

      unsigned long currentMillis = millis();

      posRightCount = encoderRight.read();
      posLeftCount = encoderLeft.read();

      if (currentMillis - previousMillis >= interval)
          {
            recvWithStartEndMarkers(); //this function is in charge of taking a peice of data that looks like <17,16> 
                                       //turning it into a string looking like 17,16 and then setting newdata to true,
                                       //letting the rest of the program know a packet of data is ready to be analyzed, does all this without blocking
            if (newData == true) //newData will be true when recvWithStartEndMarkers(); has finished recieving a whole set of data from Rpi (a set of data is denoted as being containted between <>)
              { 
              strcpy(tempChar, receivedChars); //this line makes a copy of recievedChars for parsing in parseData, I do this becasue strtok() will alter any string I give it,I want to preserve the origonal data
              parseData(); //right now parseData only parses a string of 2 numbers seperated by commas into float so the string 17.5,16 becomes two floats; 17.5 and 16   
              newData = false;
              }
            
            previousMillis = currentMillis;
            
            rev_per_sec_right = 1000 * (posRightCount - posRightCountLast) / encoderResolution / interval;
            rev_per_sec_left = 1000 * (posLeftCount - posLeftCountLast) / encoderResolution / interval;
            
            posRightRad = rev_per_sec_right * 2 * PI; // Write expression to get Rad/sec. Pi is defined above FYI.
            posLeftRad = rev_per_sec_left * 2 * PI; // Same - Rad/sec
            
            velRight = rev_per_sec_right * d * PI; // Now convert to get inches/sec (tangential velocity)
            velLeft = rev_per_sec_left * d * PI; // Same - Inches/sec
            
            newVelRight = drivePIDR(velRight);
            newVelLeft = drivePIDL(velLeft);
          
            Serial.print("RIGHT: ");
            Serial.print(velRight);
            Serial.print(',');
            Serial.print(newVelRight);
            Serial.print("  ===  LEFT: ");
            Serial.print(velLeft);
            Serial.print(',');
            Serial.println(newVelLeft);
          
            rightMotor = motorVelToSpeedCommand(newVelRight,rightMotorMax);
            leftMotor = motorVelToSpeedCommand(newVelLeft,leftMotorMax);
            /// COMMENT OUT TO HERE FOR FINDING MAX MOTOR SPEED AT 400, You need to add the print statements to get the max speed. 
            
            posRightCountLast = posRightCount;
            posLeftCountLast = posLeftCount;
            
            CommandMotors();
          }
}

void parseData(){


  char *strtokIndexer; //doing char * allows strtok to increment across my string properly frankly im not sure why... something to do with pointers that I dont expect students to understand

  
  strtokIndexer = strtok(tempChar,","); //sets strtokIndexer = to everything up to the first comma in tempChar /0 //this line is broken
  desVelL = atoi(strtokIndexer); //converts strtokIndexer into a int
  

  strtokIndexer= strtok(NULL, ","); //setting the first input to null causes strtok to continue looking for commas in tempChar starting from where it left off, im not really sure why 
  desVelR = atoi(strtokIndexer);

  
  //now that we have extracted the data from the Rpi as floats, we can use them to command actuators somewhere else in the code
  
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
                                                               
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();
                                                             
                                                                  
        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminates the string, frankly unsure why I need this
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void CommandMotors()
{  
  //read the documentation for the functions that drive the motors in the astar library
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
}

double drivePIDL(double curr){
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;
  
    currentTime = millis();                               //get current time
    elapsedTime = (double)(currentTime - priorTimeL);     // compute elasped time for this control period

    error = desVelL - curr;                               // Error
    cumErrorL += error*elapsedTime;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorL>maxErr)
    cumErrorL = maxErr;
    else if (cumErrorL<-1*maxErr)
      cumErrorL = -1*maxErr;

    rateError = (error-lastSpeedErrorL)/elapsedTime;      // Derivative Error

    double out = kpL*error+kiL*cumErrorL+kdL*rateError;   // PID output

    lastSpeedErrorL = error;                              // remember current error
    priorTimeL = currentTime;                             // remember current time
    return out;                                           // return the needed motor speed. 
}

double drivePIDR(double curr){
    double rateError;
    double error;
    unsigned long currentTime;
    unsigned long elapsedTime;
  
    currentTime = millis();                               //get current time
    elapsedTime = (double)(currentTime - priorTimeR);      // compute elasped time for this control period

    error = desVelR - curr;                                   // Error
    cumErrorR += error*elapsedTime;                       // Cumulative Error(since we add this outside the loop, needs to be unique to the motor controlled)

    // INTEGRAL WINDUP                                    // REMOVE WINDUP
    if(cumErrorR>maxErr)
    cumErrorR = maxErr;
    else if (cumErrorR<-1*maxErr)
      cumErrorR = -1*maxErr;

    rateError = (error-lastSpeedErrorR)/elapsedTime;      // Derivative Error

    double out = kpR*error+kiR*cumErrorR+kdR*rateError;   // PID output

    lastSpeedErrorR = error;                              // remember current error
    priorTimeR = currentTime;                             // remember current time
    return out;                                           // return the needed motor speed. 
}

int motorVelToSpeedCommand(double Vel, double maxVel){
    int newSpeed = 0;
    Vel = constrain(Vel,-1*maxVel, maxVel);
    newSpeed = map(Vel,-1*maxVel, maxVel, -400, 400);
    return newSpeed;
}
