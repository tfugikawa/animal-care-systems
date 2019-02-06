// Global Vars
unsigned long numSteps = 20000; // Microstepping
unsigned int stepInterval = 100; // Used for delayMicroseconds, controls max speed (30 degrees per second)
unsigned int varRate = 800;//62500;//Acceleration function used for rampup and rampdown (125000/2) This is a 125 ms frequency starting pulse (only moves 10 steps)
//const int accelRate = 2000; // Number of milliseconds for acceleration, used also for decceleration
boolean eStop = false; // Logic for determining whether or not to stop
boolean newData = false;
int desiredPos;
int currentPos = 0;

int encoderPos = 0; // position of encoder
int encoderPosLow = 0;
int encoderPrev; //previous position of encoder


// Define Control Pins, all are arbitrary for right now
const int stepPin = 5;
const int dirPin = 4;
const int enPin = 8;
// Define Encoder Pins, use both interrupt pins
const int outputA = 2;
const int outputB = 3;
const int outputZ = 6;
// Define Emergency Stop Pins
const int emergencyPin = 10;


void setup() {
  // Configure Control Pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW); //Have it be motor controlled currently
  // Configure Encoder Pins
  pinMode (outputA, INPUT_PULLUP);
  pinMode (outputB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), updateEncoder, CHANGE);
  encoderPos = 0;
  // Setup Communication
  Serial.begin(9600);
  while (!Serial) {
    //Wait until serial is up and running;
  }
}

void loop() {
  // check for input data
  while (newData == false) {
    //Wait until data is recieved from web page
    desiredPos = recvData();
  }
  if(desiredPos == 17){
    encoderPos = 0;
    encoderPosLow = 0;
    desiredPos = 0;
    currentPos = 0;
  }
  
  Serial.print("Recieved: ");
  Serial.println(desiredPos);
  newData = false; // reset serial perform operation based upon input
  // perform operations based on desired Position, use getDir()
  int num = getDir();
  moveCage(num);
  currentPos = desiredPos;
  Serial.print("Current Position:");
  Serial.println(currentPos);

  //dispEncoder();
  Serial.print("encoderPos: ");
  //Serial.println(encoderPos);
  Serial.println(encoderPosLow);
  Serial.println();
}

//------------------------------------------------------------------------------------
// List of Functions

char recvData() {
  if (Serial.available() > 0) {
    byte datain = Serial.read(); //Reads new data from serial
    newData = true;
    return int(datain) - 65; // conversion form ASCI to int
  }
}

int getDir() {
  // code for determining the shortest route to the next cage
  // flips dirPin HIGH or LOW
  // used in combination with moveCage

  // Need to:
  // Map A -- J to 0 -- 9
  // Case 1:
  // if desiredPos+10 - currentPos > desiredPos - currentPos  ==> clockwise
  // if (desiredPos + 10 - currentPos)>0 && (desiredPos - currentPos)<0 ==> counter-clockwise
  // also check sign, one will be if both positive, vs opposite sign
  // also check for magnitide.
  int diff = desiredPos - currentPos;
  if (diff > 5) {
    digitalWrite(dirPin, LOW);
    return 10 - diff;
  }
  else if (diff <= -5) {
    digitalWrite(dirPin, HIGH);
    return diff + 10;
  }
  else if (diff < 0 && diff > -5) {
    digitalWrite(dirPin, LOW);
    return 0 - diff;
  }
  else {
    digitalWrite(dirPin, HIGH);
    return diff;
  }
}

void moveCage(int numCages) {
  //Code for moving the equivalent of one cage
  if (numCages > 0) {
    accel();
    unsigned long var = 3 * numSteps * numCages / 10 - 5600; //=numSteps*numCages*3 - 40...3* gear ratio - 40 steps form acceleration and deceleration
    for (unsigned long j = 0; j < var; j++) { //40 steps form acceleration and deceleration
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepInterval);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepInterval);
    }
    deccel();
  }
  //return 0;
}

void accel() {
  while (varRate > 100) {
    for (int i = 0; i < 20; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(varRate);
    }
    varRate -= 5;
  }
}

void deccel() {
  while (varRate < 800) {
    for (int i = 0; i < 20; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(varRate);
    }
    varRate += 5; // delay by 33326/2 microSeconds less each time (maximum to meet acceleration standards);
  }
}

void updateEncoder() {
  
  //Serial.print(encoderPos);
  int a = digitalRead(outputA);
  int b = digitalRead(outputB);

  if(a == 0){
    encoderPosLow -= 2*b-1;
  }
  
  /*
  if (digitalRead(outputZ) == 1) {
    encoderPos = 0;
  }
  */
}

void dispEncoder() {
  //Displays encoder information to the serial. This should only be used while testing.
  if (encoderPrev != encoderPos) {
    int zState = digitalRead(outputZ);
    Serial.print("outputZ: ");
    Serial.print(zState);
    Serial.print("     Position: ");
    Serial.println(encoderPos);
    encoderPrev = encoderPos;
  }
}


//void checkEmergencyStop(){
//  //for unintended stopping, set enable pin to HIGH
//}
//
//void positionEncoder(){
//  //gets the position of the encoder, will always be running
//  //used to get errors if large discrepancy, and track when enable pin is HIGH
//}
//
//void reset(){
//  // resets all non-position variables to the original values
//  // need to check for name collisions before calling this "reset"
//}



