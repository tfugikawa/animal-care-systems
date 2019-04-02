// Global Vars
unsigned long numSteps = 20000; // Microstepping
unsigned int stepInterval = 175; // Used for delayMicroseconds, controls max speed (30 degrees per second)
unsigned int varRate = 800;//62500;//Acceleration function used for rampup and rampdown (125000/2) This is a 125 ms frequency starting pulse (only moves 10 steps)
//const int accelRate = 2000; // Number of milliseconds for acceleration, used also for decceleration
boolean eStop = false; // Logic for determining whether or not to stop
boolean newData = false;
int desiredPos;
int currentPos = 0;

int encoderPos = 0; //position of encoder
int encoderPrev; //previous position of encoder
int encoderZero = 0; // zeroed position of the encoder
int desEncoderPos = encoderPos;
int direc = 1;


// Define Control Pins, all are arbitrary for right now
const int stepPin = 5;
const int dirPin = 4;
const int enPin = 8;
// Define Encoder Pins, only one (A for current implementation) required to be an interrupt
const int outputA = 2;
const int outputB = 3;
const int outputZ = 6;
// Define Emergency Stop Pins
const int emergencyPin = 10;


// Movement variables (to be changed by testing)
const int accRateMax = 850; //time delay, so larger numbers are slower. originally 800
const int accRateMin = stepInterval; //matches the final movement speed of the carousel
const int accRateSteps = 7; //steps at each time delay. originally 20
const int accRateDelta = 1; //change in time delay. originally 5
const int decRateMax = 300;
const int decRateMin = stepInterval;
const int decRateSteps = 3;
const int decRateDelta = accRateDelta;

const int accTotalSteps = (int)((1.0*(accRateMax-accRateMin))/accRateDelta)*accRateSteps;
const int decTotalSteps = (int)((1.0*(decRateMax-decRateMin))/decRateDelta)*decRateSteps;
const int accDecSteps = accTotalSteps+decTotalSteps;


void setup() {
  // Configure Control Pins
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW); //Have it be motor controlled currently
  // Configure Encoder Pins
  pinMode (outputA, INPUT_PULLUP);
  pinMode (outputB, INPUT_PULLUP);
  pinMode (outputZ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), updateEncoder, CHANGE);
  // Setup Communication
  Serial.begin(9600);
  while (!Serial) {
    //Wait until serial is up and running;
  }
  if(accDecSteps > 3*numSteps/10){
    Serial.println("WARNING: accel() and/or deccel() takes too long, adjust parameters.");
    Serial.print("It takes ");
    Serial.print((accDecSteps - 3*numSteps/10));
    Serial.println(" steps too many.");
    Serial.println("Decrease ___RateSteps, increase ___RateDelta, or decrease ___RateMax");//(___RateMin should not be changed so it matches the constant speed)
  }

  Serial.println("Move carousel to A");
  digitalWrite(enPin, HIGH);
  while (newData == false) {
    //Wait until data is recieved from web page
    int temp = recvData();
  }
  delay(500);
  Serial.flush();
  newData = false;

  digitalWrite(enPin, LOW);
  //DELETE THIS LATER
  //encoderPos = 0;
  //DELETE ABOVE LATER
  
  encoderZero = encoderPos;
  
  Serial.print("\nMoved to A\nZero is at ");
  Serial.println(encoderZero);
}

void loop() {

  delay(2000);
  digitalWrite(enPin, HIGH);
  Serial.println("\nWaiting for new data");
  while (newData == false) {
    //Wait until data is recieved from web page
    desiredPos = recvData();
  }

  newData = false;
  digitalWrite(enPin, LOW);
  
  
  if(desiredPos == -23){
    //this block is run if the input was '*' for calibration
    encoderZero = encoderPos;
    desiredPos = 0;
    currentPos = 0;
    Serial.println("Recalibrated to A");
  }else{
    //+20 is for edge cases
    currentPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    
    
    Serial.print("\nCurrent: ");
    Serial.print(currentPos);
    Serial.print("  ");
    Serial.print("Recieved: ");
    Serial.println(desiredPos);
    newData = false; // reset serial perform operation based upon input
    // perform operations based on desired position, use getDir()
    int num = getDir();

    Serial.print("Interpreted: ");
    Serial.println(desiredPos);
  
    if(num>=0 && num<=9){ //moving to a cage position or one full revolution
      //desEncoderPos = (encoderPos + direc*num*40+400)%400;
      desEncoderPos = (desiredPos*40 + encoderZero)%400;
      Serial.print("Desired Encoder Pos");
      Serial.println(desEncoderPos);
    }
    
    moveCage(num);
  
    Serial.println("\n\nStopped moving the motor\n");
    //+20 is to changes the zone from the original position + [0,40] to original position + [-20,20]
    currentPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    Serial.print("\nCurrent Position (after moving):");
    Serial.println(currentPos);
    desEncoderPos = encoderPos;
  }
  
  //dispEncoder(true);
  //dispEncoder(false);
}

//------------------------------------------------------------------------------------
// List of Functions

char recvData() {
  if (Serial.available() > 0) {
    byte datain = Serial.read(); //Reads new data from serial
    newData = true;
    return int(datain) - 65; // conversion from ASCII to int
  }
}

int getDir() {
  // code for determining the shortest route to the next cage
  // flips dirPin HIGH or LOW
  // used in combination with moveCage

  //run if input was '[' or ']' for one full IJABC or one full CBAJI rotation
  // '<' for nudge ABC and '>' for nudge CBA (subject to change)
  // '+' for one cage ABC and '-' for one cage CBA
  //this should only be kept for testing purposes
  //whether the direction pins should be high or low requires testing
  //it is assumed that the enable pin is correct
  
  if(desiredPos == -22){ // +, one right
    digitalWrite(dirPin, HIGH);
    direc = 1;
    desiredPos = (currentPos+1 + 20)%10;
    return 1;
  }else if(desiredPos == -20){ // -, one left
    digitalWrite(dirPin, LOW);
    direc = -1;
    desiredPos = (currentPos-1 + 20)%10;
    return 1;
  }else if(desiredPos == 26){ // [, full rotation CW
    digitalWrite(dirPin, HIGH);
    direc = 1;
    desiredPos = currentPos;
    desEncoderPos = encoderPos;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    return -1;
  }else if(desiredPos == 28){ // ], full rotation CCW
    digitalWrite(dirPin, LOW);
    direc = -1;
    desiredPos = currentPos;
    desEncoderPos = encoderPos;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    return -1;
  }else if(desiredPos == -5){ // <, nudge left
    digitalWrite(dirPin, HIGH);
    direc = 1;
    return -99;
  }else if(desiredPos == -3){ // >, nudge right
    digitalWrite(dirPin, LOW);
    direc = -1;
    return -99;
  }

  // Unless it's a special case handled above, the carousel should only move for inputs A -- J
  // Otherwise, nothing should happen
  if((desiredPos<0) || (desiredPos>9)){
    Serial.println("Desired Position is not valid");
    return -2;
  }
  
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
    direc = -1;
    return 10 - diff;
  }
  else if (diff <= -5) {
    digitalWrite(dirPin, HIGH);
    direc = 1;
    return diff + 10;
  }
  else if (diff < 0 && diff > -5) {
    digitalWrite(dirPin, LOW);
    direc = -1;
    return 0 - diff;
  }
  else {
    digitalWrite(dirPin, HIGH);
    direc = 1;
    return diff;
  }
}

void moveCage(int numCages) {
  
  if (numCages >= 0) {
    
    int encDistance = min(abs(encoderPos-desEncoderPos),400-abs(encoderPos-desEncoderPos)); //values 0-200

    if(encDistance <= 20){ //direction not set in getDir() since it so short, need to set it here
      if(inRange((desEncoderPos-10+400)%400,encoderPos,10)){
        digitalWrite(dirPin, HIGH);
        direc = 1;
      }else{
        digitalWrite(dirPin, LOW);
        direc = -1;
      }
    }
    
    int overshoot_correction = floor(0.3*3*numSteps/10); //chosen to stop 0.3 cages before expected
    long steps = encDistance * 3 * numSteps/400; //map 0-200 to 0-30000

    if(steps<=overshoot_correction){
      Serial.println("\nToo short for overshoot");
      Serial.println(steps);
      moveArb(steps);
    }else{
      boolean short = moveArb(steps-overshoot_correction);
      
      //correct for over/undershoot
      //inRange(current,desired,threshold range)
      Serial.println("\nUndershoot correction");
      if(!short){
        while(!inRange(encoderPos,desEncoderPos,2)) { //while not within 2 encoder steps of desired
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(decRateMax); // Used for delayMicroseconds, controls max speed
          digitalWrite(stepPin, LOW);
          delayMicroseconds(decRateMax);
        }
      }else{
        Serial.println("Very short movement");
        while(!inRange(encoderPos,desEncoderPos,1)) { //while not within 1 encoder step of desired
          digitalWrite(stepPin, HIGH);
          delayMicroseconds(varRate); // Used for delayMicroseconds, controls max speed
          digitalWrite(stepPin, LOW);
          delayMicroseconds(varRate);
        }
      }
    }
    
  }else if(numCages == -1){ //full revolution
    int overshoot_correction = floor(0.3*3*numSteps/10); //chosen to stop 0.3 cages before expected
    long steps = 3 * numSteps;
    
    moveArb(steps-overshoot_correction);
      
    //correct for over/undershoot
    Serial.println("\nUndershoot correction");
    while(!inRange(encoderPos,desEncoderPos,1)) { //while not within 2 encoder steps of desired
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(decRateMax); // Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(decRateMax);
    }
      
  }else if(numCages == -99){ //nudge left or right depending on current stepPin value
    float frac = 0.1; //ordered to move 0.1 cages
    int steps = floor(3 * numSteps/10 * frac); //convert cage fraction to steps
    moveArb(steps);
  }
  
}

boolean moveArb(long steps) {
  //returns true if very short movement occured
  boolean shortMove = false;
  //As of this version, calculations assume that accRateDelta == decRateDelta == 1
  //Future versions will need reworking to account for this

  
  if(steps >= accDecSteps){ //"Normal" operation
    Serial.println("\naccel");
    accel();
    Serial.println("\nmovement");
    for (unsigned long j = 0; j < (steps-accDecSteps); j++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepInterval);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepInterval);
    }
    Serial.println("\nLong movement");
    Serial.println("deccel");
    deccel();
  }else if(steps < accRateSteps*(accRateMax-decRateMax)){ //Special case for very short movement
    //Movement is too short to reach decMaxRate at any point, so decceleration never happens
    //Current implementation is to ignore this and just accelerate to the end
    //The highest velocity reached is still slower than that at the end of deccel()
    shortMove = true;
    varRate = accRateMax;
    long sCount = 0;
    while (sCount < steps) {
      for (int i = 0; i < accRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
        sCount++;
        if(sCount==steps){
          break;
        }
      }
      varRate -= accRateDelta;
    }
  }else{ //movement less than accel() + deccel(), but without the problems above
    
    //ceil(value) so that the end position is never passed
    int desRate = ceil((1.0*(accRateSteps*accRateMax + decRateSteps*decRateMax - steps)) / (accRateSteps + decRateSteps));
    
    varRate = accRateMax;
    while (varRate > desRate) {
      for (int i = 0; i < accRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
      }
      varRate -= accRateDelta;
    }
    
    while (varRate < decRateMax) {
      for (int i = 0; i < decRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
      }
      varRate += decRateDelta;
    }
  }
  return shortMove;
}

void accel() {
  varRate = accRateMax;
  while (varRate > accRateMin) {
    for (int i = 0; i < accRateSteps; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(varRate);
    }
    varRate -= accRateDelta;
  }
}

void deccel() {
  varRate = decRateMin;
  while (varRate < decRateMax) {
    for (int i = 0; i < decRateSteps; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(varRate);
    }
    varRate += decRateDelta; // delay by 33326/2 microSeconds less each time (maximum to meet acceleration standards);
  }
}

boolean inRange(int cur, int des, int t){
  //cur is current encoder position
  //des is desired encoder position
  //t is how far off cur can be from des (modulo 400)
  return (abs((cur-des+t+400)%400-t)<=t);
}

void updateEncoder() {
  encoderPrev = encoderPos;
  boolean A = (digitalRead(outputA)==1);
  boolean B = (digitalRead(outputB)==1);
  boolean Z = (digitalRead(outputZ)==1);

  //If A matches B, positive (clockwise) rotation
  if(A==B){
    encoderPos++;
  }else{
    encoderPos--;
  }
  encoderPos = (encoderPos+400)%400;

  //If Z is low and A is high, reached the 0 position
  if(A && !Z){
    //Serial.println("\n\nZ0");
    if(encoderPos!=0){
      //Serial.println("Resetting to 0");
      
      
      encoderPos = 0;
    }else{
      //Serial.println("No reset required");
    }
  }

  
  if(encoderPos%10==0)
    Serial.println();
  if(encoderPos>=0)
    Serial.print(" ");
  if(abs(encoderPos)<100)
    Serial.print(" ");
  if(abs(encoderPos)<10)
    Serial.print(" ");
  Serial.print(encoderPos);
  Serial.print("_A");
  Serial.print(digitalRead(outputA));
  Serial.print(" ");
  
}

void dispEncoder(boolean outZ) {
  //Displays encoder information to the serial. This only needs to be used while testing.
  Serial.println();
  if(outZ) {
    Serial.print("outputZ: ");
    Serial.println(digitalRead(outputZ));
  }
  Serial.print("encoderPos: ");
  Serial.println(encoderPos);
  Serial.println();
}


//void checkEmergencyStop(){
//  //for unintended stopping, set enable pin to HIGH
//  //this is for when something is holding the motor in place
//  //undecided if this is where checking for a stop button press should occur
//}
//void reset(){
//  // resets all non-position variables to the original values
//  // need to check for name collisions before calling this "reset"
//}
