//Written in Arduino 1.8.8

// Global Vars
unsigned long numSteps = 20000; // Microstepping, steps per revolution of the motor
unsigned int stepInterval = 175; // Used for delayMicroseconds, controls max speed (30 degrees per second)
unsigned int varRate = 800;//62500;// Acceleration function used for rampup and rampdown (125000/2) This is a 125 ms frequency starting pulse (only moves 10 steps)

boolean eStop = false; // Logic for emergency stoppage
boolean newData = false; // Logic for whether or not there is data to be processed in the serial
int desiredPos = 0; //Desired cage position after moving (or other desired value for relative motion commands)

int encoderPos = 0; //current position of encoder
int encoderPrev = 0; //previous position of encoder
int encoderZero = 0; //zeroed position of the encoder
int desEncoderPos = encoderPos; //desired position of encoder after moving

int mainZero = 0; //zeroed position of main view
int altZero = 0; //zeroed position of alternate view
int prevPos = 0; //previously issued command
bool altView = false; //false if main view, true if alternate view

// Define Control Pins
const int stepPin = 5;
const int dirPin = 4;
const int enPin = 8;

// Define Encoder Pins, only one (A for current implementation) required to be an interrupt
const int outputA = 2;
const int outputB = 7;
const int outputZ = 6;

// Define Emergency Stop Pins
const int buttonPin = 3;
const int relayPin = 10;


// Movement variables (to be changed by testing)
const int accRateMax = 850; //time delay, so larger numbers are slower movement
const int accRateMin = stepInterval; //matches the final movement speed of the carousel
const int accRateSteps = 7; //steps at each time delay
const int accRateDelta = 1; //change in time delay
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
  digitalWrite(enPin, HIGH); //HIGH means the motor is disengaged
  
  // Configure Encoder Pins
  pinMode(outputA, INPUT_PULLUP);
  pinMode(outputB, INPUT_PULLUP);
  pinMode(outputZ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(outputA), updateEncoder, CHANGE);

  // Configure Emergency Stop Pins
  pinMode(buttonPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(buttonPin), buttonPush, FALLING);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);
  
  // Setup Communication
  Serial.begin(9600);
  while (!Serial) {
    //Wait until serial is up and running;
  }

  eStop = false;
  startupRoutine(); // moves carousel until it reaches the encoder zero

  
}

void loop() {
  delay(1000);
  digitalWrite(enPin, HIGH);
  Serial.println("\nWaiting for new data");
  eStop = false;
  while (newData == false) {
    //Wait until data is recieved from web page
    desiredPos = recvData();
    if(eStop){
      Serial.println("\nERROR. Source: loop while getting data");
      desiredPos = 0;
      newData = true;
    }
  }
  
  newData = false; // reset serial perform operation based upon input
  digitalWrite(enPin, LOW); //prepare to move


  if(desiredPos == -23){ //this block is run if the input was '*' for calibration

    if(altView){ // previously issued command was an alternate view
      altZero = (encoderPos-prevPos*40+400)%400;
      Serial.print("\nSet ");
      Serial.print((char)(prevPos+65));
      Serial.print(" in alternate view to: ");
      Serial.print(encoderPos);
      Serial.print(" and set alternate view 0 to: ");
      Serial.println(altZero);
    }else{ // previously issued command was a main view
      mainZero = (encoderPos-prevPos*40+400)%400;
      Serial.print("\nSet ");
      Serial.print((char)(prevPos+65));
      Serial.print(" in main view to: ");
      Serial.print(encoderPos);
      Serial.print(" and set main view 0 to: ");
      Serial.println(mainZero);
    }

  }else{
    Serial.print("\nRecieved: ");
    Serial.println(desiredPos);
    
    // perform operations based on desired position, use getDir()
    int num = getDir();

    Serial.print("Interpreted as: ");
    Serial.println(desiredPos);
  
    if(num>=0 && num<=9){ //moving to a cage position
      //full cage rotations and prev/next encoder positions are already taken care of
      //nudges don't care about encoder position
      prevPos = desiredPos;
      desEncoderPos = (desiredPos*40 + encoderZero)%400;
      Serial.print("Desired Encoder Pos");
      Serial.println(desEncoderPos);
    }
    
    moveCage(num);
  
    Serial.println("\n\nStopped moving the motor\n");
    //+20 is to change the zone from position + [0,40] to position + [-20,20]
    /*
    currentPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    Serial.print("\nCurrent Position (after moving):");
    Serial.println(currentPos);
    */
    desEncoderPos = encoderPos;
  }
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
  // code for determining the desired encoder position and direction from current position
  // flips dirPin HIGH or LOW
  // used in combination with moveCage

  //run if input was '[' or ']' for one full IJABC or one full CBAJI rotation
  // '<' for nudge ABC and '>' for nudge CBA (should be changed)
  // '+' for one cage ABC and '-' for one cage CBA
  //whether the direction pins should be high or low requires testing
  
  if(desiredPos == -22){ // +, one right
    digitalWrite(dirPin, HIGH);
    
    int currentPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    desiredPos = (currentPos+1 + 20)%10;
    
    desEncoderPos = (encoderPos+40)%400;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    
    return -88;
  }else if(desiredPos == -20){ // -, one left
    digitalWrite(dirPin, LOW);
    
    int currentPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    desiredPos = (currentPos-1 + 20)%10;
    
    desEncoderPos = (encoderPos-40+400)%400;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    
    return -88;
  }else if(desiredPos == 26){ // [, full rotation CW
    digitalWrite(dirPin, HIGH);
    
    desiredPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    
    desEncoderPos = encoderPos;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    return -77;
  }else if(desiredPos == 28){ // ], full rotation CCW
    digitalWrite(dirPin, LOW);
    
    desiredPos = floor(((encoderPos-encoderZero+20+400)%400)/40.0);
    
    desEncoderPos = encoderPos;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    return -77;
  }else if(desiredPos == -5){ // <, nudge left
    digitalWrite(dirPin, HIGH);
    return -99;
  }else if(desiredPos == -3){ // >, nudge right
    digitalWrite(dirPin, LOW);
    return -99;
  }


  // Unless it's a special case handled above, the carousel should only move for inputs A -- J and a -- j
  if((desiredPos>=0)&&(desiredPos<=9)){
    encoderZero = mainZero;
    altView = false;
    
    return desiredPos;
  }else if((desiredPos>=32)&&(desiredPos<=41)){
    encoderZero = altZero;
    altView = true;
    
    desiredPos = desiredPos - 32;
    return desiredPos;
  }


  // Otherwise, nothing should happen
  Serial.println("Desired Position is not valid");
  return -2;
}


void moveCage(int numCages) {
  
  if (numCages >= 0 || numCages == -88) { //move to cage or rotate by one cage
    
    int encDistance = min(abs(encoderPos-desEncoderPos),400-abs(encoderPos-desEncoderPos)); //values 0-200

    //int tempT = 20;
    int tempT = 200;
    if(encDistance <= tempT){ //direction not set in getDir() since it so short, need to set it here
      if(inRange((desEncoderPos-tempT/2+400)%400,encoderPos,tempT/2)){
        digitalWrite(dirPin, HIGH);
      }else{
        digitalWrite(dirPin, LOW);
      }
    }
    int direc = 2*digitalRead(dirPin)-1; // 1 if high, -1 if low
    
    int overshoot_correction = floor(0.2*3*numSteps/10); //chosen to stop 0.2 cages before desired
    long steps = encDistance * 3 * numSteps/400; //map 0-200 to 0-30000

    if(steps<=overshoot_correction){
      Serial.println("\nToo short for overshoot");
      Serial.println(steps);
      moveArb(steps);
    }else{
      boolean shortMove = moveArb(steps-overshoot_correction);
      
      //correct for over/undershoot
      //inRange(current,desired,threshold range)
      if(!eStop){
        Serial.println("\nUndershoot correction");
        int theoEnc = encoderPos;
        long sCount = 0;
        
        if(!shortMove){
          while(!inRange(encoderPos,desEncoderPos,0)) { //while not within _ encoder steps of desired
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(decRateMax); // Used for delayMicroseconds, controls max speed
            digitalWrite(stepPin, LOW);
            delayMicroseconds(decRateMax);
            sCount++;

            if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
              theoEnc = (theoEnc+direc*5+400)%400;
              if(!inRange(encoderPos,theoEnc,3)){
                eStop = true;    
              }
            }
            
            if(eStop){
              break;
            }
          }
          if(eStop){
            Serial.print("\nERROR. Source: moveCage (undershoot correction), should have reached ");
            Serial.println(theoEnc);
          }
        }else{
          Serial.println("Very short movement");
          while(!inRange(encoderPos,desEncoderPos,0)) { //while not within _ encoder steps of desired
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(varRate); // Used for delayMicroseconds, controls max speed
            digitalWrite(stepPin, LOW);
            delayMicroseconds(varRate);
            sCount++;
            
            if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
              theoEnc = (theoEnc+direc*5+400)%400;
              if(!inRange(encoderPos,theoEnc,3)){
                eStop = true;    
              }
            }
            if(eStop){
              break;
            }
          }
          if(eStop){
            Serial.print("\nERROR. Source: moveCage (short undershoot correction), should have reached ");
            Serial.println(theoEnc);
          }
        }
      }
    }
      
  }else if(numCages == -77){ //full revolution
    int direc = 2*digitalRead(dirPin)-1; // 1 if high, -1 if low
    int overshoot_correction = floor(0.2*3*numSteps/10); //chosen to stop 0.2 cages before desired
    long steps = 3 * numSteps;
    
    moveArb(steps-overshoot_correction);

    int theoEnc = encoderPos;
    long sCount = 0;
    
    //correct for over/undershoot
    Serial.println("\nUndershoot correction");
    while(!inRange(encoderPos,desEncoderPos,1)) { //while not within _ encoder steps of desired
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(decRateMax); // Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(decRateMax);
      sCount++;
            
      if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
        theoEnc = (theoEnc+direc*5+400)%400;
        if(!inRange(encoderPos,theoEnc,3)){
          eStop = true;    
        }
      }
      if(eStop){
        break;
      }
    }
    if(eStop){
      Serial.print("\nERROR. Source: moveCage (full revolution undershoot correction), should have reached ");
      Serial.print(theoEnc);
    }
  }else if(numCages == -99){ //nudge left or right depending on current stepPin value
    float frac = 0.07; //ordered to move 0.07 cages
    int steps = floor(3 * numSteps/10 * frac); //convert cage fraction to steps
    moveArb(steps);
  }

  if(eStop){
    emergencyStop();
  }

}

boolean moveArb(long steps) {
  if(eStop){
    Serial.println("\nERROR. Source: moveArb, previous error");
    return false;
  }
  
  //returns true if very short movement occured
  boolean shortMove = false;
  int direc = 2*digitalRead(dirPin)-1; // 1 if high, -1 if low

  
  //As of this version, calculations assume that accRateDelta == decRateDelta == 1
  //Future versions will need reworking to account for this
  
  
  if(steps >= accDecSteps){ //"Normal" operation
    Serial.println("\naccel");
    accel();
    Serial.println("\nmovement");
    if(eStop){
      Serial.println("\nERROR. Source: moveArb, error in accel()");
      return shortMove;
    }
    
    int theoEnc = encoderPos;
    /*
    Serial.print("\nStarts at: ");
    Serial.println(theoEnc);
    */
    
    for (unsigned long j = 0; j < (steps-accDecSteps); j++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepInterval);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepInterval);


      //Error checking
      if(j%(numSteps*3/80)==0 && j!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
        theoEnc = (theoEnc+direc*5+400)%400;
        if(!inRange(encoderPos,theoEnc,3)){
          eStop = true;    
        }
      }
      if(eStop){
        break;
      }
      
    }

    if(!eStop){
      Serial.println("\nLong movement");
      Serial.println("deccel");
      deccel();
    }else{
      Serial.print("\nERROR. Source: moveArb, should have reached ");
      Serial.println(theoEnc);
    }
    
  }else if(steps < accRateSteps*(accRateMax-decRateMax)){ //Special case for very short movement
    //Movement is too short to reach decMaxRate at any point, so decceleration never happens
    //Current implementation is to ignore this and just accelerate to the end
    //The highest velocity reached is still slower than that at the end of deccel()
    shortMove = true;
    varRate = accRateMax;
    
    long sCount = 0;
    int theoEnc = encoderPos;
    
    while (sCount < steps) {
      if(eStop){
        break;
      }
      for (int i = 0; i < accRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
        sCount++;
        if(sCount==steps){
          break;
        }

        if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
          theoEnc = (theoEnc+direc*5+400)%400;
          if(!inRange(encoderPos,theoEnc,3)){
            eStop = true;
          }
        }
        if(eStop){
          break;
        }
      }
      
      varRate -= accRateDelta;
      
    }

    if(eStop){
      Serial.print("\nERROR. Source: moveArb, should have reached ");
      Serial.println(theoEnc);
    }
  
  }else{ //movement less than accel() + deccel(), but without the problems above

    int theoEnc = encoderPos;
    long sCount = 0;
    
    //ceil(value) so that the end position is never passed
    int desRate = ceil((1.0*(accRateSteps*accRateMax + decRateSteps*decRateMax - steps)) / (accRateSteps + decRateSteps));
    
    varRate = accRateMax;
    while (varRate > desRate) {
      if(eStop){
        break;
      }
      
      for (int i = 0; i < accRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
        sCount++;
        
        if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
          theoEnc = (theoEnc+direc*5+400)%400;
          if(!inRange(encoderPos,theoEnc,3)){
            eStop = true;
            Serial.print("\nERROR: ");
            Serial.println(theoEnc);
          }
        }
        if(eStop){
          break;
        }
      }
      varRate -= accRateDelta;
      
    }


    theoEnc = encoderPos;
    sCount = 0;
    
    while (varRate < decRateMax) {
      if(eStop){
        break;
      }
      
      for (int i = 0; i < decRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
        sCount++;
        
        if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
          theoEnc = (theoEnc+direc*5+400)%400;
          if(!inRange(encoderPos,theoEnc,3)){
            eStop = true;
          }
        }
        if(eStop){
          break;
        }
      }
      
        
      varRate += decRateDelta;
    }
    
    if(eStop){
      Serial.print("\nERROR. Source: moveArb, should have reached ");
      Serial.println(theoEnc);
    }
    
  }
  
  return shortMove;
}

void accel() {
  if(eStop){
    Serial.println("\nERROR. Source: accel, previous error ");
    return;
  }
  
  int theoEnc = encoderPos;
  int direc = 2*digitalRead(dirPin)-1; // 1 if high, -1 if low
  long sCount = 0;
  
  varRate = accRateMax;
  while (varRate > accRateMin) {
    for (int i = 0; i < accRateSteps; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(varRate);
      sCount++;
      
      if(sCount%(numSteps*3/80)==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses
        theoEnc = (theoEnc+direc*5+400)%400;
        if(!inRange(encoderPos,theoEnc,3)){
          eStop = true;
          Serial.println("\nERROR");
        }
      }
      if(eStop){
        break;
      }
    }
    varRate -= accRateDelta;
    if(eStop){
      break;
    }
  }
  if(eStop){
    Serial.print("\nERROR. Source: accel, should have reached ");
    Serial.println(theoEnc);
  }
}

void deccel() {
  if(eStop){
    Serial.println("\nERROR. Source: deccel, previous error ");
    return;
  }
  
  varRate = decRateMin;
  while (varRate < decRateMax) {
    for (int i = 0; i < decRateSteps; i++) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(varRate);
      if(eStop){
        break;
      }
    }
    varRate += decRateDelta;
    if(eStop){
      break;
    }
  }
  if(eStop){
    Serial.print("\nERROR. Source: deccel");
  }
}

boolean inRange(int cur, int des, int t){
  //cur is current encoder position
  //des is desired encoder position
  //t is how far off cur can be from des (modulo 400)
  //can be modified to different control scheme by changing both 400s
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


  /*
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
  */
}

void emergencyStop(){
  Serial.println("\nERROR. Make the carousel safe to operate, reset, then recalibrate");
  digitalWrite(relayPin,HIGH);
  digitalWrite(enPin,HIGH);

  int val = digitalRead(buttonPin);
  int temp = 1;
  while(val==0){
    if(temp%20==0){
      Serial.println("Button is still pushed");
    }
    temp++;
    delay(500);
    val = digitalRead(buttonPin);
  }
  
  while(eStop){
    int temp = 0;
    while(newData==false){
      temp = recvData();
    }
    newData = false;

    if(temp == 17){ // R
      Serial.println("\nSuccessful reset: please recalibrate the system");
      digitalWrite(relayPin,LOW);
      eStop = false;
      break;
    }else{
      Serial.println("\nInvalid input, please reset the system");
    }
  }
  eStop = false;
  prevPos = 0;
}

void startupRoutine(){
  if(eStop){
    Serial.println("Previous error");
  }
  while(true){  
    eStop = false;
    newData = false;
    digitalWrite(enPin,HIGH);
    int temp = 0;
    while(temp != -23){ //wait until '*' is recieved
      Serial.println("Press calibrate to begin");
      while (newData == false) {
        temp = recvData();
      }
      newData = false;
    }
    eStop = false;
    
    
    bool fin = false;
    digitalWrite(dirPin,HIGH);
    digitalWrite(enPin,LOW);

    Serial.println("Before accel");
    //Serial.println(eStop);

    varRate = accRateMax;
    while (varRate > accRateMin) {
      for (int i = 0; i < accRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
        if(digitalRead(outputZ)==0){
          fin = true;
        }
        if(eStop){
          break;
        }
      }
      varRate -= accRateDelta;
      if(fin){
        digitalWrite(enPin,HIGH);
      }
      if(eStop){
        break;
      }
    }

    Serial.println("After accel");
    //Serial.println(eStop);
  
    while (!fin) {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(stepInterval);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin, LOW);
      delayMicroseconds(stepInterval);
      
      if(digitalRead(outputZ)==0){
        fin = true;
      }
      if(eStop){
        break;
      }
    }

    Serial.println("After move");
    //Serial.println(eStop);
    
    varRate = decRateMin;
    while (varRate < decRateMax && (digitalRead(enPin)==0) ) {
      for (int i = 0; i < decRateSteps; i++) {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(varRate);// Used for delayMicroseconds, controls max speed
        digitalWrite(stepPin, LOW);
        delayMicroseconds(varRate);
        if(eStop){
          break;
        }
      }
      
      if(eStop){
        break;
      }
      varRate += decRateDelta;
    }
    digitalWrite(enPin,HIGH);

    Serial.println("After deccel");
    //Serial.println(eStop);
    
    if(!eStop){
      newData = false;
      temp = 0;
      while(temp != -23){ //wait until '*' is recieved
        Serial.println("\nPress calibrate when facing A (main view)");
        while (newData == false) {
          temp = recvData();
        }
        newData = false;
      }
      
      digitalWrite(enPin, LOW);  
      encoderZero = encoderPos;
      mainZero = encoderZero; //first view is main view
      altZero = (encoderZero+20+400)%400; //default alternate view is half cage rotation
      Serial.print("\nCurrently at A (main view)\nZero is at ");
      Serial.println(encoderZero);
      varRate = 800;
      delay(100);
      return;
    }else{
      Serial.println("error in initial calibration");
      emergencyStop();
      eStop = false;
    }
  }
}

void buttonPush(){
  eStop = true;
}
