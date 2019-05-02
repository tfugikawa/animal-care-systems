//Written in Arduino 1.8.8

// Global Vars
unsigned long numSteps = 20000; // Microstepping, steps per revolution of the motor
unsigned int stepInterval = 175; // Used for delayMicroseconds, controls max speed (30 degrees per second)
unsigned int varRate = 800; // Used for delayMicroseconds, controls current speed
unsigned int initialVarRate = varRate;

const float gR = 3;//Gear ratio, carousel gear/motor gear
const int encoderPulsesRev = 200;//Encoder pulses per revolution
const int encPR = 2*encoderPulsesRev;//Rising and falling edges per revolution (encoder "ticks")
const int cN = 10;//Number of cages
const float tC = (1.0*encPR)/(1.0*cN);//Number of encoder ticks per cage

boolean eStop = false; // Logic for emergency stoppage
boolean newData = false; // Logic for whether or not there is data to be processed in the serial
int desiredPos = 0; //Desired cage position after moving (or other desired value for relative motion commands)

int encoderPos = 0; //current position of encoder
int encoderPrev = 0; //previous position of encoder, CURRENTLY NOT IN USE
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

const int accTotalSteps = (int)((1.0*(accRateMax-accRateMin))/accRateDelta)*accRateSteps; //number of steps during accel()
const int decTotalSteps = (int)((1.0*(decRateMax-decRateMin))/decRateDelta)*decRateSteps; //number of steps during deccel()
const int accDecSteps = accTotalSteps+decTotalSteps;


const int emergencyStopEncoderThreshold = 5; //if the encoder is farther than this many encoder pulses from theoretical, there is a problem
const int overshootCageFrac = 0.2; //number of cages to stop before desired position (intentional undershoot to stop overshoot)
const int overshootCorrection = floor(overshootCageFrac*gR*numSteps/(1.0*cN)); //number of steps to stop before desired position (intentional undershoot to stop overshoot)
const float nudgeFrac = 0.07; //fraction of a cage to move when nudging



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
      altZero = ((int)(encoderPos-prevPos*tC+encPR))%encPR;
      Serial.print("\nSet ");
      Serial.print((char)(prevPos+65));
      Serial.print(" in alternate view to: ");
      Serial.print(encoderPos);
      Serial.print(" and set alternate view 0 to: ");
      Serial.println(altZero);
    }else{ // previously issued command was a main view
      mainZero = ((int)(encoderPos-prevPos*tC+encPR))%encPR;
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
  
    if(num>=0 && num<=cN-1){ //moving to a cage position
      //full cage rotations and prev/next encoder positions are already taken care of
      //nudges don't care about encoder position
      prevPos = desiredPos; // this value is used during calibration
      desEncoderPos = ((int)(desiredPos*tC + encoderZero))%encPR;
      Serial.print("Desired Encoder Pos");
      Serial.println(desEncoderPos);
    }
    
    moveCage(num);
    
    Serial.println("\n\nStopped moving the motor\n");
    desEncoderPos = encoderPos;
  }
}

//------------------------------------------------------------------------------------
// List of Functions

char recvData() {
  /*
   * returns the recieved character-65 (so that A is 0, B is 1, etc.)
   */
  if (Serial.available() > 0) {
    byte datain = Serial.read(); //Reads new data from serial
    newData = true;
    return int(datain) - 65; // conversion from ASCII to int
  }
}

int getDir() {
  /*
   * returns the number of cages to move OR -88 for prev/next OR -77 for full rotations OR -99 for nudges
   * This output is fed to moveCage
   * 
   * This function determines the desired carousel position, desired encoder position and possibly direction
   */
  // code for determining the desired encoder position and direction from current position
  // flips dirPin HIGH or LOW
  // used in combination with moveCage

  //run if input was '[' or ']' for one full IJABC or one full CBAJI rotation or similar
  // '<' for nudge ABC and '>' for nudge CBA
  // '+' for one cage ABC and '-' for one cage CBA
  
  if(desiredPos == -22){ // +, one right
    digitalWrite(dirPin, HIGH);
    
    int currentPos = floor(((encoderPos-encoderZero+tC/2+encPR)%encPR)/tC);
    desiredPos = ((int)(currentPos+1 + tC/2))%cN;
    
    desEncoderPos = ((int)(encoderPos+tC))%encPR;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    
    return -88;
  }else if(desiredPos == -20){ // -, one left
    digitalWrite(dirPin, LOW);
    
    int currentPos = floor(((encoderPos-encoderZero+tC/2+encPR)%encPR)/tC);
    desiredPos = (currentPos-1 + tC/2)%cN;
    
    desEncoderPos = ((int)(encoderPos-tC+encPR))%encPR;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    
    return -88;
  }else if(desiredPos == 26){ // [, full rotation CW
    digitalWrite(dirPin, HIGH);
    
    desiredPos = floor(((encoderPos-encoderZero+tC/2+encPR)%encPR)/tC);
    
    desEncoderPos = encoderPos;
    Serial.print("\nDesired Encoder Pos");
    Serial.println(desEncoderPos);
    return -77;
  }else if(desiredPos == 28){ // ], full rotation CCW
    digitalWrite(dirPin, LOW);
    
    desiredPos = floor(((encoderPos-encoderZero+tC/2+encPR)%encPR)/tC);
    
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


  // Unless it's a special case handled above, the carousel should only move for inputs A -- J and a -- j or similar
  if((desiredPos>=0)&&(desiredPos<=cN-1)){// maps A -- J to 0 -- 9 or similar
    encoderZero = mainZero;
    altView = false;
    
    return desiredPos;
  }else if((desiredPos>=32)&&(desiredPos<=cN+32-1)){// maps a -- j to 0 -- 9 or similar
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
  /*
   * numCages: number of cages to move OR flag for other types of movement
   * Interprets the input value and moves the carousel accordingly
   */
  
  if (numCages >= 0 || numCages == -88) { //move to cage or rotate by one cage
    
    int encDistance = min(abs(encoderPos-desEncoderPos),encPR-abs(encoderPos-desEncoderPos)); //takes values 0 -- encPR/2
    
    
    int tempT = (int)(encPR/2.0);
    if(encDistance <= tempT){ //find which direction would be shorter and sets dirPin
      if(inRange((desEncoderPos-tempT/2+encPR)%encPR,encoderPos,tempT/2)){
        digitalWrite(dirPin, HIGH);
      }else{
        digitalWrite(dirPin, LOW);
      }
    }
    int direc = 2*digitalRead(dirPin)-1; // 1 if high, -1 if low
    
    long steps = (int)(encDistance * gR * numSteps/(1.0*encPR)); //map 0--encPR/2 to 0-gR*numSteps/2

    if(steps<=overshootCorrection){
      Serial.println("\nToo short for overshoot");
      Serial.println(steps);
      moveArb(steps);
    }else{
      boolean shortMove = moveArb(steps-overshootCorrection);
      
      //correct for over/undershoot
      if(!eStop){
        Serial.println("\nUndershoot correction");
        int theoEnc = encoderPos;
        long sCount = 0;
        
        if(!shortMove){
          while(!inRange(encoderPos,desEncoderPos,0)) { //while not within _ encoder steps of desired
            oneStep(decRateMax);
            
            sCount++;
            theoEnc = errorCheck(sCount, theoEnc, direc);
            
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
            oneStep(varRate);
            
            sCount++;
            theoEnc = errorCheck(sCount, theoEnc, direc);
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
    long steps = gR * numSteps;
    
    moveArb(steps-overshootCorrection);

    int theoEnc = encoderPos;
    long sCount = 0;
    
    //correct for over/undershoot
    Serial.println("\nUndershoot correction");
    while(!inRange(encoderPos,desEncoderPos,1)) { //while not within _ encoder steps of desired
      oneStep(decRateMax);
      
      sCount++;
      theoEnc = errorCheck(sCount, theoEnc, direc);
      if(eStop){
        break;
      }
    }
    if(eStop){
      Serial.print("\nERROR. Source: moveCage (full revolution undershoot correction), should have reached ");
      Serial.print(theoEnc);
    }
  }else if(numCages == -99){ //nudge left or right depending on current dirPin value
    int steps = floor(gR * numSteps/(1.0*cN) * nudgeFrac); //convert cage fraction to steps
    moveArb(steps);
  }

  if(eStop){
    emergencyStop();
  }

}

boolean moveArb(long steps) {
  /*
   * steps: number of motor steps to move the carousel
   * Steps the motor "steps" times. Handles acceleration, constant velocity, and deceleration movement.
   * returns true if very short movement occured, false otherwise
   * 
   * As of this version, calculations assume that accRateDelta == decRateDelta == 1
   * Future versions will need reworking to account for this
   */
  if(eStop){
    Serial.println("\nERROR. Source: moveArb, previous error");
    return false;
  }

  
  boolean shortMove = false;
  int direc = 2*digitalRead(dirPin)-1; // 1 if high, -1 if low
  
  
  if(steps >= accDecSteps){ //"Normal" operation with acceleration, full speed movement, and deceleration
    Serial.println("\naccel");
    accel();
    Serial.println("\nmovement");
    if(eStop){
      Serial.println("\nERROR. Source: moveArb, error in accel()");
      return shortMove;
    }
    
    int theoEnc = encoderPos;
    Serial.print("\nStarts at: ");
    Serial.println(theoEnc);
    
    for (unsigned long j = 0; j < (steps-accDecSteps); j++) {
      oneStep(stepInterval);

      //Error checking
      theoEnc = errorCheck(j, theoEnc, direc);

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
    //Movement is too short to reach decMaxRate at any point, so deceleration never happens
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
        oneStep(varRate);
        
        sCount++;
        if(sCount==steps){
          break;
        }
        theoEnc = errorCheck(sCount, theoEnc, direc);
        
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
        oneStep(varRate);
        
        sCount++;
        theoEnc = errorCheck(sCount, theoEnc, direc);
        
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
        oneStep(varRate);
        sCount++;
        theoEnc = errorCheck(sCount, theoEnc, direc);

        
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
  /*
   * Brings the carousel from stopped to full speed
   */
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
      oneStep(varRate);
      
      sCount++;
      theoEnc = errorCheck(sCount, theoEnc, direc);

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
  /*
   * Brings the carousel from full speed movement to slower movement
   */
  
  if(eStop){
    Serial.println("\nERROR. Source: deccel, previous error ");
    return;
  }
  
  varRate = decRateMin;
  while (varRate < decRateMax) {
    for (int i = 0; i < decRateSteps; i++) {
      oneStep(varRate);
      
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
  /*
   * cur: current encoder position
   * des: desired encoder position
   * t: how far off cur can be from des (modulo 400)
   * returns true if cur is in range of des, false otherwise
   */
  return (abs((cur-des+t+encPR)%encPR-t)<=t);
}

void updateEncoder() {
  /*
   * Updates the encoder position based on a 200 pulse/revolution quadrature encoder
   * The team chose to use rising and falling edges of a channel, rather than just rising
   * This was to remove ambiguity of position and increase precision
   * The Z channel only pulses once per revolution
   */
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
  encoderPos = (encoderPos+encPR)%encPR;

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


  //This block of commented out code MAY be necessary to include for proper functionality
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
  /*
   * Handles the emergency stop functionality. If the carousel is not moving correctly, or if the button is hit, eStop is set to true, and this usually occurs after that
   */
   
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
  /*
   * Runs on startupt before any other command can be input.
   * Waits for confirmation to begin, then rotates the carousel until it reaches the zero reset
   * It then depowers the motor and waits for the user to move it to the A (main view) position
   */
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
    
    delay(10);
    
    bool fin = false; //flag for if encoder has reached its zero reset
    digitalWrite(dirPin,HIGH);
    digitalWrite(enPin,LOW);
    
    varRate = accRateMax;
    while (varRate > accRateMin) {
      for (int i = 0; i < accRateSteps; i++) {
        oneStep(varRate);
        
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
  
    while (!fin) {
      oneStep(stepInterval);
      
      if(digitalRead(outputZ)==0){
        fin = true;
      }
      if(eStop){
        break;
      }
    }
  
    
    varRate = decRateMin;
    while (varRate < decRateMax && (digitalRead(enPin)==0) ) {
      for (int i = 0; i < decRateSteps; i++) {
        oneStep(varRate);
        
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
      altZero = ((int)(encoderZero+tC/2.0+encPR))%encPR; //default alternate view is half cage rotation
      Serial.print("\nCurrently at A (main view)\nZero is at ");
      Serial.println(encoderZero);
      varRate = initialVarRate;
      delay(100);
      return;
    }else{
      emergencyStop();
      eStop = false;
    }
  }
}


int errorCheck(long sCount, int theoEnc, int direc){
  /* 
   * sCount: current step count
   * theoEnc: what the encoder should read (with error bars) under correct operation
   * direc: 1 or -1, depending on which direction the motor should be rotating
   * 
   * Takes in the previous theoretical encoder position, the step count, and direction, and updates the theoretical encoder position if necessary
   * If the encoder position is not in bounds, it sets the error flag to true
   */
  float revFrac = 1.0/80.0;//Fraction of a revolution traveled between each check
  int theoEncNew = theoEnc;
  if(sCount%((int)(numSteps*gR*revFrac))==0 && sCount!=0){ // theoretical 1/80th of a revolution = 5 encoder pulses for 400 p/r encoder
    theoEncNew = ((int)(theoEnc+direc*encPR*revFrac+encPR))%encPR;
    if(!inRange(encoderPos,theoEncNew,emergencyStopEncoderThreshold)){
      eStop = true;
      Serial.println("\nERROR");
    }
  }

  return theoEncNew;
}

void oneStep(int r){
  /*
   * r: delay (in microseconds) between toggling the pin
   * Makes the motor step once.
   */
  digitalWrite(stepPin, HIGH);
  delayMicroseconds(r);
  digitalWrite(stepPin, LOW);
  delayMicroseconds(r);
}

void buttonPush(){
  /*
   * CURRENTLY DOES NOTHING DUE TO HARDWARE CONCERNS MAKING THE STOP BUTTON UNVIABLE
   * Sets the emergency stop flag to true
   */
  
  //eStop = true;
}
