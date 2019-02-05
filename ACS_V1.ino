// Global Vars
unsigned long numSteps = 20000; // Microstepping
unsigned int stepInterval = 100; // Used for delayMicroseconds, controls max speed (30 degrees per second)
unsigned long varRate = 800;//62500;//Acceleration function used for rampup and rampdown (125000/2) This is a 125 ms frequency starting pulse (only moves 10 steps)
//const int accelRate = 2000; // Number of milliseconds for acceleration, used also for decceleration
boolean eStop = false; // Logic for determining whether or not to stop
boolean newData = false;
int desiredPos;
int currentPos = 1;


// Define Control Pins, all are arbitrary for right now
const int stepPin = 5; 
const int dirPin = 4; 
const int enPin = 8;
// Define Encoder Pins, use both interrupt pins
const int outputA = 2;
const int outputB = 3;
// Define Emergency Stop Pins
const int emergencyPin = 10;


void setup() {
  // Configure Control Pins
  pinMode(stepPin,OUTPUT); 
  pinMode(dirPin,OUTPUT);
  pinMode(enPin,OUTPUT);
  digitalWrite(enPin,LOW); //Have it be motor controlled currently
  // Configure Encoder Pins
  pinMode (outputA,INPUT_PULLUP);
  pinMode (outputB,INPUT_PULLUP);
  // Setup Communication
  Serial.begin(9600);
  while(!Serial){
    //Wait until serial is up and running;
  }
}

void loop() {
  // check for input data
  while (newData == false){
    //Wait until data is recieved from web page
    desiredPos = recvData();
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
}

//------------------------------------------------------------------------------------
// List of Functions

char recvData(){
   if(Serial.available()>0){
   byte datain = Serial.read(); //Reads new data from serial
   newData = true;
   return int(datain)-48; // conversion form ASCI to int
   }
}

int getDir(){
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
  if(diff > 5){
    digitalWrite(dirPin,LOW);
    return 10 - diff;
  }
  else if(diff <= -5){
    digitalWrite(dirPin,HIGH);
    return diff + 10;
  }
  else if(diff < 0 && diff > -5){
    digitalWrite(dirPin,LOW);
    return 0 - diff;
  }
  else{
    digitalWrite(dirPin,HIGH);
    return diff;
  }
}

void moveCage(int numCages){
  //Code for moving the equivalent of one cage
  //Use num steps
  //call accel and deccel
  //Serial.print(numCages + "\n");
  //accel();
  unsigned long var = numSteps*numCages/10;   //=numSteps*numCages*3 - 40...3* gear ratio - 40 steps form acceleration and deceleration
    for(unsigned long j=0;j<var;j++){ //40 steps form acceleration and deceleration
      digitalWrite(stepPin,HIGH);
      delayMicroseconds(stepInterval);// Used for delayMicroseconds, controls max speed
      digitalWrite(stepPin,LOW);
      delayMicroseconds(stepInterval);  
    }
  //deccel();
  return 0;
}

void accel(){
  //accelRate = 2000000; // Number of microseconds for acceleration, used also for decceleration
  unsigned long currentTime = micros();
  while(varRate>100){
    digitalWrite(stepPin,HIGH);
    while((micros()-currentTime)<varRate){
    // do not do anything during that period will need to eventually watch out for overflow error 
    }
    currentTime = micros();
    digitalWrite(stepPin,LOW);
    while((micros()-currentTime)<varRate){
      
    }
    Serial.print(varRate);
    varRate-= 124820; //6240; // delay by 12480/2 microSeconds less each time (maximum to meet acceleration standards) 
    currentTime = micros();
  }
}

void deccel(){
  //Inverse of acceleration function, used for decceleration only moves (10 steps)
  unsigned long currentTime = micros();
  while(varRate<62500){
    digitalWrite(stepPin,HIGH);
    while((micros()-currentTime)<varRate){
    // do not do anything during that period will need to eventually watch out for overflow error 
    }
    currentTime = micros();
    digitalWrite(stepPin,LOW);
    while((micros()-currentTime)<varRate){
      
    }
    varRate+= 12480; // delay by 33326/2 microSeconds less each time (maximum to meet acceleration standards);
    currentTime = micros();
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



