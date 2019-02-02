const int firstPin = 13;
const int secondPin = 12;
const int thirdPin = 11;
const int fourthPin = 10;
byte datain;
boolean newData = false;

void setup() {
  pinMode(firstPin,OUTPUT);
  pinMode(secondPin,OUTPUT);
  pinMode(thirdPin,OUTPUT);
  pinMode(fourthPin,OUTPUT);
  Serial.begin(9600);
  while(!Serial){
    //Wait until serial is up and running;
  }
}

void loop() {
  recvData(); //Check for data
  showNewData(); //Display new data
}

void recvData(){
    if(Serial.available()>0){
    datain = Serial.read(); //Reads new data from serial
    newData = true; //Assigns new data variable to true
    }
}

void showNewData(){
  if(newData == true){
    Serial.print("New Data: ");
    Serial.println(char(datain));
    newData = false;  //Resets newData variable
    check();  //Performs action from input
  }
}

void check(){
  switch(char(datain)){ 
    case '1': //When in position 1 perform action
    digitalWrite(firstPin,HIGH);
    //Serial.println("HIT");  //For debugging purposes
    delay(1000);
    digitalWrite(firstPin,LOW);
    break;
    case '2': //when in position 2 perform action
    digitalWrite(secondPin,HIGH);
    delay(1000);
    digitalWrite(secondPin,LOW);
    break;
    case '3': //when in position 3 perform action
    digitalWrite(thirdPin,HIGH);
    delay(1000);
    digitalWrite(thirdPin,LOW);
    break;
    case '4': //when in position 4 perform action
    digitalWrite(fourthPin,HIGH);
    delay(1000);
    digitalWrite(fourthPin,LOW);
    break;
    default:
    //Nothing
    break;
  }
}

