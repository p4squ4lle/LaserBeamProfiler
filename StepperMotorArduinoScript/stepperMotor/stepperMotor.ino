  //Stepper Motor

//defines pin numbers
const int EndPin = 2;
const int stepPin = 3;
const int dirPin = 4;

String movement;
String steps;
int x;
String dir;
int d;
int SchalterZustand;
int pos;
int referenced;

void setup() {
  Serial.begin(9600);           // establish serial port
  
  pinMode(stepPin,OUTPUT);      // defines number of steps 
  pinMode(dirPin, OUTPUT);      // defines direction of movement
  pinMode(EndPin, INPUT);       // end switch condition
  referenced = 0;
}

void loop() { 
  
  if(Serial.available()>0) {        // if there is data to read
    // get type of movement
    movement=Serial.readStringUntil(';');
    if (movement.equals("R")){
      doRefRun();
      referenced = 1;
      pos = 0 ;
      Serial.println("Referenced");
    }
    else if (movement.equals("M") && referenced==1){
      // get number of steps
      steps = Serial.readStringUntil(';');
      x = steps.toInt();

      // get direction
      dir = Serial.readStringUntil(';');
      d = dir.toInt();      
      if( d == -1){
        digitalWrite(dirPin,LOW);
      }
      else if( d == 1){
        digitalWrite(dirPin,HIGH);
      }
      
      // movement inside for-loop     
      for(int y = 0; y < x; y++){
        SchalterZustand = digitalRead(EndPin);
        if  (SchalterZustand==1 && d==-1){
          Serial.println("LowEndSwitch");
          break;
        }
        else if (pos==29000 && d==1){
          Serial.println("highEnd");
          break;
        }
        moveStep();
        pos = pos + d;
        if (y%10==0 && Serial.availableForWrite()>0){
          Serial.println("s");
        }
      }
      Serial.println(pos);
    }
    else if (movement.equals("M") && referenced==0){
      Serial.println("NotYetRef");
    }
    else {
      Serial.println("BadCmd");
    }
  }
}

void moveStep() {
    digitalWrite(stepPin,HIGH);
    delayMicroseconds(950);
    digitalWrite(stepPin,LOW);
    delayMicroseconds(950); 
}

void doRefRun() {
  SchalterZustand = digitalRead(EndPin);

  if (SchalterZustand == 1){
    delay(50); 
  }
  else if (SchalterZustand == 0){
    digitalWrite(dirPin,LOW);

    do{
      moveStep();
      SchalterZustand=digitalRead(EndPin);
    } while (SchalterZustand == 0);
  }
}
