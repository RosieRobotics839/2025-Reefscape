
#include <Joystick.h>

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, JOYSTICK_TYPE_GAMEPAD, 13, 0, true, true, false, false, false, false, false, false, false, false, false);

#include <FastLED.h>
#define LED_PIN 13
#define NUM_LEDS 23

CRGB leds[NUM_LEDS];

  int button1Pin = 2;
  int button2Pin = 3;
  int button3Pin = 4;
  int button4Pin = 5;
  int button5Pin = 6;
  int button6Pin = 7;
  int button7Pin = 8;
  int button8Pin = 9;
  int button9Pin = 10;
  int button10Pin = 11;
  int button11Pin = 12;

  int elevatorPin = A0;
  int armPin = A1;

//For flash code later on:

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated

// constants won't change:
const long interval = 500;  // interval at which to blink (milliseconds)

bool flashLight = false;

//===============

// <gamepeice?, cameras?, gyro?, match_time?, drivetrain?, elevator?, gantry_arm?, intake?, funnel?, climber?>

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing

int gamepiece_value = 0;
int camera_value = 0;
int gyro_value = 0;
int matchtime_value = -1;
int drivetrain_value = 0;
int elevator_value = 0;
int gantryarm_value = 0;
int intake_value = 0;
int funnel_value = 0;
int climber_value = 0;

boolean newData = false;

//====================

void setup() {

  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50);

  pinMode(button1Pin, INPUT);
  pinMode(button2Pin, INPUT);
  pinMode(button3Pin, INPUT);
  pinMode(button4Pin, INPUT);
  pinMode(button5Pin, INPUT);
  pinMode(button6Pin, INPUT);
  pinMode(button7Pin, INPUT);
  pinMode(button8Pin, INPUT);
  pinMode(button9Pin, INPUT);
  pinMode(button10Pin, INPUT);
  pinMode(button11Pin, INPUT);

  Joystick.begin();

  Joystick.setRxAxis(0);
  Joystick.setRyAxis(0);
  Joystick.setRzAxis(0);
  Joystick.setXAxis(0);
  Joystick.setYAxis(0);
  Joystick.setZAxis(0);
  Joystick.setAccelerator(0);
  Joystick.setBrake(0);
  Joystick.setRudder(0);
  Joystick.setSteering(0);
  Joystick.setThrottle(0);

  Serial.begin(115200);


  startAnimation();

}

void loop() {

  //=======================

    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;

    // if the LED is off turn it on and vice-versa:
    if (flashLight == false) {
      flashLight = true;
    } else {
      flashLight = false;
    }
  
  }

  //============================================
  
  bool button0 = true;
  bool button1 = digitalRead(button1Pin);
  bool button2 = digitalRead(button2Pin);
  bool button3 = digitalRead(button3Pin);
  bool button4 = digitalRead(button4Pin);
  bool button5 = digitalRead(button5Pin);
  bool button6 = digitalRead(button6Pin);
  bool button7 = digitalRead(button7Pin);
  bool button8 = digitalRead(button8Pin);
  bool button9 = digitalRead(button9Pin);
  bool button10 = digitalRead(button10Pin);
  bool button11 = digitalRead(button11Pin);

  if(button1){
    Joystick.pressButton(1);
  }else{
    Joystick.releaseButton(1);
  }

  if(button2){
    Joystick.pressButton(2);
  }else{
    Joystick.releaseButton(2);
  }

  if(button3){
    Joystick.pressButton(3);
  }else{
    Joystick.releaseButton(3);
  }

  if(button4){
    Joystick.pressButton(4);
  }else{
    Joystick.releaseButton(4);
  }

  if(!button1 && !button2 && !button3 && !button4){
    Joystick.pressButton(0);
    button0 = true;
  }else{
    Joystick.releaseButton(0);
    button0 = false;
  }

  if(button5){
    Joystick.pressButton(5);
  }else{
    Joystick.releaseButton(5);
  }

  if(button6){
    Joystick.pressButton(6);
  }else{
    Joystick.releaseButton(6);
  }

  if(button7){
    Joystick.pressButton(7);
  }else{
    Joystick.releaseButton(7);
  }

  if(button8){
    Joystick.pressButton(8);
  }else{
    Joystick.releaseButton(8);
  }

  if(button9){
    Joystick.pressButton(9);
  }else{
    Joystick.releaseButton(9);
  }

  if(button10){
    Joystick.pressButton(10);
  }else{
    Joystick.releaseButton(10);
  }
  
  if(button11){
    Joystick.pressButton(11);
  }else{
    Joystick.releaseButton(11);
  }

  int elevatorValueRaw = analogRead(elevatorPin);
  int elevatorValue = map(elevatorValueRaw, 0, 1023, 1000, 0);
  Joystick.setXAxis(elevatorValue);
  //Serial.println(elevatorValue);

  int armValueRaw = analogRead(armPin);
  int armValue = map(armValueRaw, 0, 1023, 1000, 0);
  Joystick.setYAxis(armValue);
  //Serial.println(armValue);

  if(button1 && button6){
    leds[0] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  if(button2 && button6){
    leds[1] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[1] = CRGB::Black;
    FastLED.show();
  }
  if(button3 && button6){
    leds[2] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[2] = CRGB::Black;
    FastLED.show();
  }
  if(button4 && button6){
    leds[3] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[3] = CRGB::Black;
    FastLED.show();
  }
  if(button4 && button5){
    leds[4] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[4] = CRGB::Black;
    FastLED.show();
  }
  if(button3 && button5){
    leds[5] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[5] = CRGB::Black;
    FastLED.show();
  }
  if(button2 && button5){
    leds[6] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[6] = CRGB::Black;
    FastLED.show();
  }
  if(button1 && button5){
    leds[7] = CRGB::OrangeRed;
    FastLED.show();
  }else{
    leds[7] = CRGB::Black;
    FastLED.show();
  }

//Delta Light
 /*if(){
  leds[8] = CRBG::OrangeRed;
 }else{

 }*/

 //Gamepiece Light
 if(gamepiece_value == 1){
  leds[9] = CRGB::Black;
  FastLED.show();
 }else if(gamepiece_value == 2){
  leds[9] = CRGB::Blue;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[9] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[9] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Camera Light
 if(camera_value == 1){
  leds[10] = CRGB::Yellow;
  FastLED.show();
 }else if(camera_value == 2){
  leds[10] = CRGB::OrangeRed;
  FastLED.show();
 }else if(camera_value == 3){
  leds[10] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[10] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[10] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Gyro Light
 if(gyro_value == 1){
  leds[11] = CRGB::Red;
  FastLED.show();
 }else if(gyro_value == 2){
  leds[11] = CRGB::Yellow;
  FastLED.show();
 }else if(gyro_value == 3){
  leds[11] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[11] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[11] = CRGB::Black;
    FastLED.show();
  }
  
 }

  //Match Time
if(matchtime_value >= 5400){
  leds[22] = CRGB::Blue;
  leds[21] = CRGB::Blue;
  leds[20] = CRGB::Blue;
  leds[19] = CRGB::Blue;
  leds[18] = CRGB::Blue;
  FastLED.show();
}else if(matchtime_value >= 4050 && matchtime_value < 5400){
  leds[22] = CRGB::Black;
  leds[21] = CRGB::Green;
  leds[20] = CRGB::Green;
  leds[19] = CRGB::Green;
  leds[18] = CRGB::Green;
  FastLED.show();
}else if(matchtime_value >= 2700 && matchtime_value < 4050){
  leds[22] = CRGB::Black;
  leds[21] = CRGB::Black;
  leds[20] = CRGB::Yellow;
  leds[19] = CRGB::Yellow;
  leds[18] = CRGB::Yellow;
  FastLED.show();  
}else if(matchtime_value >= 1350 && matchtime_value < 2700){
  leds[22] = CRGB::Black;
  leds[21] = CRGB::Black;
  leds[20] = CRGB::Black;
  leds[19] = CRGB::OrangeRed;
  leds[18] = CRGB::OrangeRed;
  FastLED.show();
}else if(matchtime_value >= 0 && matchtime_value < 1350){
  leds[22] = CRGB::Black;
  leds[21] = CRGB::Black;
  leds[20] = CRGB::Black;
  leds[19] = CRGB::Black;
  leds[18] = CRGB::Red;
}else{
  
  if(flashLight == true){
  leds[18] = CRGB::Red;
  leds[19] = CRGB::Red;
  leds[20] = CRGB::Red;
  leds[21] = CRGB::Red;
  leds[22] = CRGB::Red;
    FastLED.show();
  }else{
  leds[18] = CRGB::Black;
  leds[19] = CRGB::Black;
  leds[20] = CRGB::Black;
  leds[21] = CRGB::Black;
  leds[22] = CRGB::Black;
    FastLED.show();
  }
  
 }



  //Drivetrain Light
 if(drivetrain_value == 1){
  leds[17] = CRGB::Red;
  FastLED.show();
 }else if(drivetrain_value == 2){
  if(flashLight == true){
    leds[17] = CRGB::Yellow;
    FastLED.show();
  }else{
    leds[17] = CRGB::Black;
    FastLED.show();
  }
 }else if(drivetrain_value == 3){
  leds[17] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[17] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[17] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Elevator Light
 if(elevator_value == 1){
  leds[16] = CRGB::Red;
  FastLED.show();
 }else if(elevator_value == 2){
  if(flashLight == true){
    leds[16] = CRGB::Yellow;
    FastLED.show();
  }else{
    leds[16] = CRGB::Black;
    FastLED.show();
  }
 }else if(elevator_value == 3){
  leds[16] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[16] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[16] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Gantry Arm Light
 if(gantryarm_value == 1){
  leds[15] = CRGB::Red;
  FastLED.show();
 }else if(gantryarm_value == 2){
  if(flashLight == true){
    leds[15] = CRGB::Yellow;
    FastLED.show();
  }else{
    leds[15] = CRGB::Black;
    FastLED.show();
  }
 }else if(gantryarm_value == 3){
  leds[15] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[15] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[15] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Intake Light
 if(intake_value == 1){
  leds[14] = CRGB::Red;
  FastLED.show();
 }else if(intake_value == 2){
  leds[14] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[14] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[14] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Funnel Light
 if(funnel_value == 1){
  leds[13] = CRGB::Red;
  FastLED.show();
 }else if(funnel_value == 2){
  leds[13] = CRGB::Purple;
  FastLED.show();
 }else if(funnel_value == 3){
  leds[13] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[13] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[13] = CRGB::Black;
    FastLED.show();
  }
  
 }

 //Climber Light
 if(climber_value == 1){
  leds[12] = CRGB::Red;
  FastLED.show();
 }else if(climber_value == 2){
  if(flashLight == true){
    leds[12] = CRGB::Yellow;
    FastLED.show();
  }else{
    leds[12] = CRGB::Black;
    FastLED.show();
  }
 }else if(climber_value == 3){
  leds[12] = CRGB::Purple;
  FastLED.show();
 }else if(climber_value == 4){
  leds[12] = CRGB::Green;
  FastLED.show();
 }else{
  
  if(flashLight == true){
    leds[12] = CRGB::DarkRed;
    FastLED.show();
  }else{
    leds[12] = CRGB::Black;
    FastLED.show();
  }
  
 }
 
 


recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            // this temporary copy is necessary to protect the original data
            //   because strtok() used in parseData() replaces the commas with \0
        parseData();
        //showParsedData();
        newData = false;
    }




}

//============

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
                receivedChars[ndx] = '\0'; // terminate the string
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

//============

void parseData() {      // split the data into its parts

    char * strtokIndx; // this is used by strtok() as an index

    strtokIndx = strtok(tempChars,",");      // get the first part - the string
    gamepiece_value = atoi(strtokIndx);     // convert this part to an integer
 
    strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
    camera_value = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    gyro_value = atoi(strtokIndx);     // convert this part to an integer

    strtokIndx = strtok(NULL, ",");
    matchtime_value = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    drivetrain_value = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    elevator_value = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    gantryarm_value = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    intake_value = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    funnel_value = atoi(strtokIndx);

    strtokIndx = strtok(NULL, ",");
    climber_value = atoi(strtokIndx);

}

//============

void showParsedData() {
    Serial.print("Gamepiece?: ");
    Serial.println(gamepiece_value);
    Serial.print("Camera?:  ");
    Serial.println(camera_value);
    Serial.print("Gyro?: ");
    Serial.println(gyro_value);
    Serial.print("Match Time?: ");
    Serial.println(matchtime_value);
    Serial.print("Drivetrain?:  ");
    Serial.println(drivetrain_value);
    Serial.print("Elevator?: ");
    Serial.println(elevator_value);
    Serial.print("Arm?:  ");
    Serial.println(gantryarm_value);
    Serial.print("Intake?: ");
    Serial.println(intake_value);
    Serial.print("Funnel?: ");
    Serial.println(funnel_value);
    Serial.print("Climber?:  ");
    Serial.println(climber_value);
}

void startAnimation() {

  for(int i = 0; i < 23; i++){
    leds[i] = CRGB::OrangeRed;
    FastLED.show();
    delay(80);
  }
  leds[0] = CRGB::Black;
  leds[1] = CRGB::Black;
  leds[2] = CRGB::Black;
  leds[3] = CRGB::Black;
  leds[4] = CRGB::Black;
  leds[5] = CRGB::Black;
  leds[6] = CRGB::Black;
  leds[7] = CRGB::Black;
  leds[8] = CRGB::Black;
  leds[9] = CRGB::Black;
  leds[10] = CRGB::Black;
  leds[11] = CRGB::Black;
  leds[12] = CRGB::Black;
  leds[13] = CRGB::Black;
  leds[14] = CRGB::Black;
  leds[15] = CRGB::Black;
  leds[16] = CRGB::Black;
  leds[17] = CRGB::Black;
  leds[18] = CRGB::Black;
  leds[19] = CRGB::Black;
  leds[20] = CRGB::Black;
  leds[21] = CRGB::Black;
  leds[22] = CRGB::Black;
  FastLED.show();

}



