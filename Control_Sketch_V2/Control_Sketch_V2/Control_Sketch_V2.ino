/* Control_Sketch_V2

    This program used an L298N motor driver with a 12VDC motor.
    The 20x4 LCD displays motor speed, turbofan speed, and motor voltage.
    An encoder is used to control the motor speed.
    ON THE FIRST STARTUP: Press the button to set speed back to zero.
    Hold the button for 2 seconds to set the Maximum Fan RPM
    Hold the button for 5 seconds to reset the Maximum Fan RPM to uncapped.
    Maximum RPM set is remembered inside the program even after power down.

    Visit https://www.thingiverse.com/thing:4743929 for the project directory

    Written and designed by Adam B Johnson
    in British Columbia, Canada
    V1 - 15 January 2021
    V2 - 19 November 2022

    KNOWN ISSUES - THE ENCODER CAN BOUNCE AROUND WHEN TURNING THE SPEED TO ZERO
                   FROM SOME HIGHER SPEED OR FROM ZERO. A WORK AROUND TO ZERO THE MOTOR IS
                   TO PRESS THE BUTTON ONCE TO RESET THE SPEED. THE LONGER DELAY SEEMS
                   TO WORK WELL. POWER BUTTON RESET ALSO WORKS...
                   ANY SOFTWARE SUGGESTIONS ARE WELCOME. DECOUPLING CAPACITORS
                   DIDN'T MAKE A LARGE DIFFERENCE.

                 - THE SCREEN FLICKERS. I HAVE TRIED A CAPACITOR AND IT DOESN'T HELP MUCH.
                   I HAVE READ ITS FAIRLY COMMON FOR 'CHEAPER' LCDs, AND THE BACK EMF FROM
                   THE MOTOR IS LIKELY CAUSING SOME LARGE VOLTAGE SPIKES ACROSS THE CIRCUIT.

*/

#define gearRatio 2.27              //fixed gear ratio

#include <Encoder.h>                //Might need to download this library. Tools > Manage Libraries > Search for 'Encoder'
#include <LiquidCrystal_I2C.h>      //Might also need to download this one in the same way.
#include <EEPROM.h>                 //This should have been included when you installed Arduino IDE. No download needed

//##################  Adjust these values if needed #######################################

#define chB 2                           //***pin B of the rotary encoder (channel B)
#define chA 3                           //***pin A of the rotary encoder (channel A)
#define pushButton 4                    //***Button pin           
#define motorPWM 6                      //***PWM pin to motor controller 
#define motorDir1 9                     //***motor direction forward pin to driver
#define motorDir2 10                    //***motor direction reverse pin to driver 
#define WINDOW_SIZE 20
//#define  motorVoltagePin A0            //***Analog pin for motor voltage measurement

float motorRPMperVolt = (4825.0 / 12.0);         //***DC motor max RPM (4825) divided by max voltage (12VDC. Measure both paraments and adjust
LiquidCrystal_I2C lcd(0x3F, 20, 4);     //***Adjust screen address if needed. Use File > Examples > Wire > i2c_scanner to acquire address and change 0x3F as needed.
Encoder myEncoder(chA, chB);

//##################  End of adjustable parameters #######################################

// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& PROGRAM CONSTANTS &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&

int INDEX = 0;
int VALUE = 0;
int SUM = 0;
int READINGS[WINDOW_SIZE];
int AVERAGED = 0;

int eeAddress = 0;
int eeAddressForInitialization = 1;
int initializationConst = 0;
int pwmMax;
int pwmVal = 0;
bool setFlag = false;

int fanSpeed = 0;                   //speed of the turbofan in RPM
int mtrSpeed = 0;                   //speed of the motor in RPM
int deadBand = 20;                  //minimum PWM signal for the motor to move
float vout = 0.0;                   //Voltage out of the motor
float roughMotorVoltage = 0.0;
float oldVolt = 0.0;                //check to see if voltage changes after each loop

long timeNew = 0;                   //time stamp at beginning of loop
long timeOld = 0;                   //time stamp at end of end
long tempTimer = 0;
long oldPos = 0;                    //old position of the encoder
long newPos = 0;                    //new position of the encoder

void setup() {
  Serial.begin(9600);
  pinMode(A0, INPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);
  pinMode(motorPWM, OUTPUT);
  pinMode(pushButton, INPUT_PULLUP);
  pinMode(chA, INPUT_PULLUP);
  pinMode(chB, INPUT_PULLUP);

  myEncoder.write(oldPos);
  digitalWrite(motorDir1, HIGH);  //motor direction (FWD = HIGH). DON'T CHANGE. SWITCH THE WIRES IF DIRECTION IS WRONG
  digitalWrite(motorDir2, LOW);   //motor direction (REV = LOW). DON'T CHANGE. SWITCH THE WIRES IF DIRECTION IS WRONG
  analogWrite(motorPWM, 0);

  lcd.init();
  lcd.setCursor ( 1, 0 );
  lcd.print("V2 Turbofan Driver");
  lcd.setCursor ( 0, 2 );
  lcd.print("Adam B. Johnson");
  lcd.setCursor( 0, 3);
  lcd.print("BC, Canada");
  lcd.setCursor ( 16, 3 );
  lcd.print("2022");
  delay(2000);


  //first - flash setup (only works if EEPROM untouched since factory)
  EEPROM.get(eeAddressForInitialization, initializationConst);
  if  (initializationConst = ! 1) {
    EEPROM.put(eeAddress, 255);
    EEPROM.put(eeAddressForInitialization, 1);
    lcd.clear();
    lcd.setCursor ( 0, 0 );
    lcd.print("First program flash");
    lcd.setCursor( 1, 1);
    lcd.print("detected... ");
    lcd.setCursor ( 2, 2 );
    lcd.print("resetting maximum ");
    lcd.setCursor ( 2, 3 );
    lcd.print("speed setting...");
    delay(10000);
  }

  lcd.clear();
  screenSetup();
  zeroNums();
  printSpeeds();
  printVoltage();
  myEncoder.write(0);
  analogWrite(motorPWM, 0);
}


// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& MAIN LOOP &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&

void loop() {

  // start loop timer
  timeNew = millis();
  EEPROM.get(eeAddress, pwmMax);

  // moving average of readings
  SUM = SUM - READINGS[INDEX];       // Remove the oldest entry from the sum
  VALUE = analogRead(A0);        // Read the next sensor value
  READINGS[INDEX] = VALUE;           // Add the newest reading to the window
  SUM = SUM + VALUE;                 // Add the newest reading to the sum
  INDEX = (INDEX + 1) % WINDOW_SIZE; // Increment the index, and wrap to 0 if it exceeds the window size
  AVERAGED = SUM / WINDOW_SIZE;      // Divide the sum of the window by the window size for the result
  roughMotorVoltage = AVERAGED * 12.0 / 1024.0;

  //  Serial.println(roughMotorVoltage);

  // update screen
  if (timeNew - timeOld > 300) {
    timeOld = timeNew;
    if (abs(roughMotorVoltage - oldVolt) >= 0.01) {
      mtrSpeed = motorRPMperVolt * roughMotorVoltage;
      fanSpeed = mtrSpeed / gearRatio;
      oldVolt = roughMotorVoltage;
      clearNums();
      printVoltage();
      printSpeeds();
    }
  }

  //get encoder position
  Serial.println(pwmMax);
  newPos = myEncoder.read();
  if (newPos != oldPos) {
    if ((newPos > 0) && ((newPos + deadBand) <= pwmMax)) {
      analogWrite(motorPWM, (newPos + deadBand));
      oldPos = newPos;
    } else if ((newPos + deadBand) > pwmMax) {
      analogWrite(motorPWM, pwmMax);
      delay(50);
      newPos = oldPos;
      for (int i = 0; i < 20; i++) {
        lcd.setCursor(i, 0);
        lcd.print(" ");
      }
      lcd.setCursor(1, 0);
      lcd.print("Max Speed Reached!");
      delay(1000);
      for (int i = 0; i < 20; i++) {
        lcd.setCursor(i, 0);
        lcd.print(" ");
      }
      lcd.setCursor(2, 0);
      lcd.print("V2 Turbofan Stats");
      myEncoder.write(oldPos);
    } else if (newPos <= 0) {
      analogWrite(motorPWM, 0);
      myEncoder.write(0);
      clearNums();
      zeroNums();
      newPos = oldPos;
      delay(1000);
      myEncoder.write(0);
    }
  }


  // button press protocols
  if (digitalRead(pushButton) == LOW) {
    delay(100);
    if (digitalRead(pushButton) == LOW) {
      delay(300);

      //set current speed to zero - used to turn off fan quickly
      if (digitalRead(pushButton) == HIGH) {
        delay(100);
        if (digitalRead(pushButton) == HIGH) {
          analogWrite(motorPWM, 0);
          for (int i = 0; i < 20; i++) {
            lcd.setCursor(i, 0);
            lcd.print(" ");
          }
          lcd.setCursor(4, 0);
          lcd.print("Speed Reset");
          delay(3000);
          for (int i = 0; i < 20; i++) {
            lcd.setCursor(i, 0);
            lcd.print(" ");
          }
          lcd.setCursor(2, 0);
          lcd.print("V2 Turbofan Stats");
          myEncoder.write(0);
          setFlag = true;
          clearNums();
          zeroNums();
        }
      }

      // Maximum speed set to current value
      if ((digitalRead(pushButton) == LOW)) {
        delay(2000);
        setFlag = false;
        if ((digitalRead(pushButton) == LOW) && (setFlag == false)) {
          int temp1 = (newPos + deadBand);
          if (temp1 > 255) temp1 = 255;
          if (temp1 < 0) temp1 = 0;
          EEPROM.put(eeAddress, temp1);
          for (int i = 0; i < 20; i++) {
            lcd.setCursor(i, 0);
            lcd.print(" ");
          }
          lcd.setCursor(1, 0);
          lcd.print("Max Speed Set: ");
          lcd.setCursor(16, 0);
          lcd.print(fanSpeed);
          delay(3000);

          //reset maximum speed
          if (digitalRead(pushButton) == LOW) {
            EEPROM.put(eeAddress, 255);
            for (int i = 0; i < 20; i++) {
              lcd.setCursor(i, 0);
              lcd.print(" ");
            }
            lcd.setCursor(2, 0);
            lcd.print("Max Speed Reset");
            delay(3000);
          }
        }
        for (int i = 0; i < 20; i++) {
          lcd.setCursor(i, 0);
          lcd.print(" ");
        }
        lcd.setCursor(2, 0);
        lcd.print("V2 Turbofan Stats");
        setFlag = false;
      }
    }
  }
}


// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& FUNCTIONS &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&

void printVoltage() {
  if ((roughMotorVoltage > 0) && (roughMotorVoltage < 10.0)) {
    lcd.setCursor (12, 3 );
  } else if ((roughMotorVoltage < 15.0) && (roughMotorVoltage >= 10.0)) {
    lcd.setCursor (11, 3 );
  } else {
    lcd.setCursor (12, 3 );
    roughMotorVoltage = 0;
  }
  lcd.print(roughMotorVoltage, 2);
}


void printSpeeds() {
  if (fanSpeed < 100 && fanSpeed >= 10) {
    lcd.setCursor(14, 1);
  } else if (fanSpeed < 1000 && fanSpeed >= 100) {
    lcd.setCursor (13, 1 );
  } else if (fanSpeed < 10000 && fanSpeed >= 1000) {
    lcd.setCursor (12, 1 );
  } else if (fanSpeed == 0) {
    lcd.setCursor (15, 1 );
  } else {
    lcd.setCursor (15, 1 );
    fanSpeed = 0;
  }
  lcd.print(fanSpeed);
  if (mtrSpeed < 100 && mtrSpeed >= 10) {
    lcd.setCursor(14, 2);
  } else if (mtrSpeed < 1000 && mtrSpeed >= 100) {
    lcd.setCursor (13, 2 );
  } else if (mtrSpeed < 10000 && mtrSpeed >= 1000) {
    lcd.setCursor (12, 2 );
  } else if (mtrSpeed == 0) {
    lcd.setCursor (15, 2 );
  } else {
    lcd.setCursor (15, 2 );
    mtrSpeed = 0;
  }
  lcd.print(mtrSpeed);
}


void clearNums() {
  for (int i = 11; i < 17; i++) {
    for (int j = 1; j < 4; j++) {
      lcd.setCursor (i, j);
      lcd.print(" ");
    }
  }
}


void zeroNums() {
  oldPos = 0;
  mtrSpeed = 0;
  fanSpeed = 0;
  delay(100);
  printSpeeds();
  printVoltage();
}


void screenSetup() {
  lcd.setCursor ( 2, 0 );
  lcd.print("V2 Turbofan Stats");
  lcd.setCursor ( 0, 1 );
  lcd.print("Fan Speed:");
  lcd.setCursor (17, 1 );
  lcd.print("RPM");
  lcd.setCursor ( 0, 2 );
  lcd.print("Mtr Speed:");
  lcd.setCursor (17, 2 );
  lcd.print("RPM");
  lcd.setCursor ( 0, 3 );
  lcd.print("Voltage:");
  lcd.setCursor (17, 3 );
  lcd.print("VDC");
}
