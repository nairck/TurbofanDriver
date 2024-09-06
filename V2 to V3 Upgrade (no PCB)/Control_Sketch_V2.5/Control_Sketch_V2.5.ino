/* Control_Sketch_V3

    This program used an L298N motor driver with a 12VDC motor.
    The 20x4 LCD displays motor speed, turbofan speed, and motor voltage.
    An encoder is used to control the motor speed.
    BUTTON INPUTS TO THE ENCODER:
      Press the button to set speed back to zero.
      Hold the button for 2 seconds to set the Maximum Fan speed.
      Hold the button for 5 seconds to reset the Maximum Fan speed to be uncapped.
    Maximum speed set is remembered inside the program even after power down.

    Visit https://www.thingiverse.com/thing:4743929 for the project directory

    Written and designed by Adam B Johnson
    in British Columbia, Canada
    V1 - 15 January 2021
    V2 - 19 November 2022
    V3 - 9 August 2024
       - changed moving average window filter for voltage measurement to single pole RC type low-pass filter (simpler/better).
       - bug fix when turning encoder from higher value to zero wouldn't work. Encoder can bounce still, but zeroing now works a lot better.
       - bug fix where maximum pwm input was equal to maximum allowed minus the deadband
       - bug fixs for EEPROM code, setting new value would reset initialization sometimes or not work at all.
       - Serial monitor will now show encoder position and measured voltage at the sample rate for debugging purposes. Does not affect program when not in use.

    KNOWN ISSUES - THE ENCODER CAN BOUNCE AROUND WHEN TURNING THE SPEED TO ZERO
                   FROM SOME HIGHER SPEED OR WHEN TURNING THE ENCODER IN THE NEGATIVE
                   DIRECTION WHEN AT ZERO. V3 UPDATES SHOULD IMPROVE THIS, BUT MECHANICALLY, THE 
                   DETENTING CAN CAUSE BOUNCING. THE PUSHBUTTON TO RESET THE SPEED SHOULD ALWAYS
                   WORK, OR THE POWER BUTTON.

                 - THE SCREEN FLICKERS. I HAVE TRIED A CAPACITOR AND IT DOESN'T HELP MUCH.
                   I HAVE READ ITS FAIRLY COMMON FOR 'CHEAPER' LCDs.

*/

#define gearRatio 2.27              //fixed gear ratio

#include <Encoder.h>                //Might need to download this library. Tools > Manage Libraries > Search for 'Encoder'
#include <LiquidCrystal_I2C.h>      //Might also need to download this one in the same way.
#include <EEPROM.h>                 //This should have been included when you installed Arduino IDE. No download needed

//##################  Adjust these values if needed #######################################

#define chB 2                           //***pin B of the rotary encoder (channel B should be pin 2). Can be non-interrupt capable pin, but will have lower performance
#define chA 3                           //***pin A of the rotary encoder (channel A should be pin 3). Can be non-interrupt capable pin, but will have lower performance
#define pushButton 4                    //***Button pin           
#define motorPWM 6                      //***PWM pin to motor controller 
#define motorDir1 9                     //***motor direction forward pin to driver
#define motorDir2 10                    //***motor direction reverse pin to driver 
//#define  motorVoltagePin A0            //***Analog pin for motor voltage measurement

float motorRPMperVolt = (5000.0 / 12.0);         //***DC motor max RPM (4825) divided by max voltage (12VDC. Measure both paraments and adjust
LiquidCrystal_I2C lcd(0x27, 20, 4);     //***Adjust as needed. Use File > Examples > Wire > i2c_scanner to acquire address. 0x27 for replacement screen, 0x3F for original 
Encoder myEncoder(chA, chB);

//##################  End of adjustable parameters #######################################

// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& PROGRAM CONSTANTS &&&&&&&&&&&&&&&&&&&&&&&
// &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&& &&&&&&&&&&&&&&&&&&&&&&&

const float sampleR = 0.5; // 500ms sample rate

int eeAddress = 0;
int eeAddressForInitialization = 100;
byte initializationConst = 0;
byte pwmMax = 255;
int pwmVal = 0;
bool setFlag = false;

int fanSpeed = 0;                   //speed of the turbofan in RPM
int mtrSpeed = 0;                   //speed of the motor in RPM
int deadBand = 30;                  //minimum PWM signal for the motor to move
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
  lcd.backlight();
  lcd.setCursor ( 1, 0 );
  lcd.print("V3 Turbofan Driver");
  lcd.setCursor ( 0, 2 );
  lcd.print("Adam B. Johnson");
  lcd.setCursor( 0, 3);
  lcd.print("BC, Canada");
  lcd.setCursor ( 16, 3 );
  lcd.print("2024");
  delay(2000);


  //first - flash setup (only works if EEPROM untouched since factory)
  EEPROM.get(eeAddressForInitialization, initializationConst);
  if  (initializationConst != 1) {
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

  // get max speed
  EEPROM.get(eeAddress, pwmMax);

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

  //get encoder position
  newPos = myEncoder.read();

  // update screen
  if (abs(timeNew - timeOld) >= ((unsigned long)(1000.0 * sampleR))) {
    timeOld = timeNew;

    // read and filter voltage
    float f_cutoff = 0.25;
    float alpha = (2.0 * 3.14 * f_cutoff * sampleR) / ((2.0 * 3.14 * f_cutoff * sampleR) + 1.0);
    roughMotorVoltage = (((1.0 - alpha) * roughMotorVoltage) + (alpha * (analogRead(A0) * 12.0 / 1024.0)));
    roughMotorVoltage = ((int)(roughMotorVoltage * 100.0)) / 100.0;



    if (newPos != oldPos) {
      if ((newPos > 0) && (newPos <= deadBand) && (oldPos < deadBand)) {
        oldPos = newPos;
        newPos = deadBand;
        analogWrite(motorPWM, newPos);
        myEncoder.write(deadBand);
      } else if ((newPos > deadBand) && (newPos <= pwmMax)) {
        analogWrite(motorPWM, newPos);
        oldPos = newPos;
      } else if (newPos > pwmMax) {
        oldPos = pwmMax;
        newPos = pwmMax;
        analogWrite(motorPWM, pwmMax);
        myEncoder.write((int)pwmMax);
        delay(50);
        for (int i = 0; i < 20; i++) {
          lcd.setCursor(i, 0);
          lcd.print(" ");
        }
        lcd.setCursor(1, 0);
        lcd.print("Max Speed Reached!");
        delay(750);
        for (int i = 0; i < 20; i++) {
          lcd.setCursor(i, 0);
          lcd.print(" ");
        }
        lcd.setCursor(2, 0);
        lcd.print("V3 Turbofan Stats");
      } else if ((newPos <= 0) || (newPos < deadBand) && (oldPos >= deadBand)) {
        analogWrite(motorPWM, 0);
        myEncoder.write(0);
        clearNums();
        zeroNums();
        oldPos = 0;
        newPos = 0;
        delay(1000);
      }
    }
    Serial.print(newPos);
    Serial.print("    ");
    Serial.println(roughMotorVoltage);

    if (abs(roughMotorVoltage - oldVolt) >= 0.005) {
      // mtrSpeed = motorRPMperVolt * ((2.0 * roughMotorVoltage) - ((float)newPos * 12.0 / 255.0));     // this should be a good enough estimate for armature current, but doesn't work well
      mtrSpeed = motorRPMperVolt * roughMotorVoltage;
      fanSpeed = mtrSpeed / gearRatio;
      oldVolt = roughMotorVoltage;
      clearNums();
      printVoltage();
      printSpeeds();
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
          lcd.print("V3 Turbofan Stats");
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
          byte temp1 = newPos;
          if (temp1 > 255) temp1 = 255;
          if (temp1 < 0) temp1 = 0;
          EEPROM.put(eeAddress, temp1);
          pwmMax = temp1;
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
            // get max speed
            pwmMax = 255;
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
        lcd.print("V3 Turbofan Stats");
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
  roughMotorVoltage = 0.0;
  delay(100);
  printSpeeds();
  printVoltage();
}


void screenSetup() {
  lcd.setCursor (2, 0 );
  lcd.print("V3 Turbofan Stats");
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
