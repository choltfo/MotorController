
//#define USE_LCD

#ifdef USE_LED
#include "Wire.h"
#include "Adafruit_LiquidCrystal.h"
#endif

#include <SPI.h>

// Why am I writing this in Arduino again?
// Pin mappings:
const uint8_t An = 5;
const uint8_t Ap = 9;
const uint8_t Bn = 6;
const uint8_t Bp = 3;

//const uint8_t fanControl = 9;
const uint8_t vBat = A1;

const uint8_t FaultN = 8;
const uint8_t SleepN = A0;
const uint8_t SPI_CS = 10; // active high

//const uint8_t TachA1 = 2;
//const uint8_t TachA2 = A2;
//const uint8_t TachB1 = 3;
//const uint8_t TachB2 = A3;


const uint16_t UndervoltageMask = 0x20;
const uint16_t PredriverBMask = 0x10;
const uint16_t PredriverAMask = 0x08;
const uint16_t OverccurentBMask = 0x04;
const uint16_t OverccurentAMask = 0x02;
const uint16_t OverTempMask = 0x01;


// Control objects
#ifdef USE_LED
Adafruit_LiquidCrystal lcd(0);
#endif



uint8_t speedA = 0;
bool reverseA = false;
uint8_t speedB = 0;
bool reverseB = false;

bool braking = true;    // When speedX is 0, coast or brake?

bool interlockClosed = false;


// Writes to the registers on the DRV8704
void writeReg(byte reg, uint16_t data) {
  uint16_t output = 0x0000 | (reg << 12) | data;
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, HIGH);
  uint16_t dat = SPI.transfer16(output);
  digitalWrite(SPI_CS, LOW);
  SPI.endTransaction();
}

// Reads the value of a register on the DRV8704
uint16_t readReg(byte reg) {
  uint16_t output = 0x8000 | (reg << 12);
  SPI.beginTransaction(SPISettings(100000, MSBFIRST, SPI_MODE0));
  digitalWrite(SPI_CS, HIGH);
  uint16_t dat = SPI.transfer16(output);
  digitalWrite(SPI_CS, LOW);
  SPI.endTransaction();
  return dat;
}


void setup() {

  Serial.begin(57600);

  Serial.println("Initializing...");

//  lcd.begin(20, 4);
//  lcd.setBacklight(HIGH);
//  lcd.setCursor(0,1);
//  lcd.println("Motor Driver");
//  lcd.println("Holforster");
  
  // Set pin directions
  pinMode(An, OUTPUT);
  pinMode(Ap, OUTPUT);
  pinMode(Bn, OUTPUT);
  pinMode(Bp, OUTPUT);

  digitalWrite(An, LOW);
  digitalWrite(Ap, LOW);
  digitalWrite(Bn, LOW);
  digitalWrite(Bp, LOW);
  
  //pinMode(fanControl, OUTPUT); // The fan might want 12 V. If it won't start, change this to INPUT and leave it floating.

  pinMode(FaultN, INPUT);

  pinMode(SleepN, OUTPUT);
  digitalWrite(SleepN, HIGH); // Turn ON the driver for now

  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, LOW);

  /*pinMode(TachA1, INPUT);
  pinMode(TachA2, INPUT);
  pinMode(TachB1, INPUT);
  pinMode(TachB2, INPUT);
  // TODO: Set up ISRs on the tachometer pins*/

  // Set up SPI bus with default lines and pins
  SPI.begin();

  
  // Say hello!
  chirpMotor(An,Ap,955,100); // C6
  chirpMotor(An,Ap,1318,250); // E6
  digitalWrite(SleepN, LOW); // Turn OFF the driver for now
  delay(500);
  
  
  /*// Beep out the voltage
  int batteryVolts = (getBatteryVoltage() + 500) / 1000;

  // Tens of volts - beep beep
  for (int i = 0; i < batteryVolts/10; ++i) {
    chirpMotor(An,Ap,955,100);
    delay(100);
  }
  delay(100);

  // Volts
  for (int i = 0; i < batteryVolts%10; ++i) {
    chirpMotor(Bn,Bp,1175,100); // D6
    delay(100);
  }*/

  
  // Power chord/arpegio!
  digitalWrite(SleepN, HIGH); // Turn ON the driver for now
  chirpMotor(An,Ap,1911,150); // C5
  chirpMotor(An,Ap,1275,150); // G5
  chirpMotor(An,Ap,955,150); // C6
  
  delay(50); // Give the motors time to stop cold
  
  interlockClosed = false;
  digitalWrite(SleepN, LOW); // Put motor driver to sleep
  
  Serial.println("Initialized; Locked out.");
  
}


// Check whether the motor driver has experienced a fault, and return a representative bitmask.
uint16_t checkFault() {
  if (digitalRead(FaultN) == false) {
    uint16_t reg = readReg(7);
    return reg & 0x3F;
  } else {
    return 0;
  }
}

// pn is xn pin
// pp is xp pin
// spd is speed (0-255)
// rev is reverse, as opposed to go forward.
// brk is brake, as opposed to coast, when spd == 0.
void setMotorSpeed(uint8_t pn, uint8_t pp, uint8_t spd, bool rev, bool brk) {
  /*
   *    |  fwd | rev | brk | cst
   * pn |   +  |  -  |  +  |  -
   * pp | -pwm | pwm |  +  |  -
   */
  if (spd == 0) {
    // If brk, this will brake.
    // Else, coast!
    digitalWrite(pn, brk);
    digitalWrite(pp, brk);
    return;
  }
  
  if (rev) {
    // Reverse
    digitalWrite(pp, LOW); // Direction
    analogWrite(pn, spd);
    
  } else {
    // Forward
    analogWrite(pp, spd);
    digitalWrite(pn, LOW); // Direction
  }  
}


void updateMotors() {
  bool actuallyBraking = (speedA==0) && (speedB == 0) && braking;
  setMotorSpeed(An, Ap, speedA, reverseA, actuallyBraking);
  setMotorSpeed(Bn, Bp, speedB, reverseB, actuallyBraking);
}


void chirpMotor(uint8_t pn, uint8_t pp, uint16_t microPeriod, uint16_t milliLen) {
  for (uint32_t i = 0; i < 1000L*milliLen; i += microPeriod) {
    digitalWrite(pn, LOW); // Forward
    digitalWrite(pp, HIGH);
    delayMicroseconds (microPeriod/2);
    digitalWrite(pn, HIGH); // Brake
    digitalWrite(pp, HIGH);
    delayMicroseconds (microPeriod/2);
  }
  // Resume whatever it is that we were supposed to be doing. Hopefully nothing?
  updateMotors(); 
}

// Returns the total battery voltage in millivolts
uint32_t getBatteryVoltage() {
  // The voltage divider constant is 6.
  // Each increment is 4.9 mV, or (5000/1023) mV.
  return analogRead(vBat) * 29;
}

#ifdef USE_LED
unsigned long int lMillis = 0;
void setStatusScreen() {
  /* |01234567890123456789
   * |Battery:    12.123 V
   * |A: brk   -255
   * |B: rev   +255
   * |
   */
  uint32_t vbat = getBatteryVoltage();
  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Vbat:");
  //lcd.print(vbat,DEC);
  lcd.setCursor(14,0);
  lcd.print(1000 + (vbat%1000),DEC);
  lcd.setCursor(14,0);
  lcd.print(".");
  lcd.setCursor(12,0);
  lcd.print(vbat/1000,DEC);
  
  lcd.setCursor(0,1);
  lcd.print("A:");
  if (speedA == 0) {
  if (braking)
    lcd.print(" brk ");
  else 
    lcd.print(" cst ");
  } else {
  if (reverseA)
    lcd.print(" rev ");
  else 
    lcd.print(" fwd ");
  }
  lcd.setCursor(7,1);
  lcd.print("   ");
  lcd.setCursor(7,1);
  lcd.print(speedA, DEC);
  
  lcd.setCursor(0,2);
  lcd.print("B:");
  if (speedB == 0) {
  if (braking)
    lcd.print(" brk ");
  else 
    lcd.print(" cst ");
  } else {
  if (reverseB)
    lcd.print(" rev ");
  else 
    lcd.print(" fwd ");
  }
  lcd.setCursor(7,2);
  lcd.print("   ");
  lcd.setCursor(7,2);
  lcd.print(speedB, DEC);

  uint16_t fReg = checkFault();
  lcd.setCursor(0,3);
  if (fReg) {
/*
const uint16_t UndervoltageMask = 0x20;
const uint16_t PredriverBMask = 0x10;
const uint16_t PredriverAMask = 0x08;
const uint16_t OverccurentBMask = 0x04;
const uint16_t OverccurentAMask = 0x02;
const uint16_t OverTempMask = 0x01;
*/
    if (fReg & UndervoltageMask)
      lcd.print(" UVLO");
    if (fReg & PredriverBMask)
      lcd.print(" BPDF");
    if (fReg & PredriverAMask)
      lcd.print(" APDF");
    if (fReg & OverccurentBMask)
      lcd.print(" BOCP");
    if (fReg & OverccurentAMask)
      lcd.print(" AOCP");
    if (fReg & OverTempMask)
      lcd.print(" OTS");
  } else {
    lcd.print("                    ");
  }
  
  
  //lcd.setCursor(0,3);
  //lcd.print(millis() - lMillis, DEC);
  lMillis = millis();
}
#endif

unsigned long tLastCommand = 0;

void getSerialUpdate() {
  if (!Serial.available()) return;
  while (Serial.available() && Serial.peek() != '>') {
    Serial.read();
  }
//  Serial.println("Getting packet");
  Serial.read(); // chomp the >.
  char header = Serial.read();
  int16_t in = Serial.parseInt();
  if (header == 'A') {
    speedA = abs(in);
    reverseA = in < 0;
//    Serial.print("A:");
//    Serial.println(in, DEC); 
//    Serial.print("spd: ");
//    Serial.println(speedA, DEC);
//    Serial.print("rev: ");
//    Serial.println(reverseA, DEC);
  }
  if (header == 'B') {
    speedB = abs(in);
    reverseB = in < 0;
//    Serial.print("B:");
//    Serial.println(in, DEC);
  }
  if (header == 'S') {
    braking = in;
//    Serial.print("S:");
//    Serial.println(in, DEC);
  }
  if (header == 'R') {
    // Clear faults on DRV8704
    writeReg(7,0); // Wipes the last 6 bits of register 7
  }
  if (header == 'L') {
    interlockClosed = in;
    if (in == 1) chirpMotor(Bn,Bp,150,20); // 150 us
  }

  // Set overcurrent protection register
  if (header == 'D') {
    uint16_t DRIVEReg= readReg(0x06);
    writeReg(0x06, ((DRIVEReg & 0x0FFC) | (in & 0x03)) & (0x0FFF));
  }

  tLastCommand = millis();
  
  if (Serial.peek() == '\n') Serial.read();
}

bool checkConnectionStatus() {
  return (millis() - tLastCommand) < 100;
}

void sendSerialData() {
  uint32_t vbat = getBatteryVoltage();
  Serial.print("V");
  Serial.print(vbat/1000);
  Serial.print(".");
  Serial.print(vbat%1000);
  
  Serial.print("B");
  if (braking) Serial.print("1");
  else Serial.print("0");

  Serial.print("D:");
  Serial.print(readReg(0x06),HEX);
  
  
  uint16_t fReg = checkFault();
  if (fReg) {
    Serial.print("F");
    if (fReg & UndervoltageMask)
      Serial.print(" UVLO");
    if (fReg & PredriverBMask)
      Serial.print(" BPDF");
    if (fReg & PredriverAMask)
      Serial.print(" APDF");
    if (fReg & OverccurentBMask)
      Serial.print(" BOCP");
    if (fReg & OverccurentAMask)
      Serial.print(" AOCP");
    if (fReg & OverTempMask)
      Serial.print(" OTS");
  }
  
  
  Serial.println();
}

uint32_t updateCount = 0;
bool hasConnection = false;

void loop() {
  /*
   * Get input
   * Get feedback
   * Compute change to output
   * Set output
   * Check faults:
   * - Driver faults
   * - Battery voltage
   * Update screen and else
   */
  getSerialUpdate();

  // Once every half second or so...
  if (updateCount % 25 == 0) {
    sendSerialData();
  }
  
  // LOW VOLTAGE LOCK-OUT
  if (getBatteryVoltage() < 22000) {
    digitalWrite(SleepN, LOW); // Turn off the driver, we're damaging the batteries
    return;
  } else {
    digitalWrite(SleepN, HIGH); // Turn on the driver, everything's fine!
  }
  
  // Once every 200 ms or so...
  if (updateCount % 10 == 0) {
    hasConnection = checkConnectionStatus();
  }

  // System is live
  if (interlockClosed) {
    digitalWrite(SleepN, HIGH);
    
    if (hasConnection) {
      updateMotors();
    } else {
      // Emergency loss of signal.
      // System interlock is engaged, but not connected!
      // Stop wheels, and start beeping madly.
      
      speedA = 0;
      speedB = 0;
      
      // Quick emergency chime
      if (updateCount % 10 == 0) {
        chirpMotor(An,Ap,189,50);
        chirpMotor(Bn,Bp,176,50);
      }
    }

  // System is locked out. Stop everything.
  } else {
    digitalWrite(SleepN, LOW);
    
    speedA = 0;
    speedB = 0;
    updateMotors();
      
    if (hasConnection) {
      // Interlock disabled, connection active
      // Kind, safe chirps to indicate that the system is safed.
      if (updateCount % 100 == 0) {
        chirpMotor(Bn,Bp,131,20); // C3, ish
        chirpMotor(An,Ap,165,20); // E3, ish
      }
    } else {
      // Interlock disabled, no connection. Just hang tight.
      if (updateCount % 125 == 0) {
        chirpMotor(Bn,Bp,131,20); // C3, ish
        chirpMotor(An,Ap,165,20); // E3, ish
        chirpMotor(Bn,Bp,131,20); // C3, ish
        chirpMotor(An,Ap,165,20); // E3, ish
      }
    }
  }
  
  
  #ifdef USE_LED
  setStatusScreen();
  #endif

  updateCount ++;
  updateCount %= 1000;
  
  delay(20);
}









