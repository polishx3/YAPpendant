#include <Modbus.h>
#include <ModbusSerial.h>
#include <Encoder.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleKeypad.h>

#define VERSION      "alpha 1.8"
#define SERIAL_RX_BUFFER_SIZE 128
#define SERIAL_TX_BUFFER_SIZE 128

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display

const int buttonPin = 2;    // the number of the pushbutton pin
const int ledPin = 13;      // the number of the LED pin

//ModbusSerial object
ModbusSerial mb;

uint16_t pPos;
uint16_t tCount;
unsigned long lastTime;
unsigned long lastTest;
unsigned long debugTest;

bool prevState1 = false;

#define baurate 19200  //
#define slaveid 1       // Set the Slave ID (1-247)

Encoder MPG1(3, 2);
int MPG1oldPosition = 0;
int MPG1displayedPosition = 0;
int MPG1memory = 0;

byte sMode;
float lastDRODecimal[3];

//byte feedRateOV = 100;
int fOVRmemory = 400;

//byte feedRateOV = 100;
int sOVRmemory = 400;

#define knob1 A1
#define knob2 A2
byte r1old, r1new, r2old, r2new, r1tick, r2tick;

byte enold, ennew;

const byte nb_rows = 2;                         // two rows
const byte nb_cols = 4;                         // four columns
char key_chars[nb_rows][nb_cols] = {            // The symbols of the keys
  {'1', '2', '3', '4'},
  {'5', '6', '7', '8'}
};
byte rowPins[nb_rows] = {9, 8};           // The pins where the rows are connected
byte colPins[nb_cols] = {4, 5, 6, 7};           // The pins where the columns are connected

//initialize an instance of class NewKeypad
SimpleKeypad kp1((char*)key_chars, rowPins, colPins, nb_rows, nb_cols);   // New keypad called kp1
char oldkey;

void setup() {
  // Start cmmunication as a slave Modbus
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
  // put your setup code here, to run once:
  // Config Modbus Serial (port, speed, byte format)
  mb.config(&Serial, baurate, SERIAL_8N2);
  // Set the Slave ID (1-247)
  mb.setSlaveId(slaveid);

  //Register type     Use as          Access      Library methods
  //Coil              Digital Output  Read/Write  addCoil(), Coil()
  //Holding Register  Analog Output   Read/Write  addHreg(), Hreg()
  //Input Status      Digital Input   Read Only   addIsts(), Ists()
  //Input Register    Analog Input    Read Only   addIreg(), Ireg()

  //Input Register
  mb.addIreg(0);
  mb.Ireg(0, 0);
  mb.addIreg(1);
  mb.Ireg(1, 0);
  mb.addIreg(2);
  mb.Ireg(2, 0);
  mb.addIreg(3);
  mb.Ireg(3, 0);
  mb.addIreg(4);
  mb.Ireg(4, 92);
  mb.addIreg(5);
  mb.Ireg(5, 0);

  //Input Status or Discrete Input
  mb.addIsts(0);
  mb.Ists(0, true); //MPG enable
  mb.addIsts(1);
  mb.Ists(1, true);
  mb.addIsts(2);
  mb.Ists(2, true);

  //Holding Register
  mb.addHreg(0);
  mb.Hreg(0, 100);
  mb.addHreg(1);
  mb.addHreg(2);
  mb.Hreg(2, 100);
  mb.addHreg(3);
  mb.addHreg(4);
  mb.Hreg(4, 100);
  mb.addHreg(5);
  mb.addHreg(6);
  mb.addHreg(7);
  mb.addHreg(8);
  mb.addHreg(9);
  mb.addHreg(10);

  //Coil
  mb.addCoil(0);
  mb.addCoil(1);

  lastTime = millis();
  lastTest = millis();
  debugTest = millis();

  lcd.begin();                      // initialize the lcd
  // Print a message to the LCD.
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Hello, Mach3!");
  lcd.setCursor(4, 2);
  lcd.print(VERSION);
  delay(10000);
  lcd.clear();
}

void loop() {
  int MPG1newPosition;



  MPG1newPosition = MPG1.read();
  if (MPG1newPosition != MPG1oldPosition) {
    MPG1displayedPosition = MPG1newPosition / 4;
    switch (ennew) {
      case 0:
      case 1:
      case 2:
        mb.Ireg(1,  MPG1displayedPosition);
        if (MPG1newPosition > 32760 || MPG1newPosition < -32760) {
          MPG1newPosition = MPG1.readAndReset(); //first one read; then resets
          MPG1newPosition = MPG1.readAndReset();
        }
        break;
      case 3:
        mb.Ireg(4,  MPG1displayedPosition);
        if (MPG1newPosition > 1200) {
          MPG1.write(1200);
          MPG1newPosition = 1200;
        }
        if (MPG1newPosition < 40) {
          MPG1.write(40);
          MPG1newPosition = 40;
        }
        break;
      case 4:
        mb.Ireg(5,  MPG1displayedPosition);
        if (MPG1newPosition > 1200) {
          MPG1.write(1200);
          MPG1newPosition = 1200;
        }
        if (MPG1newPosition < 40) {
          MPG1.write(40);
          MPG1newPosition = 40;
        }
        break;
    }

    MPG1oldPosition = MPG1newPosition;
  }








  switch (sMode) {
    case 0:
      drawDROs(sMode);
      sMode += 1;
      break;
    case 1:
      drawDROs(sMode);
      sMode += 1;
      break;
    case 2:
      drawDROs(sMode);
      sMode += 1;
      break;
    case 3:
      doKnobs();
      sMode += 1;
      break;
    case 4:
      doExtra();
      sMode += 1;
      break;
    case 5:
      doPad();
      sMode += 1;
      break;
    case 6:
      drawXDROs();
      sMode += 1;
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      sMode = 0;
      break;
  }






  //prevState1 = digitalRead(buttonPin);
  //digitalWrite(ledPin, LOW);
  mb.Ireg(0, (int16_t)((millis() - lastTime) * 20));

  mb.task();
  yield();
  lastTime = millis();

  if (ennew != enold) {
    switch (enold) {
      case 0:
      case 1:
      case 2:
        MPG1memory = MPG1newPosition;
        break;
      case 3:
        fOVRmemory = MPG1newPosition;
        break;
      case 4:
        sOVRmemory = MPG1newPosition;
        break;
    }
    switch (ennew) {
      case 0:
      case 1:
      case 2:
        MPG1.write( MPG1memory);
        break;
      case 3:
        MPG1.write( fOVRmemory);
        break;
      case 4:
        MPG1.write( sOVRmemory);
        break;
    }
    enold = ennew;
  }
}

String formPercent(int pp)
{
  String r;
  r = "";
  if (pp < 100) {
    r += " ";
  }
  if (pp < 10) {
    r += " ";

  }
  r += String(pp);
  return r;
}

void drawXDROs()
{
  lcd.setCursor(14, 0);
  if (r1new == 3) {
    lcd.print("F:*");
  }
  else {
    lcd.print("F: ");
  }

  lcd.print(formPercent(mb.Hreg(7)));

  lcd.setCursor(14, 2);
  if (r1new == 4) {
    lcd.print("S:*");
  }
  else {
    lcd.print("S: ");
  }

  lcd.print(formPercent(mb.Hreg(8)));


  lcd.setCursor(9, 3);
  if (mb.Coil(0) == 1) {
    lcd.print("Prb");
  }
  else {
    lcd.print("   ");
  }
}

void doPad()
{
  char key = kp1.scan();                      // scan
  byte c = 0;
  if (key) {

    if (key == oldkey) { //for debounce
      c = (byte)key - 48;
    }
    oldkey = key;
  }
  else {
    oldkey = "";
  }

  if (c == 5) {
    c = 10 + r1new;
  }


  mb.Ireg(3, (int16_t)c);
}

void doExtra()
{
  float sStep;
  char stg[5];
  lcd.setCursor(0, 3);
  if (r2new > 1) {
    lcd.print("S");
  }
  else {
    lcd.print("V");
  }

  sStep = mb.Hreg(6) / 1000.0;
  lcd.setCursor(2, 3);
  lcd.print(sStep, 3);


  //lcd.setCursor(8, 3);
  //lcd.print("Fr:");
  //lcd.setCursor(16, 2);
  //lcd.print("    ");
  lcd.setCursor(16, 1);
  sprintf(stg, "%4d", mb.Hreg(9));
  lcd.print(stg);

  
  lcd.setCursor(15, 3);
  sprintf(stg, "%5d", mb.Hreg(10));
  lcd.print(stg);
}

void doKnobs()
{

  r1new = (byte)((analogRead (knob1) + 100) / 200);
  if (r1new != r1old) {
    r1tick += 1;
    if (r1tick > 3) {
      r1old = r1new;
    }
    else {
      r1new = r1old;
    }
  }
  else {
    r1tick = 0;
    r1new = r1old;
  }



  r2new = (byte)((analogRead (knob2) + 100) / 200);
  if (r2new != r2old) {
    r2tick += 1;
    if (r2tick > 3) {
      r2old = r2new;
    }
    else {
      r2new = r2old;
    }
  }
  else {
    r2tick = 0;
    r2new = r2old;
  }

  ennew = r1new;

  mb.Ireg(2,  (int16_t)(r1new + (r2new * 10)));
}

// Call with number to display (FLOAT), decimal point column,
// line number (0 or 1),number of columns after DP (start with 4),
// number of columns after decimal point (try 3)

void lcdNumFlt (float num, byte dpcol, byte line, byte bdp, byte adp)
{
  long  tmp;
  byte offset = 1;

  tmp = num;
  while (abs(tmp) >= 10) {
    tmp /= 10;
    offset ++;
  }
  if (num < 0) offset ++;
  lcd.setCursor(dpcol - bdp, line);
  for (byte i = 0; i < bdp + adp + 1; i ++)
    lcd.print(" ");
  lcd.setCursor(dpcol - offset, line);
  lcd.print(num, adp);
}



void drawDROValue(byte axisID, byte hreg1, byte hreg2) {
  int16_t droPrefix = mb.Hreg(hreg1);
  int16_t droPostfix = mb.Hreg(hreg2);
  float droDecimal = droPrefix + (droPostfix / 10000.0);

  if (droDecimal != lastDRODecimal[axisID] ) {
    //lcdNumFlt(droPrefix, droPostfix, 7, axisID);
    lcdNumFlt(droDecimal, 7, axisID, 4, 4);
  }
  lastDRODecimal[axisID] = droDecimal;
}

void updateDROs(byte axisID) {
  byte axisReg;
  axisReg = axisID * 2;
  drawDROValue(axisID, axisReg, axisReg + 1);
  //drawDROValue(0, 0, 1);
  //drawDROValue(1, 2, 3);
  //drawDROValue(2, 4, 5);
}

void drawDROs(byte axisID) {
  switch (axisID) {
    case 0:
      lcd.setCursor(0, 0);
      if (r1new == 0) {
        lcd.print("X:*");
      }
      else {
        lcd.print("X: ");
      }

      break;
    case 1:
      lcd.setCursor(0, 1);
      if (r1new == 1) {
        lcd.print("Y:*");
      }
      else {
        lcd.print("Y: ");
      }
      break;
    case 2:
      lcd.setCursor(0, 2);
      if (r1new == 2) {
        lcd.print("Z:*");
      }
      else {
        lcd.print("Z: ");
      }
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }

  updateDROs(axisID);
}
