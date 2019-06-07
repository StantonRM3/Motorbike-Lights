/*
  LCD Display Shield with Buttons Demo
  lcd-button-demo.ino
  Use Display Shield with Analog Buttons
  DroneBot Workshop 2018
  https://dronebotworkshop.com
*/

// Include LiquidCrystal library
#include <LiquidCrystal.h>
#include <TimerOne.h>
#include <Adafruit_GFX.h>
#include <Adafruit_NeoMatrix.h>
#include <Adafruit_NeoPixel.h>
#ifndef PSTR
#define PSTR // Make Arduino Due happy
#endif

#define MATRIX_DATA_PIN A5

// MATRIX DECLARATION:
// Parameter 1 = width of NeoPixel matrix
// Parameter 2 = height of matrix
// Parameter 3 = pin number (most are valid)
// Parameter 4 = matrix layout flags, add together as needed:
//   NEO_MATRIX_TOP, NEO_MATRIX_BOTTOM, NEO_MATRIX_LEFT, NEO_MATRIX_RIGHT:
//     Position of the FIRST LED in the matrix; pick two, e.g.
//     NEO_MATRIX_TOP + NEO_MATRIX_LEFT for the top-left corner.
//   NEO_MATRIX_ROWS, NEO_MATRIX_COLUMNS: LEDs are arranged in horizontal
//     rows or in vertical columns, respectively; pick one or the other.
//   NEO_MATRIX_PROGRESSIVE, NEO_MATRIX_ZIGZAG: all rows/columns proceed
//     in the same order, or alternate lines reverse direction; pick one.
//   See example below for these values in action.
// Parameter 5 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)


// Example for NeoPixel Shield.  In this application we'd like to use it
// as a 5x8 tall matrix, with the USB port positioned at the top of the
// Arduino.  When held that way, the first pixel is at the top right, and
// lines are arranged in columns, progressive order.  The shield uses
// 800 KHz (v2) pixels that expect GRB color data.
Adafruit_NeoMatrix matrix = Adafruit_NeoMatrix(24, 8, MATRIX_DATA_PIN,
                            NEO_MATRIX_BOTTOM     + NEO_MATRIX_LEFT +
                            NEO_MATRIX_COLUMNS + NEO_MATRIX_PROGRESSIVE,
                            NEO_GRB            + NEO_KHZ800);

// Setup lcd object with display pinouts
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

char animationCycleCounter = 0;
#define animationCycles 16
#define aminationSchemeCount 4
char animationScheme = 0;

byte breakLight[8] = {
  B00000000,
  B00011000,
  B00111100,
  B01111110,
  B01111110,
  B00111100,
  B00011000,
  B00000000
};

byte indPattern[aminationSchemeCount][animationCycles] =
{{
    B00000001,
    B00000011,
    B00000111,
    B00001111,
    B00011111,
    B00111111,
    B01111111,
    B11111111,
    B11111110,
    B11111100,
    B11111000,
    B11110000,
    B11100000,
    B11000000,
    B10000000,
    B00000000
  },
  {
    B00000001,
    B00000011,
    B00000111,
    B00001111,
    B00011110,
    B00111100,
    B01111000,
    B11110000,
    B11100001,
    B11000011,
    B10000111,
    B00001111,
    B00011110,
    B00111100,
    B01111000,
    B11110000
  },
  {
    B00000000,
    B00000000,
    B11111111,
    B11111111,
    B00000000,
    B00000000,
    B11111111,
    B11111111,
    B00000000,
    B00000000,
    B11111111,
    B11111111,
    B00000000,
    B00000000,
    B11111111,
    B11111111
  },
  {
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B11111111,
    B11111111,
    B11111111,
    B11111111,
    B00000000,
    B00000000,
    B00000000,
    B00000000,
    B11111111,
    B11111111,
    B11111111,
    B11111111
  }
};

// Define button constants
#define btnRIGHT  0
#define btnUP     1
#define btnDOWN   2
#define btnLEFT   3
#define btnSELECT 4
#define btnNONE   5

// Define variable to hold current button constant value
int lcd_key = 0;
#define lcd_numRows   2
#define lcd_numChars 16

char lcdDisplayStr[lcd_numRows][lcd_numChars];    // To hold values so they can be automatically scrolled upwards...

// Function to read the buttons
// Returns button constant value
int read_LCD_buttons() {
  // Define variable to hold button analog value
  int adc_key_in = 0;
  static int lastButtonPressed = btnNONE;
  int currentButtonPressed = btnNONE;

  // read the value from the sensor
  adc_key_in = analogRead(0);

  // Approx button values are 0, 144, 329, 504, 741
  // Add approx 50 to those values and check to see if we are close
  if (adc_key_in > 1000) currentButtonPressed = btnNONE; // No button is pressed
  else if (adc_key_in < 50)   currentButtonPressed = btnRIGHT; // Right pressed
  else if (adc_key_in < 195)  currentButtonPressed = btnUP;  // Up presed
  else if (adc_key_in < 380)  currentButtonPressed = btnDOWN;  // Down pressed
  else if (adc_key_in < 555)  currentButtonPressed = btnLEFT;  // Left pressed
  else if (adc_key_in < 790)  currentButtonPressed = btnSELECT; // Select pressed

  // If nothing pressed return accordingly...
  if ( currentButtonPressed == btnNONE) {
    lastButtonPressed = btnNONE;
    return btnNONE;
  }

  // MUST DEBOUNCE THE BUTTONS....
  // Otherwise, if something has been pressed, check its not what was pressed before
  // Ignore if its the same button pressed...
  // If its not the same button, return it and remember the last button...
  if ( currentButtonPressed == lastButtonPressed ) {
    return btnNONE;
  }
  else {
    lastButtonPressed = currentButtonPressed;
    return currentButtonPressed;
  }
}

void displayMessage(char *pDisplayStr) {
  // Copy rows down so newest message is at the top...
  for (int i = (lcd_numRows - 1); i > 0; i--) {
    sprintf(lcdDisplayStr[i], "%-15s", lcdDisplayStr[i - 1]);
  }

  sprintf(lcdDisplayStr[0], "%-15s", pDisplayStr);

  lcd.clear();

  // Display new line at top...
  for (int i = 0; i < lcd_numRows; i++) {
    lcd.setCursor(0, i);
    lcd.print(lcdDisplayStr[i]);
  }
}

#define BUTTON_POLL_FREQ 200000     // Check for button presses every xxxx micro seconds (100000 = 0.1s)

void setup() {
  // Setup the LCD display
  lcd.begin(lcd_numChars, lcd_numRows);

  // Setup interupt timer...
  Timer1.initialize(BUTTON_POLL_FREQ);
  Timer1.attachInterrupt(buttonInterrupt);

  // Setup the matrix display
  matrix.begin();
  matrix.setBrightness(40);

  Serial.begin(9600);
  displayMessage("Motorbike Light");
}

#define LS_CURR   0   // Used to index the latchState arrays below...
#define LS_PREV   1

bool latchState_IndLeft[2] = {false, false};
bool latchState_IndRight[2] = {false, false};
bool latchState_Hazard[2] = {false, false};
bool latchState_Breaking[2] = {false, true};    // Needed to initially draw the break light in off state...
bool latchState_BreakingHard[2] = {false, false};

unsigned long timer_IndicatorsOn = 0;
unsigned long timer_IndicatorsOff = 1000;
unsigned long prevTime = 0;

#define CYCLE_TIME_MS 50   // Time to wait between program iterations.  This value will be used as multiplier when we need to flash items...

void loop() {
  unsigned long currTime = millis();

  // We dont need to worry about buttons here as these are handled by the Timer1 interrupt.
  // In here we simply need to manage the peripherals and lights (i.e. indicators, break lights, etc)...
  if ((currTime - prevTime) >= CYCLE_TIME_MS)  {
    // Check and set the light status...
    manageLights();

    if (latchState_IndLeft[LS_CURR] == true)
      indLeft(latchState_IndLeft[LS_CURR]);

    if (latchState_IndRight[LS_CURR] == true)
      indRight(latchState_IndRight[LS_CURR]);

    if (latchState_Hazard[LS_CURR] == true) {
      indLeft(latchState_Hazard[LS_CURR]);
      indRight(latchState_Hazard[LS_CURR]);
    }

    if (++animationCycleCounter > animationCycles)
      animationCycleCounter = 0;

    // The indicators are on, check how long and turn them off after defined time...
    if (timer_IndicatorsOn > 0) {
      if (( millis() - timer_IndicatorsOn) > timer_IndicatorsOff) {
        latchState_IndLeft[LS_CURR] = false;
        latchState_IndRight[LS_CURR] = false;
        timer_IndicatorsOn = 0;
      }
    }

    prevTime = currTime;
  }
}

//
// LIGHT DISPLAY FUNCTIONS
//
// THESE FUNCTIONS MANAGE LIGHT STATES BASED ON THE LOGIC STATES SET BY THE BUTTONS.
// ALL USE THE APPROPRIATE latchState_XXXXXX[LS_CURR] VALUES
//

// This function contains all the code to manage which lights should be turned on or off.
// It does not directly manage the illumination, flashing, turning off fucntionality which
// is contained within specific functions...
void manageLights() {
  // Check if the state has changed, and only do something if it has...
  if (latchState_IndLeft[LS_CURR] != latchState_IndLeft[LS_PREV] ) {
    animationCycleCounter = 0;
    indLeft(latchState_IndLeft[LS_CURR]);
    latchState_IndLeft[LS_PREV] = latchState_IndLeft[LS_CURR];
  }
  if (latchState_IndRight[LS_CURR] != latchState_IndRight[LS_PREV]) {
    animationCycleCounter = 0;
    indRight(latchState_IndRight[LS_CURR]);
    latchState_IndRight[LS_PREV] = latchState_IndRight[LS_CURR];
  }
  if (latchState_Hazard[LS_CURR] != latchState_Hazard[LS_PREV]) {
    animationCycleCounter = 0;
    hazard(latchState_Hazard[LS_CURR]);
    latchState_Hazard[LS_PREV] = latchState_Hazard[LS_CURR];

    // Temporary control for changing the animation scheme...
    if (latchState_Hazard[LS_CURR] == false) {
      if (++animationScheme >= aminationSchemeCount)
        animationScheme = 0;
    }

  }
  if (latchState_Breaking[LS_CURR] != latchState_Breaking[LS_PREV] ) {
    breaking(latchState_Breaking[LS_CURR]);
    latchState_Breaking[LS_PREV] = latchState_Breaking[LS_CURR];
  }
  if (latchState_BreakingHard[LS_CURR] != latchState_BreakingHard[LS_PREV]) {
    breakingHard(latchState_BreakingHard[LS_CURR]);
    latchState_BreakingHard[LS_PREV] = latchState_BreakingHard[LS_CURR];
  }
}

void indLeft(bool pOnOff) {
  if (pOnOff == true)
    timer_IndicatorsOn = millis();
  else
    timer_IndicatorsOn = 0;

  draw_leftInd(pOnOff);
}

void indRight(bool pOnOff) {
  if (pOnOff == true)
    timer_IndicatorsOn = millis();
  else
    timer_IndicatorsOn = 0;

  draw_rightInd(pOnOff);
}

void hazard(bool pOnOff) {
  draw_leftInd(pOnOff);
  draw_rightInd(pOnOff);
}

void breaking(bool pOnOff) {
  int redIntensity = 64;
  int brightness = 40;

  //  {
  //    char dbgMsg[64];
  //    sprintf(dbgMsg, "Break lights turned %s", pOnOff ? "ON" : "OFF");
  //    Serial.println(dbgMsg);
  //  }

  if (pOnOff == true) {
    redIntensity = 128;
    brightness = 128;
  }

  draw_breakLight(redIntensity, brightness);
}

void breakingHard(bool pOnOff) {
  int redIntensity = 128;
  int brightness = 128;

  //  {
  //    char dbgMsg[64];
  //    sprintf(dbgMsg, "Hard Breaking: Break lights %s", pOnOff ? "FLASHING" : "NORMAL");
  //    Serial.println(dbgMsg);
  //  }

  if (pOnOff == true) {
    redIntensity = 255;
    brightness = 255;
  }

  draw_breakLight(redIntensity, brightness);
}

void draw_breakLight(int pRedIntensity, int pBrightness) {
  uint16_t lightCol = matrix.Color(pRedIntensity, 0, 0);
  uint16_t offCol = matrix.Color(0, 0, 0);
  int centerCol = (matrix.width() / 2) - 4; // Finds the left most column for the break light...

  matrix.setBrightness(pBrightness);

  for (int i = 0; i < 8; i++) {
    for (int j = 0; j < 8; j++) {
      if ((breakLight[i] >> j) & 1)
        matrix.drawPixel((centerCol + j), i, lightCol);
      else
        matrix.drawPixel((centerCol + j), i, offCol);
    }
  }

  matrix.show();
}

#define NUM_INDROWS 2

void draw_leftInd(bool pOnOff) {
  uint16_t lightCol = matrix.Color(255, 255, 0);
  uint16_t offCol = matrix.Color(0, 0, 0);
  int rightCol = (matrix.width() / 2) - 5;                // Finds the left most column for the right indicator light...
  int topRow = (matrix.height() / 2) - (NUM_INDROWS / 2); // For a NUM_INDROWS row display...

  if (pOnOff == true) {
    for (int i = 0; i < NUM_INDROWS; i++) {
      for (int j = 0; j < animationCycles; j++) {
        if ((indPattern[animationScheme][animationCycleCounter] >> j) & 1)
          matrix.drawPixel((rightCol - j), (topRow + i), lightCol);
        else
          matrix.drawPixel((rightCol - j), (topRow + i), offCol);
      }
    }
  }
  else {
    for (int i = 0; i < NUM_INDROWS; i++) {
      for (int j = 0; j < animationCycles; j++) {
        matrix.drawPixel((rightCol - j), (topRow + i), offCol);
      }
    }
  }
  matrix.show();
}

void draw_rightInd(bool pOnOff) {
  uint16_t lightCol = matrix.Color(255, 255, 0);
  uint16_t offCol = matrix.Color(0, 0, 0);
  int leftCol = (matrix.width() / 2) + 4;       // Finds the left most column for the right indicator light...
  int topRow = (matrix.height() / 2) - (NUM_INDROWS / 2); // For a 2 row display...

  if (pOnOff == true) {
    for (int i = 0; i < NUM_INDROWS; i++) {
      for (int j = 0; j < animationCycles; j++) {
        if ((indPattern[animationScheme][animationCycleCounter] >> j) & 1)
          matrix.drawPixel((leftCol + j), (topRow + i), lightCol);
        else
          matrix.drawPixel((leftCol + j), (topRow + i), offCol);
      }
    }
  }
  else {
    for (int i = 0; i < NUM_INDROWS; i++) {
      for (int j = 0; j < animationCycles; j++) {
        matrix.drawPixel((leftCol + j), (topRow + i), offCol);
      }
    }
  }
  matrix.show();
}

//
// BUTTON CONTROL FUNCTIONS
//
// THESE FUNCTIONS MANAGE BUTTON DETECTION AND LOGIC STATES.
// ALL SET THE APPROPRIATE latchState_XXXXXX[LS_CURR] VALUES
//
// THE DISPLAY FUNCTIONS ABOVE WILL USE THESE LOGIC STATES TO DETERMINE WHAT TO DISPLAY
//
void buttonInterrupt() {
  checkForControls();
}

void checkForControls() {
  // Call the read buttons function
  lcd_key = read_LCD_buttons();

  if (lcd_key != btnNONE) {
    // Print button value on second line
    switch (lcd_key)
    {
      case btnRIGHT: {
          // Ignore if Hazards on...
          if ( latchState_Hazard[LS_CURR] == false)   indRight_pressed();
          break;
        }
      case btnLEFT: {
          if ( latchState_Hazard[LS_CURR] == false)   indLeft_pressed();
          break;
        }
      case btnUP: {
          if ( latchState_Breaking[LS_CURR] == true)  breakingHard_pressed();
          break;
        }
      case btnDOWN: {
          breaking_pressed();
          break;
        }
      case btnSELECT: {
          hazard_pressed();
          break;
        }
      case btnNONE:
        break;
    }
  }
}

void indLeft_pressed() {
  char displayStr[lcd_numChars];

  latchState_IndLeft[LS_CURR] = !latchState_IndLeft[LS_CURR];

  sprintf(displayStr, "Ind:Left:%s", latchState_IndLeft[LS_CURR] == true ? "On" : "Off");
  displayMessage(displayStr);

  // If the left indicator is now ON, must turn off the right indicator.  Simulate right button press...
  if ((latchState_IndLeft[LS_CURR] == true) && (latchState_IndRight[LS_CURR] == true)) {
    indRight_pressed();
  }
}

void indRight_pressed() {
  char displayStr[lcd_numChars];

  latchState_IndRight[LS_CURR] = !latchState_IndRight[LS_CURR];

  sprintf(displayStr, "Ind:Right:%s", latchState_IndRight[LS_CURR] == true ? "On" : "Off");
  displayMessage(displayStr);

  // If the right indicator is now ON, must turn off the left indicator.  Simulate left button press...
  if ((latchState_IndLeft[LS_CURR] == true) && (latchState_IndRight[LS_CURR] == true)) {
    indLeft_pressed();
  }
}

void breaking_pressed() {
  char displayStr[lcd_numChars];

  latchState_Breaking[LS_CURR] = !latchState_Breaking[LS_CURR];

  sprintf(displayStr, "Breaking:%s", latchState_Breaking[LS_CURR] == true ? "On" : "Off");
  displayMessage(displayStr);

  // If not breaking anymore, reset the hard breaking latchs...
  if (latchState_Breaking[LS_CURR] == false) {
    latchState_BreakingHard[LS_CURR] = false;
    latchState_BreakingHard[LS_PREV] = false;   // Needed to prevent the HardBreaking latch overiding the normal break status...
  }
}

void breakingHard_pressed() {
  char displayStr[lcd_numChars];

  latchState_BreakingHard[LS_CURR] = !latchState_BreakingHard[LS_CURR];

  sprintf(displayStr, "BreakHard:%s", latchState_BreakingHard[LS_CURR] == true ? "True" : "False");
  displayMessage(displayStr);
}

void hazard_pressed() {
  char displayStr[lcd_numChars];

  latchState_Hazard[LS_CURR] = !latchState_Hazard[LS_CURR];
  latchState_IndLeft[LS_CURR] = false;
  latchState_IndRight[LS_CURR] = false;

  sprintf(displayStr, "Hazard:%s", latchState_Hazard[LS_CURR] == true ? "On" : "Off");
  displayMessage(displayStr);
}
