#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include "Pitches.h"

// bpSPD Button -----------------------------------------------------------------------------------
#define bpSPD 12

// OLED display TWI address -----------------------------------------------------------------------
#define OLED_ADDR   0x3C

// Reset pin not used on 4-pin OLED module --------------------------------------------------------
Adafruit_SSD1306 display(-1);  // -1 = no reset pin

// 128 x 32 pixel display -------------------------------------------------------------------------
#if (SSD1306_LCDHEIGHT != 64)
#error("Height incorrect, please fix Adafruit_SSD1306.h!");
#endif

// Voltage measurement ----------------------------------------------------------------------------
#define NUM_SAMPLES 10
#define REF_VOLTAGE 4.175

float R = 0.82;               // Initialisation of variables for resistance, and for maximum and
float Vmax = 2.70;            // minimum voltage
float Vmin = 0.10;
float I = 0.00;               // Initialisation of variable for current
float V = 0.00;               // Initialisation of variable for voltage on capacitors
float Vr = 0.00;              // Initialisation of variable for voltage on discharge resistor
float Vperc = 0;              // Declaration of variable percentage of Vmax
float C = 0;                  // Declaration of variable for capacitance
float t = 0;                  // Initialisation of variable for time

byte cycle = 0;               // Initialise counter for current cycle, number of cycles
byte cycles = 3;
byte cyclesMax = 9;           // Initialisation of variable determining maximum number of cycles

// Relays ---------------------------------------------------------------------------------------
byte relayCS = 3;              // Define pin connected to charging relay (CS == "Capacitor - Supply")
byte relayCD = 2;              // Define pin connected to discharging relay (CD == "Capacitor - Discharge")
// Button bpT -----------------------------------------------------------------------------------
int bpTPin = 11;              // The number of the input pin for screen toggle
boolean bpTState = HIGH;      // The current state of the output pin
int bpTReading;               // The current reading from the input pin
boolean bpTPrevious = LOW;    // The previous reading from the input pin
long bpTTime = 0;             // The last time the output pin was toggled
long bpTDebounce = 100;       // The debounce time, increase if the output flickers
int bpTVar = 0;               // Variable recording state of screen Toggle button
// Button bpSPD ---------------------------------------------------------------------------------
int bpSPDVar = 1;             // Variable recording state of screen bpSPD button
// Button timing variables
int debounce = 20;            // ms debounce period to prevent flickering when pressing or releasing the button
int DCgap = 250;              // max ms between clicks for a double click event
int holdTime = 1000;          // ms hold period: how long to wait for press+hold event
int longHoldTime = 3000;      // ms long hold period: how long to wait for press+hold event
// Other button variables
boolean buttonVal = HIGH;     // value read from button
boolean buttonLast = HIGH;    // buffered value of the button's previous state
boolean DCwaiting = false;    // whether we're waiting for a double click (down)
boolean DConUp = false;       // whether to register a double click on next release, or whether to wait and click
boolean singleOK = true;      // whether it's OK to do a single click
long downTime = -1;           // time the button was pressed down
long upTime = -1;             // time the button was released
boolean ignoreUp = false;     // whether to ignore the button release because the click+hold was triggered
boolean waitForUp = false;          // when held, whether to wait for the up event
boolean holdEventPast = false;      // whether or not the hold event happened already
boolean longHoldEventPast = false;  // whether or not the long hold event happened already
// Caps -----------------------------------------------------------------------------------------
byte capVoltage = A7;
byte capCurrent = A6;
// LEDs -----------------------------------------------------------------------------------------
byte LEDCharge = A1;
byte LEDDischarge = A0;
// Graph vars -------------------------------------------------------------------------------------
double x, y;
double ox, oy ;
// Koef -------------------------------------------------------------------------------------------
double koef = 1.1376;
// Speaker ----------------------------------------------------------------------------------------
byte speaker = 10;

//-------------------------------------------------------------------------------------------------
//                                          SETUP/LOOP
//-------------------------------------------------------------------------------------------------


void setup() {
  analogReference(EXTERNAL);
  Serial.begin(9600);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  display.display();
  delay(2000);
  display.clearDisplay();  
  
  // Relays ------------------------------------------------------------------------------------
  pinMode(relayCS, OUTPUT);
  digitalWrite(relayCS, LOW);
  pinMode(relayCD, OUTPUT);
  digitalWrite(relayCD, LOW);
  // Button bpT---------------------------------------------------------------------------------
  pinMode(bpTPin, INPUT);
  // Button bpSPD-------------------------------------------------------------------------------
  pinMode(bpSPD, INPUT);
  // LEDs---------------------------------------------------------------------------------------
  pinMode(LEDCharge, OUTPUT);
  digitalWrite(LEDCharge, LOW);
  pinMode(LEDDischarge, OUTPUT);
  digitalWrite(LEDDischarge, LOW);

  testdrawline();
  display.display();
  display.clearDisplay();

  playMelody(1);
}

void loop() {
  
  checkSPD();
  
  switch (bpSPDVar) {
    case 1:
      cycle = 0;
      idle();      
      break;
    case 2:
      advanceCyclesNumber();
      break;
    case 3:    
    if (cycle < cycles) {
        cycle++;
        discharge();
        charge();
      } else {
        discharge();
        playMelody(3);
        bpSPDVar = 1;
        cycle = 0;
        Serial.println("Measurement complete!");     
      }
      break;
  }
}

//-------------------------------------------------------------------------------------------------
//                                        FUNCTIONS
//-------------------------------------------------------------------------------------------------

void discharge() {                      // Function for discharging   
  unsigned long startTime = millis();         // Start measuring time in milliseconds
  unsigned long currentTime = 0;              // Mark down time in milliseconds again
  unsigned long tPause = 0;
  Serial.println("Discharging...");  
  V = measureVoltage();
  display.clearDisplay();                      // Calculate percentage that current voltage is to value Vmax
  while (V > Vmin) {                           // While measured voltage is lower than Vmax...
    checkSPD();                                // Check state of SPD button
      currentTime = millis();
      if (bpSPDVar == 1) {
        digitalWrite(relayCD, LOW);            // Disconnect capacitors discharging resistor
        digitalWrite(relayCS, LOW);            // Connect capacitors to supply
        digitalWrite(LEDDischarge, LOW);
        tPause = t - (currentTime - startTime);
      } else if (bpSPDVar == 2) { 
        advanceCyclesNumber();
      } else if (bpSPDVar == 4) {
        bpSPDVar = 1; 
        break;
      } else {
        digitalWrite(LEDDischarge, HIGH);       // Turn on charge LED
        digitalWrite(relayCS, LOW);             // Disconnect capacitors discharging resistor
        digitalWrite(relayCD, HIGH);            // Connect capacitors to supply
        V = measureVoltage();
        Vperc = (V / Vmax) * 100;
        I = measureCurrent();                   // Read voltage on discharge resistor, taking voltage divider into consideration
                                                // Calculate capacitor discharge current (converted to mA)        
        t = currentTime - startTime + tPause;   // Calculate time passed between two marks
        if (V > ((0.368 * Vmax)+0.002)) {       // While voltage is less than 63.2% of Vmin,
          C = -1;                               // set C to -1 (see displayOut())
        } else if (V > ((0.368 * Vmax)-0.002) && V < ((0.368 * Vmax)+0.002)) {         // When voltage reaches 63.2% of Vmax,
          C = (t / 1000) / R;                   // calculate capacitance (while converting time from ms to s)
        }
        displayOut();                           // Keep updating display
        Serial.print((int)(V * 1000));          // Send values for V (converted to mV), I (in mA), C (in F) and t (converted to s) over Serial
        Serial.print(",");
        Serial.print((int)I);
        Serial.print(",");        
        Serial.println((int)(t / 1000));
        delay(3);
      }
  }  
  digitalWrite(relayCD, LOW);                   // Disconnect capacitors from supply
  digitalWrite(LEDDischarge, LOW);              // Turn off charge LED
  t = 0;                                        // Reset time to 0
  Serial.print("Discharge cycle ");
  Serial.print(cycle);
  Serial.print(" done! C = ");
  Serial.println(C);
  playMelody(2);
  //delay(3000); 
}

void charge() {                         // Function for charging  
  unsigned long startTime = millis();         // Start measuring time in milliseconds
  unsigned long currentTime = 0;              // Mark down time in milliseconds again
  unsigned long tPause = 0;
  Serial.println("Charging...");
  V = measureVoltage();
  display.clearDisplay();                     // Calculate percentage that current voltage is to value Vmax
  while (V < Vmax) {                          // While measured voltage is lower than Vmax...
    checkSPD();                               // Check state of SPD button
      currentTime = millis();
      if (bpSPDVar == 1) {
        digitalWrite(relayCD, LOW);           // Disconnect capacitors discharging resistor
        digitalWrite(relayCS, LOW);           // Connect capacitors to supply
        digitalWrite(LEDCharge, LOW);
        tPause = t - (currentTime - startTime);
      } else if (bpSPDVar == 2) { 
        advanceCyclesNumber();
      } else if (bpSPDVar == 4) {
        bpSPDVar = 1; 
        break;
      } else {
        digitalWrite(LEDCharge, HIGH);          // Turn on charge LED
        digitalWrite(relayCD, LOW);             // Disconnect capacitors discharging resistor
        digitalWrite(relayCS, HIGH);            // Connect capacitors to supply
        V = measureVoltage();
        Vperc = (V / Vmax) * 100;
        I = measureCurrent();                   // Read voltage on discharge resistor, taking voltage divider into consideration
                                                // Calculate capacitor discharge current (converted to mA)        
        t = currentTime - startTime + tPause;   // Calculate time passed between two marks
        if (V < ((0.632 * Vmax)-0.002)) {       // While voltage is less than 63.2% of Vmin,
          C = -1;                               // set C to -1 (see displayOut())
        } else if (V > ((0.632 * Vmax)-0.002) && V < ((0.632 * Vmax)+0.002)) {         // When voltage reaches 63.2% of Vmax,
          C = (t / 1000) / R;                   // calculate capacitance (while converting time from ms to s)
        }
        displayOut();                           // Keep updating display
        Serial.print((int)(V * 1000));          // Send values for V (converted to mV), I (in mA), C (in F) and t (converted to s) over Serial
        Serial.print(",");
        Serial.print((int)I);
        Serial.print(",");        
        Serial.println((int)(t / 1000));
        delay(3);
      }
  }  
  digitalWrite(relayCS, LOW);         // Disconnect capacitors from supply
  digitalWrite(LEDCharge, LOW);       // Turn off charge LED
  t = 0;                              // Reset time to 0
  Serial.print("Charge cycle ");
  Serial.print(cycle);
  Serial.print(" done! C = ");
  Serial.println(C);
  playMelody(2);
  //delay(3000);
}

void idle() {                           // Function for idling
  digitalWrite(relayCS, LOW);           // Disconnect capacitors from supply
  digitalWrite(relayCD, LOW);           // Disconnect capacitors from discharge resistor
  //Serial.println("Ready...");
  V = measureVoltage();
  Vperc = (V / Vmax) * 100;             // Calculate percentage that current voltage is to value Vmax
  I = measureCurrent();
  displayOut();
}

void displayOut() {                       // Function for display handling
  bpTCheck();                             // This way user can switch between readout and graph mid-measurement. Graph would be partial, however.
  if (bpTVar == 0) {
    display.clearDisplay();
    display.setTextColor(WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.print("Voltage: ");
    display.setCursor(48, 0);
    display.print((int)(V * 1000));            // Voltage converted to mV
    display.setCursor(75, 0);
    display.print("mV");
    display.setCursor(93, 0);
    display.print("(");
    display.setCursor(99, 0);
    display.print((int)Vperc);
    display.setCursor(117, 0);
    display.print("%");
    display.setCursor(121, 0);
    display.print(")");
    display.setCursor(0, 16);
    display.print("Current: ");
    display.setCursor(48, 16);
    display.print((int)I);
    display.setCursor(75, 16);
    display.print("mA");
    display.setCursor(97, 16);
    display.print((int)(t / 1000));            // Time converted from ms to s
    display.setCursor(120, 16);
    display.print("s");
    display.drawFastHLine(0, 29, 128, WHITE);
    display.drawFastHLine(0, 33, 128, WHITE);
    display.drawFastVLine(89, 0, 64, WHITE);
    display.setCursor(6, 45);
    if (bpSPDVar == 1) {
      display.setTextSize(2);
      display.print("Ready");
      display.setCursor(65, 51);
      display.setTextSize(1);
      display.print("...");
    } else if (bpSPDVar == 2) {
      display.setCursor(4, 45);
      display.setTextSize(2);
      display.print("#CyS:");
      display.setTextSize(3);
      display.setCursor(65, 42);
      display.print(cycles);
    } else {
      display.setTextSize(3);
      if (C == -1) {
        display.setCursor(6, 35);
        display.print("...");
      } else {
        display.setCursor(6, 42);
        display.print((int)C);
      }
      display.setCursor(69, 42);
      display.print("F");
    }
    display.setTextSize(3);
    display.setCursor(103, 42);
    display.print(cycle);
    display.display();
    display.clearDisplay();
  } else {
    DrawCGraph(display, t / 1000, V, 9, 60, 100, 57, 0, 1500, 50, 0, 3, 1, 0, "", true);
    DrawBarChartV(display, I / 1000, 113,  61, 5, 57, 0, 3 , 1, 0, "", true);
  }
}

void DrawCGraph(Adafruit_SSD1306 &d, double x, double y, double gx, double gy, double w, double h, double xlo, double xhi, double xinc, double ylo, double yhi, double yinc, double dig, String title, boolean Redraw) {

  /*

    function to draw a cartesian coordinate system and plot whatever data you want
    just pass x and y and the graph will be drawn huge arguement list

    &display to pass the display object, mainly used if multiple displays are connected to the MCU
    x = x data point
    y = y datapont
    gx = x graph location (lower left)
    gy = y graph location (lower left)
    w = width of graph
    h = height of graph
    xlo = lower bound of x axis
    xhi = upper bound of x asis
    xinc = division of x axis (distance not count)
    ylo = lower bound of y axis
    yhi = upper bound of y asis
    yinc = division of y axis (distance not count)
    title = title of graph
    &redraw = flag to redraw graph on fist call only

  */

  double i;
  double temp;
  int rot, newrot;

  if (Redraw == true) {
    /*Redraw = false;
      d.fillRect(0, 0,  127 , 16, WHITE);
      d.setTextColor(BLACK, WHITE);
      d.setTextSize(1);
      d.setCursor(2, 4);
      d.println(title);*/
    ox = (x - xlo) * ( w) / (xhi - xlo) + gx;
    oy = (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
    // draw y scale
    d.setTextSize(1);
    d.setTextColor(WHITE, BLACK);
    for ( i = ylo; i <= yhi; i += yinc) {
      // compute the transform
      // note my transform funcition is the same as the map function, except the map uses long and we need doubles
      temp =  (i - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
      if (i == 0) {
        d.drawFastHLine(gx - 2, temp, w + 2, WHITE);
      }
      else {
        d.drawFastHLine(gx - 2, temp, 2, WHITE);
      }
      d.setCursor(gx - 9, temp - 3);
      d.println(i, dig);
    }
    // draw x scale
    for (i = xlo; i <= xhi; i += xinc) {
      // compute the transform
      d.setTextSize(1);
      d.setTextColor(WHITE, BLACK);
      temp =  (i - xlo) * ( w) / (xhi - xlo) + gx;
      if (i == 0) {
        d.drawFastVLine(temp, gy - h, h + 2, WHITE);
      }
      else {
        d.drawFastVLine(temp, gy, 2, WHITE);
      }
      d.setCursor(temp, gy + 6);
      d.println(i, dig);
    }
  }

  // graph drawn now plot the data
  // the entire plotting code are these few lines...

  x =  (x - xlo) * ( w) / (xhi - xlo) + gx;
  y =  (y - ylo) * (gy - h - gy) / (yhi - ylo) + gy;
  d.drawLine(ox, oy, x, y, WHITE);
  d.drawLine(ox, oy - 1, x, y - 1, WHITE);
  ox = x;
  oy = y;

  // up until now print sends data to a video buffer NOT the screen
  // this call sends the data to the screen
  d.display();
}

void DrawBarChartV(Adafruit_SSD1306 &d, double curval, double x , double y , double w, double h , double loval , double hival , double inc , double dig, String label, bool Redraw) {

  /*
    This method will draw a vertical bar graph for single input
    it has a rather large arguement list and is as follows

    &d to pass the display object, mainly used if multiple displays are connected to the MCU
    curval = date to graph (must be between loval and hival)
    x = position of bar graph (lower left of bar)
    y = position of bar (lower left of bar)
    w = width of bar graph
    h =  height of bar graph (does not need to be the same as the max scale)
    loval = lower value of the scale (can be negative)
    hival = upper value of the scale
    inc = scale division between loval and hival
    dig = format control to set number of digits to display. (not includeing the decimal)
    label = bottom lable text for the graph
    redraw = flag to redraw display. only on first pass (to reduce flickering)

  */

  double stepval, my, level, i, data;

  if (Redraw) {
    /*Redraw = false;
      d.fillRect(0, 0,  127 , 14, WHITE);
      d.setTextColor(BLACK, WHITE);
      d.setTextSize(1);
      d.setCursor(2, 4);
      d.println(label);*/
    // step val basically scales the hival and low val to the height
    // deducting a small value to eliminate round off errors
    // this val may need to be adjusted
    stepval = ( inc) * (double (h) / (double (hival - loval))) - .001;
    for (i = 0; i <= h; i += stepval) {
      my =  y - h + i;
      d.drawFastHLine(x + w + 0, my, 2, WHITE);
      // draw lables
      d.setTextSize(1);
      d.setTextColor(WHITE, BLACK);
      d.setCursor(x + w + 4, my - 3 );
      data = hival - ( i * (inc / stepval));
      d.print(data, dig);
    }
  }
  // compute level of bar graph that is scaled to the  height and the hi and low vals
  // this is needed to accompdate for +/- range
  level = (h * (((curval - loval) / (hival - loval))));
  // draw the bar graph
  // write a upper and lower bar to minimize flicker cause by blanking out bar and redraw on update
  d.drawRect(x, y - h, w, h, WHITE);
  d.fillRect(x, y - h, w, h - level,  BLACK);
  d.drawRect(x, y - h, w, h, WHITE);
  d.fillRect(x, y - level, w,  level, WHITE);
  // up until now print sends data to a video buffer NOT the screen
  // this call sends the data to the screen
  d.display();
}

void advanceCyclesNumber() {            // Function for advancing number of cycles
  if (cycles < cyclesMax) {
    cycles++;
  } else {
    cycles = 1;
  }
  Serial.println(cycles);
  displayOut();
  delay(1000);
  bpSPDVar = 1;
}

boolean bpTCheck() {                    // Function for checking state of toggle button
  bpTReading = digitalRead(bpTPin);
  if (bpTReading == HIGH && bpTPrevious == LOW && millis() - bpTTime > bpTDebounce) {
    if (bpTState == HIGH) {
      bpTState = LOW;
      bpTVar = 0;
    } else {
      bpTState = HIGH;
      bpTTime = millis();
      bpTVar = 1;
    }
  }
  bpTPrevious = bpTReading;
  return bpTState;
}

void testdrawline() {
  display.clearDisplay();
  for (int16_t i = 0; i < display.height(); i += 4) {
    display.drawLine(display.width() - 1, 0, 0, i, WHITE);
    display.display();
    delay(1);
  }
  for (int16_t i = 0; i < display.width(); i += 4) {
    display.drawLine(display.width() - 1, 0, i, display.height() - 1, WHITE);
    display.display();
    delay(1);
  }
}

void playMelody (int id) {

  int melody[4];
  int noteDurations[4];

  int durations[] = {8, 8, 8, 8};

  int melodyJingle[] = {NOTE_C5, NOTE_G5, NOTE_C6, NOTE_C6};  

  int melodySingle[] = {NOTE_C5, NOTE_E5, NOTE_G5, NOTE_D6};

  int melodyCycle[] = {NOTE_C6, NOTE_G5, NOTE_E5, NOTE_C5};

  switch (id) {
    case 1:
      for (int i = 0; i < 4; i++) {
        melody[i] = melodyJingle[i];
        noteDurations[i] = durations[i];
      }
      break;
    case 2:
      for (int i = 0; i < 4; i++) {
        melody[i] = melodySingle[i];
        noteDurations[i] = durations[i];
      }
      break;
    case 3:
      for (int i = 0; i < 4; i++) {
        melody[i] = melodyCycle[i];
        noteDurations[i] = durations[i];
      }
      break;
  }

  for (int thisNote = 0; thisNote < sizeof(melody); thisNote++) {
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(speaker, melody[thisNote], noteDuration);
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);    
    if(thisNote%2 == 0 && id == 1) {
          digitalWrite(LEDCharge, LOW);
          digitalWrite(LEDDischarge, HIGH);
        } else if(thisNote%2 != 0 && id == 1){
          digitalWrite(LEDCharge, HIGH);
          digitalWrite(LEDDischarge, LOW);
        }
    }
    digitalWrite(LEDCharge, LOW);
    digitalWrite(LEDDischarge, LOW);
    noTone(speaker);
}

float measureVoltage() {
  int sum = 0;                    // sum of samples taken
  unsigned char sample_count = 0; // current sample number
  float voltage = 0.0;            // calculated voltage

  while (sample_count < NUM_SAMPLES) {
    sum += analogRead(capVoltage);                                // Read voltage on capacitors, taking voltage divider and reference voltage into consideration
    sample_count++;
    delay(7);
  }

  V = ((float)sum / (float)NUM_SAMPLES) * (REF_VOLTAGE / 1024.0) * koef;
  return V;
}

float measureCurrent() {
  int sum = 0;                    // sum of samples taken
  unsigned char sample_count = 0; // current sample number
  float voltage = 0.0;            // calculated voltage

  while (sample_count < NUM_SAMPLES) {
    sum += analogRead(capCurrent);                                // Read voltage on capacitors, taking voltage divider and reference voltage into consideration
    sample_count++;
    delay(7);
  }

  Vr = ((float)sum / (float)NUM_SAMPLES) * (REF_VOLTAGE / 1024.0) * koef;
  I = (Vr / R) * 1000;                                                 // Calculate capacitor discharge current (converted to mA)
  return I;
}

void checkSPD() {
  short b = checkButton();
  if (b == 1) clickEvent();
  else if (b == 2) doubleClickEvent();
  else if (b == 3) holdEvent();
  else if (b == 4) longHoldEvent();
}

short checkButton() {
  short event = 0;
  // Read the state of the button
  buttonVal = digitalRead(bpSPD);
  // Button pressed down
  if (buttonVal == HIGH && buttonLast == LOW && (millis() - upTime) > debounce) {
    downTime = millis();
    ignoreUp = false;
    waitForUp = false;
    singleOK = true;
    holdEventPast = false;
    longHoldEventPast = false;
    if ((millis() - upTime) < DCgap && DConUp == false && DCwaiting == true) DConUp = true;
    else DConUp = false;
    DCwaiting = false;
  }
  // Button released
  else if (buttonVal == LOW && buttonLast == HIGH && (millis() - downTime) > debounce) {
    if (not ignoreUp) {
      upTime = millis();
      if (DConUp == false) DCwaiting = true;
      else {
        event = 2;
        DConUp = false;
        DCwaiting = false;
        singleOK = false;
      }
    }
  }
  // Test for normal click event: DCgap expired
  if ( buttonVal == LOW && (millis() - upTime) >= DCgap && DCwaiting == true && DConUp == false && singleOK == true) {
    event = 1;
    DCwaiting = false;
  }
  // Test for hold
  if (buttonVal == HIGH && (millis() - downTime) >= holdTime) {
    // Trigger "normal" hold
    if (not holdEventPast) {
      event = 3;
      waitForUp = true;
      ignoreUp = true;
      DConUp = false;
      DCwaiting = false;
      //downTime = millis();
      holdEventPast = true;
    }
    // Trigger "long" hold
    if ((millis() - downTime) >= longHoldTime) {
      if (not longHoldEventPast) {
        event = 4;
        longHoldEventPast = true;
      }
    }
  }
  buttonLast = buttonVal;
  return event;
}

void clickEvent() {               // bpSPDVar = 1
  bpSPDVar = 1;
}

void doubleClickEvent() {         // bpSPDVar = 2
  bpSPDVar = 2;
}

void holdEvent() {                // bpSPDVar = 3
  bpSPDVar = 3;
}

void longHoldEvent() {            // bpSPDVar = 4
  bpSPDVar = 4;
}


