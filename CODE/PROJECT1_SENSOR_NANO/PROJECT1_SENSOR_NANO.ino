#include <Arduino.h>

/*********************************************************************
This code is writen by Jonathan Jenkins. 

Key Information:
  - Revision 1.0
  - Board : Arduino nano
  
 *  if you are sharing this code, you must keep this copyright note.
*********************************************************************/


 // **************** INCLUDE NECESSARY LIBRARIES *****************

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>




// **************** VARIABLES *****************

// INITIATE CONSTANT INTEGERS
// constants won't change. They're used here to set pin numbers:
  const int fsrPin = A2; //  the FSR and 10K pulldown
  const int HALLpin = A1; // input pullup
  const int Temppin = A3; // input pullup
  const int pixel = 4 ;


  
 // INITIATE VARIABLES (GLOBAL)
 // variables will change:
  int SerialSensorMore = false; // PUSH BUTTON 1 --- ARM 
  int forceMeasured = 0; // PUSH BUTTON 1 --- ARM 
  int tempmeasured = 0; // PUSH BUTTON 1 --- ARM 
  int force2measured= 0; // PUSH BUTTON 1 --- ARM 
  int force1measured = 0; // PUSH BUTTON 1 --- ARM 
   int pixelLT = 0;
    int pixelRT = 0 ;

// **************** Thermistor:  Adafruit *****************


// which analog pin to connect
#define THERMISTORPIN Temppin         
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
#define NUMSAMPLES 5
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000    
int samples[NUMSAMPLES];


int ThermistorPin = A3;
int Vo;
float R1 = 10000;
float logR2, R2, T;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


// **************** Hall Effect :  Adafruit *****************
//A0 used with analog output, D2 with digital output
#define Hall_Sensor HALLpin
//Here you can store both values, the Val2 can be boolean
int Val1=0,Val2=0;
int hallReading;     // the analog reading from the FSR resistor divider
int hallVoltage;     // the analog reading converted to voltage
unsigned long hallResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long hallConductance; 
long hallForce;       // Finally, the resistance converted to force



// **************** FSR: *****************
/* Connect one end of FSR to power, the other end to Analog 0.
Then connect one end of a 10K resistor from Analog 0 to ground  */
 int fsrReading;     // the analog reading from the FSR resistor divider
int fsrVoltage;     // the analog reading converted to voltage
unsigned long fsrResistance;  // The voltage converted to resistance, can be very big so make "long"
unsigned long fsrConductance; 
long fsrForce;       // Finally, the resistance converted to force


// **************** OLED LIBRARY *****************
// OLED SCREEN  SETUP
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library. 
// On an arduino UNO:       A4(SDA), A5(SCL)
// this is the Width and Height of Display which is 128 xy 32 
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
#define XPOS 0
#define YPOS 1
#define DELTAY 2
double count=0;

// **************** NEOPIXEL LIBRARY *****************

// Simple demonstration on using an input device to trigger changes on your
// NeoPixels. Wire a momentary push button to connect from ground to a
// digital IO pin. When the button is pressed it will change to a new pixel
// animation. Initial state has all pixels off -- press the button once to
// start the first animation. As written, the button does not interrupt an
// animation in-progress, it works only when idle.

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

// Digital IO pin connected to the button. This will be driven with a
// pull-up resistor so the switch pulls the pin to ground momentarily.
// On a high -> low transition the button press logic will execute.
#define BUTTON_PIN   2
#define PIXEL_PIN    4  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT 35  // Number of NeoPixels

// Declare our NeoPixel strip object:
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);
// Argument 1 = Number of pixels in NeoPixel strip
// Argument 2 = Arduino pin number (most are valid)
// Argument 3 = Pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
//   NEO_RGBW    Pixels are wired for RGBW bitstream (NeoPixel RGBW products)












// ****************SETUP *****************
void setup() {
  Serial.begin(9600); 
  Serial.println();
  Serial.println("Starting...");

//OLED SETUP
  // OLED by default, we'll generate the high voltage from the 3.3v line internally! (neat!)
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32)
  // drawing commands to make them visible on screen!
   // the library initializes this with an Adafruit splash screen.
  display.display();
  delay(2000);
   // Clear the buffer.
  display.clearDisplay(); 

//NEOPIXEL SETUP
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
  #if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
  #endif
  // END of Trinket-specific code.
 // pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'

//  sensors.begin();  // Start up the library


  Serial.println("READY");
  delay(100);
}
// ****************End SETUP ***********************




// ****************  Main Loop *****************

void loop() {
   thermistor();
   hall_effect();
    FSR();
    neopixel();   // function that controls Neopixel state
    UI();   // function that controls changing arm state   

  delay(200);
}

// **************** End Main Loop *****************



void FSR(void) {
   fsrReading = analogRead(fsrPin);  
  // analog voltage reading ranges from about 0 to 1023 which maps to 0V to 5V (= 5000mV)
  fsrVoltage = map(fsrReading, 0, 1023, 0, 5000);

  if (SerialSensorMore == true) {
    Serial.print("FSR Analog reading = ");
    Serial.println(fsrReading);
    Serial.print("FSR Voltage reading in mV = ");
    Serial.println(fsrVoltage);
  } 
 
  if (fsrVoltage == 0) {
    Serial.println("FSR: No pressure");  
  } else {
    // The voltage = Vcc * R / (R + FSR) where R = 10K and Vcc = 5V
    // so FSR = ((Vcc - V) * R) / V        yay math!
    fsrResistance = 5000 - fsrVoltage;     // fsrVoltage is in millivolts so 5V = 5000mV
    fsrResistance *= 10000;                // 10K resistor
    fsrResistance /= fsrVoltage;
    fsrConductance = 1000000;           // we measure in micromhos so 
    fsrConductance /= fsrResistance;

 
    // Use the two FSR guide graphs to approximate the force
    if (fsrConductance <= 1000) {
      fsrForce = fsrConductance / 80;
      force1measured = fsrForce *101.97162; //newtons to grams
    } else {
      fsrForce = fsrConductance - 1000;
      fsrForce /= 30; 
      force1measured = fsrForce *101.97162; //newtons to grams

      
    }

    if (SerialSensorMore == true) {
     Serial.print("FSR resistance in ohms = ");
      Serial.println(fsrResistance);
      Serial.print("FSR Conductance in microMhos: ");
      Serial.println(fsrConductance);
      Serial.print("FSR Force in Newtons: ");
      Serial.println(fsrForce);
  } 
  
  }


}






void thermistor(){



 Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  T = (T * 9.0)/ 5.0 + 32.0; 
  T=-93-T;
  T=abs(T);

   tempmeasured=T;

 if (SerialSensorMore == true) {
      Serial.print("Temperature: "); 
  Serial.print(T);
  Serial.println(" F"); 

  } 

   
}



void hall_effect(){

uint8_t i;
  float averagehall;

  // take N samples in a row, with a slight delay
  for (i=0; i< NUMSAMPLES; i++) {
   samples[i] = analogRead(Hall_Sensor);
   delay(10);
  }
  
  // average all the samples out
  averagehall = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     averagehall += samples[i];
  }
  averagehall /= NUMSAMPLES;


// convert the value to resistance
    hallReading = map (averagehall,470,250,0,220);

    if (SerialSensorMore == true) {
      Serial.print("hall analog: "); 
      Serial.println(averagehall);
      Serial.print("hall absolute: "); 
      Serial.println(hallReading);
      Serial.print("hall force: "); 
      Serial.print(hallForce);
      Serial.println(" *g");
  } 
 
     Serial.print("hall analog: "); 
      Serial.println(averagehall);
      Serial.print("hall absolute: "); 
      Serial.println(hallReading);
      Serial.print("hall force: "); 
      Serial.print(hallForce);
      Serial.println(" *g");  

      delay(1000);
  if (hallReading < 3) {
      force2measured=0 ;         
  // We can set some threshholds to display how much pressure is roughly applied:
 }
 else if (hallReading < 7) {
     force2measured = map (hallReading,0,7,0,120);}
     
 else if (hallReading < 15) {
     force2measured = map (hallReading,7,15,40,300);
  } else if (hallReading < 35) {
     force2measured = map (hallReading,15,35,300,470);
  }
  
else if (hallReading < 77) {
     force2measured = map (hallReading,35,77,470,760);
  }
else if (hallReading < 225) {
     force2measured = map (hallReading,77,225,760,1000);
  }
else if (hallReading >= 225) {
     force2measured = 1000;
  }
}


void UI() {

int forcemeasured; 
if (force1measured < 200) {
forcemeasured=force2measured;
  // We can set some threshholds to display how much pressure is roughly applied:
 }
else if (force1measured >2 && force2measured < 400) {
forcemeasured=force1measured;
  // We can set some threshholds to display how much pressure is roughly applied:
 }
 
else  {
forcemeasured=force1measured+force2measured;
         forcemeasured=(force1measured+force2measured)/2;  }
  


  
  // UPDATE UI
  
// using a float and the
    display.clearDisplay();
    
   //  TEMP STATUS;
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(4,2);
    display.println("TEMP:"); //do something when var equals 1
    display.setCursor(50,2);
    display.print(tempmeasured); 
    display.print(' '); 
    display.print('F'); 
    
   //  Force STATUS;
    display.setTextSize(2);
    display.setTextColor(WHITE);
    display.setCursor(4,14);
 display.print(forcemeasured);  
     display.print(' '); 
    display.print('g');    
    display.display();

 Serial.print("forcemeasured: "); 
      Serial.println(forcemeasured);
      Serial.print("force1measured: "); 
      Serial.println(force1measured);
      Serial.print("force2measured: "); 
      Serial.println(force2measured);
           Serial.print("forceMeasured: "); 
      Serial.println(forceMeasured);
     
    
}












void neopixel() {


  pixelLT = map (force1measured,0,500,0,18);
  pixelRT = map (hallReading,0,160,0,18);


    
 if (tempmeasured < 80) {
      colorWipe1(strip.Color(  0,   0, 255), 5,pixelLT);    // Blue            
      colorWipe2(strip.Color(  0,   0, 255), 5,pixelRT);    // Blue
  // We can set some threshholds to display how much pressure is roughly applied:
 }else if (tempmeasured < 90) {
      //colorWipe(strip.Color(  0,   0,   0), 5);    // Black/off          
      colorWipe1(strip.Color(  0, 255,   0), 5,pixelLT);    // Green 
      colorWipe2(strip.Color(  0, 255,   0), 5,pixelRT);    // Green 

  } else if (tempmeasured < 150) {
      colorWipe1(strip.Color(255,   0,   0), 5,pixelLT);    // Red
      colorWipe2(strip.Color(255,   0,   0), 5,pixelRT);    // Red
  }


}



// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe1(uint32_t color, int wait,int limit) {
  

int pixelorder[] = {9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26};

  for(int i=1; i<18; i++) { // For each pixel in strip...

   int pixel =  pixelorder[i];

    if (i < limit) {
         strip.setPixelColor(pixel, color);         //  Set pixel's color (in RAM)
       }
   strip.show();                          //  Update strip to match
    delay(wait); //  Pause for a moment 
}

  for(int i=18; i>1; i--) { // For each pixel in strip...
  int pixel =  pixelorder[i];
  if (i > limit) {
        strip.setPixelColor(pixel, strip.Color(  0,   0,   0));         // / Black/off          
          strip.show();                          //  Update strip to match
    delay(wait); //  Pause for a moment 
  }
  }
}






  
// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
// 0-9 &26-35
void colorWipe2(uint32_t color, int wait, int limit) {


int pixelorder[] = {9,8,7,6,5,4,3,2,1,0,34,33,32,31,30,29,28,27};

  for(int i=1; i<18; i++) { // For each pixel in strip...

   int pixel =  pixelorder[i];

    if (i < limit) {
         strip.setPixelColor(pixel, color);         //  Set pixel's color (in RAM)
       }
   strip.show();                          //  Update strip to match
    delay(wait); //  Pause for a moment 
}

  for(int i=18; i>1; i--) { // For each pixel in strip...
  int pixel =  pixelorder[i];
  if (i > limit) {
        strip.setPixelColor(pixel, strip.Color(  0,   0,   0));         // / Black/off          
          strip.show();                          //  Update strip to match
    delay(wait); //  Pause for a moment 
  }
  }
}



  
  
