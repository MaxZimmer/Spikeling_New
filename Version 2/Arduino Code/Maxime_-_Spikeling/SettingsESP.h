/* ----------------------------------------------------------------------------------------------
 
  Settings for the Adafruit HUZZAH32 Feather Board (ESP32)
    https://www.adafruit.com/product/3405
  in combination with the Adafruit 2.4" TFT FeatherWing,
    https://learn.adafruit.com/adafruit-2-4-tft-touch-screen-featherwing


  Libraries (to be installed via Arduino IDE/Library manager):
    - "Mcp3208" by Patrick Rogalla, v1.0.2
    - "Adafruit_STMPE610" by Adafruit, v1.0.1
    - "Mini Grafx" by Daniel Eichhorn, v1.0.0 (contains a driver for the ILI9341 TFT spi display)
    - "Mcp23s08" by sumotoy (https://github.com/sumotoy/gpio_expander) adapted by Thomas Euler
    - "LedControl" by Eberhard Fahle, v1.0.6 (http://wayoda.github.io/LedControl/)


---------------------------------------------------------------------------------------------- */

  //#define USES_FULL_REDRAW
  #define   USES_HOUSEKEEPING

  #include "Definitions.h"
  #include <SPI.h>
  #include "Adafruit_GFX.h"
  #include "ILI9341_SPI.h"
  #include "Adafruit_HX8357.h"
  #include <Adafruit_STMPE610.h>
  #include <Mcp3208.h>
  #include <Mcp23s08.h>
  #include <LedControl.h>       

// ----------------------------------------------------------------------------------------------

  #define   MCP3208_FIRST  100
  #define   MCP3208_LAST   107
  #define   MCP23S08_FIRST 110
  #define   MCP23S08_LAST  117
  #define   MAX7219_FIRST  120
  #define   MAX7219_LAST   129

/* ----------------------------------------------------------------------------------------------
                      Pin definitions (simulation-related)
---------------------------------------------------------------------------------------------- */
  
#define DISPLAY_TEST  0x0F /* Registre Display-Test */
#define SHUTDOWN      0x0C /* Registre Shutdown */
#define SCAN_LIMIT    0x0B /* Registre Scan-limit */
#define INTENSITY     0x0A /* Registre Intensity */ 

// LED Bargraph
enum color_t {NOCOLOR, RED, GREEN, YELLOW};
void bargraphDraw(color_t const *mybargraph);
void levelGauge(unsigned int v);
const byte nb_digits = 6;   /* number of digit on the bargraph */
const byte nb_segments_digit = 4; /* number of segments per digit */

// Digital and analog I/O helper macros
#define digitalReadHelper(pin)        digitalReadNew(pin)
#define digitalWriteHelper(pin, val)  digitalWriteNew(pin, val)
#define analogWriteHelper(pin, val)   analogWriteNew(pin, val)
#define analogReadHelper(pin)         analogReadNew(pin)
/*  analogReadNew() uses the 8-channel ADC MCP3208; therefore instead of pins, the mapping to the ADC channels 
on the chip are defined above. It's a 12 bit ADC, therefore the results need to be divided by 4. The MCP3208 
is connected to the second SPI bus of the ESP (HSPI), while the TFT display uses the primary SPI bus (VSPI). 
This is nescessary to be able to run the TFT at a particular frequency, at which the MCP3208 does not seem to 
work reliable. For pins, see "hardware add-ons" section further below.                                     */


// Serial out
// (115200=testing; 921600=standard; 1843200=a bit faster but less stable)
#define SerOutBAUD  921600

/* ----------------------------------------------------------------------------------------------
*                       Pin definitions (hardware add-ons)
*                       
*         function       address            equivalent on Spikeling 1 version
---------------------------------------------------------------------------------------------- */

  #define VmPotPin       MCP3208_FIRST+0    // -> A3       Resting membrane potential
  #define NoisePotPin    MCP3208_FIRST+1    // -> A6       scaling of Noise level
  #define PhotoDiodePin  MCP3208_FIRST+2    // -> A0       Photodiode
  #define Syn1PotPin     MCP3208_FIRST+3    // -> A7       efficacy synapse 1
  #define Syn2PotPin     MCP3208_FIRST+4    // -> A5       efficacy synapse 2
  #define AnalogInPin    MCP3208_FIRST+5    // -> A2       Analog in- takes 0-5V (positive only)
  #define AnalogPotPin   MCP3208_FIRST+6    // -> New      Analog in Gain
  #define StimPotPin     MCP3208_FIRST+7    // -> New      Stimulus Rate
  
  #define ModeLED        MCP23S08_FIRST+0   // -> builtin  Push button to switch spike modes
  #define ButtonPin      MCP23S08_FIRST+1   // -> 2        Push button to switch spike modes
  #define DigitalIn1Pin  MCP23S08_FIRST+2   // -> 4        Synapse 1 Input - expects 5V pulses
  #define DigitalIn2Pin  MCP23S08_FIRST+3   // -> 5        Synapse 2 input - expects 5V pulses
  #define StimulatorPin  MCP23S08_FIRST+4   // -> New      Stimulator out

  
    #define DIO_CS         4                            // secondary SPI bus (HSPI), client #2
    #define ADC_CS         13                           // secondary SPI bus (HSPI), client #1
    #define TFT_CS         15                           // primary SPI bus (VSPI), client #1
    #define DigitalOutPin  16                           // -> D3/DO    "Axon" - generates 5V pulses
    #define AnalogOutPin   17                           // -> D11/PWM  Analog out for full spike waveform
    #define ADC_MOSI       22                           // secondary SPI bus DataIn
    #define ADC_MISO       23                           // secondary SPI bus (HSPI) DataOut
    #define ADC_SCK        27                           // secondary SPI bus Clock
    #define TOUCH_CS       32                           // primary SPI bus (VSPI), client #2
    #define TFT_DC         33                           // Screen
    #define LED_CS         36                           // secondary SPI bus (HSPI), client #3
    #define TFT_RST        -1
  
    #define DIO_ADDR       0x20                         // address of MCP23S08 (defined by A0,A1 pins)
    #define ADC_VREF       5000                         // Vref for A/D
    #define ADC_CLK        1600000                      // secondary SPI bus (HSPI), clock

/* ----------------------------------------------------------------------------------------------
 *                         TFT Screen Definitions
 ---------------------------------------------------------------------------------------------- */
 
// Define colors usable in the paletted 16 color frame buffer

#define HX8357_BLACK   0x0000
#define HX8357_BLUE    0x001F
#define HX8357_RED     0xF800
#define HX8357_GREEN   0x07E0
#define HX8357_CYAN    0x07FF
#define HX8357_MAGENTA 0xF81F
#define HX8357_YELLOW  0xFFE0
#define HX8357_WHITE   0xFFFF
uint16_t palette[] = {HX8357_BLACK, // 0
                      HX8357_WHITE, // 1                      
                      HX8357_RED, // 2
                      HX8357_GREEN, // 3
                      HX8357_BLUE, // 4
                      HX8357_CYAN, // 5
                      HX8357_MAGENTA, // 6
                      HX8357_YELLOW}; // 7

const int SCREEN_WIDTH    = 480;
const int SCREEN_HEIGHT   = 320;
const int BITS_PER_PIXEL  = 4; // 2^4 = 16 colors
const int FONT_HEIGHT     = 14; // Standard font
const int SCREEN_ORIENT   = 1;


// Initialize the drivers
Adafruit_HX8357 tft = Adafruit_HX8357(TFT_CS, TFT_DC, TFT_RST);
Adafruit_STMPE610 ts      = Adafruit_STMPE610(TOUCH_CS);

unsigned long testLines(uint16_t color) {
  unsigned long start, t;
  int           x1, y1, x2, y2,
                w = tft.width(),
                h = tft.height();

  tft.fillScreen(HX8357_BLACK);

  x1 = y1 = 0;
  y2    = h - 1;
  start = micros();
  for(x2=0; x2<w; x2+=6) tft.drawLine(x1, y1, x2, y2, color);
  x2    = w - 1;
  for(y2=0; y2<h; y2+=6) tft.drawLine(x1, y1, x2, y2, color);
  t     = micros() - start; // fillScreen doesn't count against timing


  return micros() - start;
}

SPIClass        *hspi   = new SPIClass(HSPI);
MCP3208         adc(ADC_VREF, ADC_CS, hspi);
uint16_t        ADCData[8][2];
MCP23S08        dio(DIO_CS, DIO_ADDR, 0, hspi);
uint8_t         DIOData;
LedControl lc=LedControl(ADC_MOSI,ADC_SCK,LED_CS,2);
  

// Definitions and variables for plotting
#define INFO_DY      20  // Height of info panel
#define MAX_TRACES   3   // Maximal number of traces shown in parallel
#define MAX_VALUES   320 // Maximal trace length
#define PLOT_UPDATE  16  // Redraw screen every # values

const char* OutputInfoStr[] = {"V_m[mV]", "I_t[pA]", "I_PD[pA]", "I_AI[pA]",
                               "I_Sy[pA]", "StmSt", "SpIn1", "SpIn2",
                               "t[us]"};

int    TraceCols[MAX_TRACES] = {2,4,3};
int    Traces[MAX_TRACES][MAX_VALUES];
int    TracesStrIndex[MAX_TRACES];
int    TraceSet;
float  TracesMinMax[MAX_TRACES][2];
int    iPnt, dyPlot, dxInfo;
char   timeStr[16];

byte   allRed = 0x0F;
byte   allGreen = allRed << 4;
byte   allYellow = allRed | allGreen;


// -----------------------------------------------------------------------------
// Helpers
// -----------------------------------------------------------------------------

void setTraceSet()
{
  switch(TraceSet) {
    case 0:
    default:
      TracesStrIndex[0]  = ID_V;
      TracesMinMax[0][0] = -110;
      TracesMinMax[0][1] = 25;
      TracesStrIndex[1]  = ID_I_TOTAL;
      TracesMinMax[1][0] = -250;
      TracesMinMax[1][1] = 250;
      TracesStrIndex[2]  = ID_I_STIM_STATE;
      TracesMinMax[2][0] = -50;
      TracesMinMax[2][1] = 50;
  }
}

void writeRegister(byte thisRegister, byte thisValue) // Écrire dans un registre du MAX7219
{
  // Mettre l'entrée LOAD à l'état bas
  digitalWrite(LED_CS, LOW);

  SPI.transfer(thisRegister); // Adresse du registre
  SPI.transfer(thisValue); // Contenu du registre

  // Basculer l'entrée LOAD à l'état haut pour verrouiller et transférer les données
  digitalWrite(LED_CS, HIGH);
}
void initializeHardware()
{

    // Set pins
    pinMode(DigitalOutPin, OUTPUT);
    pinMode(ADC_CS, OUTPUT);
    digitalWrite(ADC_CS, HIGH);
    pinMode(DIO_CS, OUTPUT);
    digitalWrite(DIO_CS, HIGH);
    pinMode(LED_CS, OUTPUT);
    digitalWrite(LED_CS,HIGH);

    // Initialize SPI interface for MCP3208, MCP23S08 and MAX7219    
    SPISettings settingsHSPI(ADC_CLK, MSBFIRST, SPI_MODE0);
    hspi->begin(ADC_SCK, ADC_MISO, ADC_MOSI, ADC_CS);
    hspi->beginTransaction(settingsHSPI);
    for(int i=0; i<8; i++) {
      ADCData[0][0] = 0;
      ADCData[0][1] = 0;
    }
    dio.begin();
    dio.gpioPinMode(ButtonPin -MCP23S08_FIRST, INPUT);
    dio.gpioPinMode(DigitalIn1Pin -MCP23S08_FIRST, INPUT);
    dio.gpioPinMode(DigitalIn2Pin -MCP23S08_FIRST, INPUT);
    dio.gpioPinMode(ModeLED -MCP23S08_FIRST, OUTPUT);
    DIOData = 0;
    
    SPI.begin();
    SPI.setBitOrder(MSBFIRST); /* bits de poids fort en premier */

  // Initialize Screen
  tft.begin(HX8357D);

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(HX8357_RDPOWMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDCOLMOD);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDDIM);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(HX8357_RDDSDR);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 
  
  Serial.println(F("Benchmark                Time (microseconds)"));

  tft.setRotation(SCREEN_ORIENT);

  Serial.print(F("Lines                    "));
  Serial.println(testLines(HX8357_CYAN));
  delay(500);



  // Initialise a few variables
  iPnt = 0;
  dyPlot = SCREEN_HEIGHT -INFO_DY;
  dxInfo = SCREEN_WIDTH /(MAX_TRACES +1);
  timeStr[0] = 0;
  for(int i=0; i<MAX_TRACES; i++) {
    TracesMinMax[i][0] = 0;
    TracesMinMax[i][1] = 0;
  }
  TraceSet = 0;
  setTraceSet();

  
//  // Set rotation and clear screen
//  // (landscape, USB port up)
//  gfx.setFastRefresh(true);
//  gfx.fillBuffer(0);
//  gfx.setTextAlignment(TEXT_ALIGN_CENTER);
//  gfx.commit();
//}

////////////////////////////////////////////////////////////////////////////////////////////////
  //LED BarGraph
  writeRegister(DISPLAY_TEST, 0x00);  /* Normal operation */
  writeRegister(SCAN_LIMIT, nb_digits - 1); /* Display digits 0, 1, 2 */
  writeRegister(INTENSITY, 0x08);   /* LED brightness, max=0x0F */
  writeRegister(SHUTDOWN, 0x01);    /* Normal operation */
  writeRegister(SHUTDOWN, 0x00);  /* Shutdown mode */
}



void bargraphDraw(color_t const *mybargraph) {
  byte c;
  for (byte d = 0; d < nb_digits; d++) { /* pour chaque digit */
    c = 0;
    for (byte s = 0; s < nb_segments_digit; s++) { /* pour chaque segment du digit */
      switch (mybargraph[d * nb_segments_digit + s]) {
        case NOCOLOR : break;
        case RED : c |= (1 << s); break;
        case GREEN : c |= (1 << (s + 4)); break;
        case YELLOW :  c |= ((1 << s) | (1 << (s + 4))); break;
        default: break;
      }
    }
    writeRegister(d + 1, c);
  }
}

void levelGauge(unsigned int v) { /* indicateur de niveau, pourcentage entre 0 et 100 */
  unsigned int level ,
               levelGreen = nb_digits * nb_segments_digit - 3,
               levelYellow = levelGreen - 3;
      
  color_t bargraph[nb_digits * nb_segments_digit] = {NOCOLOR}; /* parce que NOCOLOR=0, toutes les autres valeurs non indiquées prendront la valeur 0 */

  if (v > -21)
    return;

  if (v > -90) {
    level = map(v, -90, -21, 0, nb_digits * nb_segments_digit - 1);

    if (level >= levelGreen) { /* niveau dans le vert */
      for (int i = levelGreen;i <= level; i++){
      bargraph[i] = GREEN;
      }
    }
    
    if (level >= levelYellow) { /* niveau dans le orange */
      for (int i = levelYellow; i <= min(level, levelGreen - 1); i++) {
        bargraph[i] = YELLOW;
      }
    }
    
    for (int i = 0; i<= min(level, levelYellow - 1); i++){/* niveau dans le rouge */
        bargraph[i] = RED;    
    }
  }
  bargraphDraw(bargraph);
}


// -----------------------------------------------------------------------------
// Digital and analog I/O helper macros

void digitalWriteNew(uint8_t pin, bool val)
{
  if((pin >= MCP23S08_FIRST) && (pin <= MCP23S08_LAST)) {
    // Handle output pins connected to the port expander MCP23S08
    #ifdef USES_HOUSEKEEPING
    dio.gpioDigitalWriteFast(pin -MCP23S08_FIRST, val);
    #else
    dio.gpioDigitalWrite(pin -MCP23S08_FIRST, val);
    #endif
  }
  else {
    // Handle directly to the ESP connected pins
    switch(pin) {
      case DigitalOutPin:
        digitalWrite(pin, val);
        break;
    }
  }
}

int digitalReadNew(uint8_t pin)
{
  if((pin >= MCP23S08_FIRST) && (pin <= MCP23S08_LAST)) {
    // Currently only input pins connected to the port expander
    // MCP23S08 are handled
    #ifdef USES_HOUSEKEEPING
    return (DIOData & 0x01 << (pin -MCP23S08_FIRST)) > 0 ? HIGH : LOW;
    #else
    return dio.gpioDigitalRead(pin -MCP23S08_FIRST) > 0 ? HIGH : LOW;
    #endif
  }
  return LOW;
}


int analogReadNew(int pin)
{
  uint16_t res = 0;

  if((pin >= MCP3208_FIRST) && (pin <= MCP3208_LAST)) {
    // Currently only input pins connected to A/D IC MCP3208 are handeled
    switch (pin) {
      case Syn1PotPin:
        res = 300;
        break;
      case Syn2PotPin:
        res = 128;
        break;
      case AnalogInPin:
        res = 100;
        break;

      default:
        #ifdef USES_HOUSEKEEPING
        res = ADCData[pin -MCP3208_FIRST][1] >> 2;
        #else
        // *** TODO ***
        #endif
    }
  }
  return res;
}

void analogWriteNew(int pin, int val)
{
  ledcWrite(pin, val);
}

// -----------------------------------------------------------------------------
// Housekeeping routine, to be called once per loop
// -----------------------------------------------------------------------------

void housekeeping()
{
  uint16_t v;
  uint8_t  iCh;

  // Read A/D channels from MCP3208 and store data
  for(iCh=0; iCh<3; iCh++) {
    v  = adc.read(MCP3208::Channel(iCh | 0b1000));
    if((v > 4066) || (v < 30)) {
      ADCData[iCh][1] = ADCData[iCh][0];
    }
    else {
      ADCData[iCh][0] = v;
      ADCData[iCh][1] = v;
    }
   }

   // Refresh MCP23S08 and retrieve data
   DIOData = dio.readGpioPort();
   dio.gpioPortUpdate();
}


// -----------------------------------------------------------------------------
// Graphics
// -----------------------------------------------------------------------------

int getYCoord(int iTr, float v)
{
  // Convert the value into a coordinate on the screen
  //
  return SCREEN_HEIGHT -1 -map(round(v), TracesMinMax[iTr][0], TracesMinMax[iTr][1], 0, dyPlot);
}


void plot(output_t* Output)
{
  int y, iTr;

  // Depending on selected trace set, add data to trace array
  switch(TraceSet) {
    case 0:
    default:
      Traces[0][iPnt] = getYCoord(0, Output->v);
      Traces[1][iPnt] = getYCoord(1, Output->I_total);
      Traces[2][iPnt] = getYCoord(2, Output->Stim_State);
  }

  // Draw new piece of each trace
  if(iPnt > 0) {
    #ifndef USES_FULL_REDRAW
      tft.fillScreen(HX8357_BLACK);
      tft.drawLine(iPnt, 0, iPnt, dyPlot,HX8357_BLACK);
    #endif
    for(iTr=0; iTr<MAX_TRACES; iTr++) {
      tft.drawLine(iPnt-1, Traces[iTr][iPnt-1], iPnt, Traces[iTr][iPnt],TraceCols[iTr]);
    }
  }

  iPnt++;
  if(iPnt >= MAX_VALUES) {
    // Set trace data pointer to the beginning of the array and clear screen
    iPnt = 0;
    #ifndef USES_FULL_REDRAW
      tft.fillScreen(HX8357_BLACK);
      tft.drawLine(0, 0, 0, dyPlot,HX8357_BLACK);
    //#else
      //gfx.fillBuffer(0);
    #endif

    // Redraw info area
    for(iTr=0; iTr<MAX_TRACES; iTr++) {
      tft.setCursor(dxInfo/2 +(iTr+1)*dxInfo +5, dyPlot);
      tft.setTextColor(TraceCols[iTr]);  tft.setTextSize(1);
      tft.println(OutputInfoStr[TracesStrIndex[iTr]]);
    }
  }
  if((((iPnt-1) % PLOT_UPDATE) == 0) || (iPnt == 0)) {
    // Redraw time and mode
    // (first old string in black, then new string in white; this is much faster then
    //  clearing the info area with a filled rectangle)
    tft.setCursor(dxInfo/2, dyPlot);
    tft.setTextColor(HX8357_BLACK);  tft.setTextSize(1);
    tft.println(timeStr);
    sprintf(timeStr, "M%d %.1fs\n", Output->NeuronBehaviour, Output->currentMicros /1E6);
    tft.setCursor(dxInfo/2, dyPlot);
    tft.setTextColor(HX8357_WHITE);  tft.setTextSize(1);
    tft.println(timeStr);
  }
}

// -----------------------------------------------------------------------------








unsigned long testFastLines(uint16_t color1, uint16_t color2) {
  unsigned long start;
  int           x, y, w = tft.width(), h = tft.height();

  tft.fillScreen(HX8357_BLACK);
  start = micros();
  for(y=0; y<h; y+=5) tft.drawFastHLine(0, y, w, color1);
  for(x=0; x<w; x+=5) tft.drawFastVLine(x, 0, h, color2);

  return micros() - start;
}

unsigned long testRects(uint16_t color) {
  unsigned long start;
  int           n, i, i2,
                cx = tft.width()  / 2,
                cy = tft.height() / 2;

  tft.fillScreen(HX8357_BLACK);
  n     = min(tft.width(), tft.height());
  start = micros();
  for(i=2; i<n; i+=6) {
    i2 = i / 2;
    tft.drawRect(cx-i2, cy-i2, i, i, color);
  }

  return micros() - start;
}

unsigned long testFilledRects(uint16_t color1, uint16_t color2) {
  unsigned long start, t = 0;
  int           n, i, i2,
                cx = tft.width()  / 2 - 1,
                cy = tft.height() / 2 - 1;

  tft.fillScreen(HX8357_BLACK);
  n = min(tft.width(), tft.height());
  for(i=n; i>0; i-=6) {
    i2    = i / 2;
    start = micros();
    tft.fillRect(cx-i2, cy-i2, i, i, color1);
    t    += micros() - start;
    // Outlines are not included in timing results
    tft.drawRect(cx-i2, cy-i2, i, i, color2);
  }

  return t;
}

unsigned long testFilledCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int x, y, w = tft.width(), h = tft.height(), r2 = radius * 2;

  tft.fillScreen(HX8357_BLACK);
  start = micros();
  for(x=radius; x<w; x+=r2) {
    for(y=radius; y<h; y+=r2) {
      tft.fillCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testCircles(uint8_t radius, uint16_t color) {
  unsigned long start;
  int           x, y, r2 = radius * 2,
                w = tft.width()  + radius,
                h = tft.height() + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros();
  for(x=0; x<w; x+=r2) {
    for(y=0; y<h; y+=r2) {
      tft.drawCircle(x, y, radius, color);
    }
  }

  return micros() - start;
}

unsigned long testTriangles() {
  unsigned long start;
  int           n, i, cx = tft.width()  / 2 - 1,
                      cy = tft.height() / 2 - 1;

  tft.fillScreen(HX8357_BLACK);
  n     = min(cx, cy);
  start = micros();
  for(i=0; i<n; i+=5) {
    tft.drawTriangle(
      cx    , cy - i, // peak
      cx - i, cy + i, // bottom left
      cx + i, cy + i, // bottom right
      tft.color565(200, 20, i));
  }

  return micros() - start;
}

unsigned long testFilledTriangles() {
  unsigned long start, t = 0;
  int           i, cx = tft.width()  / 2 - 1,
                   cy = tft.height() / 2 - 1;

  tft.fillScreen(HX8357_BLACK);
  start = micros();
  for(i=min(cx,cy); i>10; i-=5) {
    start = micros();
    tft.fillTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(0, i, i));
    t += micros() - start;
    tft.drawTriangle(cx, cy - i, cx - i, cy + i, cx + i, cy + i,
      tft.color565(i, i, 0));
  }

  return t;
}

unsigned long testRoundRects() {
  unsigned long start;
  int           w, i, i2,
                cx = tft.width()  / 2 ,
                cy = tft.height() / 2 ;

  tft.fillScreen(HX8357_BLACK);
  w     = min(tft.width(), tft.height());
  start = micros();
  for(i=0; i<w; i+=8) {
    i2 = i / 2 - 2;
    tft.drawRoundRect(cx-i2, cy-i2, i, i, i/8, tft.color565(i, 100, 100));
  }

  return micros() - start;
}

unsigned long testFilledRoundRects() {
  unsigned long start;
  int           i, i2,
                cx = tft.width()  / 2 + 10,
                cy = tft.height() / 2 + 10;

  tft.fillScreen(HX8357_BLACK);
  start = micros();
  for(i=min(tft.width(), tft.height()) - 20; i>25; i-=6) {
    i2 = i / 2;
    tft.fillRoundRect(cx-i2, cy-i2, i-20, i-20, i/8, tft.color565(100, i/2, 100));
  }

  return micros() - start;
}
