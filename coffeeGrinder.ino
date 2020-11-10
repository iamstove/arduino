/* Works as is
 * TODO: 
 *  1 - Add double tap always on
 *  2 - Dynamic computation of power draw
 */

#include <U8g2lib.h>
#include <U8x8lib.h>
#include <elapsedMillis.h>

#define WIDTH 32
#define HEIGHT 10
//delay for turning off in miliseconds
#define DELAY 3000
//number of samples
#define SAMPLECOUNT 10

//state machine states
enum State
{
    STANDBY,
    STARTGRIND,
    GRINDING
} state;

//pin defines 
const int buttonPin = PIN2;
const int ammeterPin = A0;
const int controlPin = PIN7;

//global variables 
elapsedMillis timeout;
int buttonState = 0;
double ammeterState = 0;
double ampMin = 4.72;
double amps = 0;

U8G2_SSD1306_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ 16, /* data=*/ 17);

void setup() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_helvB08_tr);

  // initialize input pins
  pinMode (buttonPin, INPUT);
  pinMode (ammeterPin, INPUT);

  // init output pins
  pinMode (controlPin, OUTPUT);
  
  state = STANDBY;
}

void loop() {
  //update display
  displayUpdate ();


  // state machine for processing the grinder state
  switch (state)
  {
    case STANDBY:
    {}
      // the only thing we do in standy is check if we need to start
      buttonState = digitalRead (buttonPin);

      // if it's pressed change states
      if (buttonState == HIGH)
      {
        // close relay 
        state = STARTGRIND;
      }
      break;

    case STARTGRIND:
      // this state closes the relay and takes the first reading
      // close relay
      digitalWrite (controlPin, HIGH);

      // check the ammeter and calculate the minimum
      checkAmmeter ();
      // the min should be 50% of the load read
      //ampMin = amps * .5;

      //start the timer and change state
      timeout = 0;
      state = GRINDING;
      break; 

    case GRINDING:
      // continually check the ammeter, if it's less than the min stop the grinder
      checkAmmeter ();

      if ((amps < ampMin) && (timeout >= DELAY))
      {
        //start the timer, when it expires shut off
        // close the relay
        digitalWrite (controlPin, LOW);
        state = STANDBY;
        timeout = 0;        
      }
      
      if (amps > ampMin)
      {
        //reset the timer.
        timeout = 0;
      }
      

      break;

  }
}

// returns the average over 20ms, this sets the global variable amps to the current average reading 
void checkAmmeter()
{
  int sampleTot = 0;
  double ammeterAvg = 0;

  // check the ammeter amprage and report if it's time to turn off the grinder
  for (int i = 0; i < SAMPLECOUNT; i++)
  {
    sampleTot += analogRead (ammeterPin);
    //add a little delay
    delay (2);
  }

  ammeterAvg = sampleTot / (double)SAMPLECOUNT;
  amps = inputToAmps (ammeterAvg);
  
  return;
}

/* 
 * coverts the analog input into amps
 * uses ACS712 5A, https://www.addicore.com/ACS712-Current-Sensor-Module-5A-p/ad468.htm
 */
double inputToAmps (double pinReading)
{
  double v = 0;
  double in = 0;
  double out = 0;

  //with the ACS712 a reading of 512 is 0 amps
  in = (double)pinReading/(double)1024;
  // this outputs 0-5 volts
  v = (double)5*in;
  // the current flows in both directions so if it drops below 2.5v outupt that's a negative current, we only care about the absolute value
  v = abs(v - 2.5);
  // 0.185 is the scale factor (185mV / amp)
  out = v / 0.185;

  return out;
}

//updates the display with some global variable's status 
void displayUpdate (void)
{

  u8g2.firstPage ();
  do
  {
    u8g2.setCursor (0,10);
    u8g2.print ("Ammeter Reading");

    u8g2.setCursor (0,20);
    u8g2.print (amps);

    u8g2.setCursor (0,30);
    if (buttonState == HIGH)
    {  
      u8g2.print ("Button");
    }
    else{
      u8g2.print ("No Button");
    }

    u8g2.setCursor (0,40);
    u8g2.print (timeout);

    u8g2.setCursor (0,50);
    u8g2.print (state);

  } while (u8g2.nextPage());

  return;
}