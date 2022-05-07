#include <dht_nonblocking.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Wire.h>
#include "RTClib.h"
RTC_DS1307 rtc;

//LED pins
#define red 47 //water level
#define green 51 //idle state
#define yellow 49 //disable state
#define blue 53 //running


#define DHT_SENSOR_TYPE DHT_TYPE_11
char week[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};


Servo myservo;
const int j = 0b00000010;

//LED int
int rv = 255;
int gv = 255;
int bv = 255;
int yv = 255;

//Define ADC Register Pointers
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;


int adc_id = 0;
int hiv = 0;
char bf[128];

//lcd pins
LiquidCrystal lcd(12,11,10,9,8,7);

static const int DHT_SENSOR_PIN = 2;
DHT_nonblocking dht_sensor( DHT_SENSOR_PIN, DHT_SENSOR_TYPE );

//define port E pointers
volatile unsigned char* port_e = (unsigned char*) 0x2E;
volatile unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;

//define PORT B pointers
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

//define PORT L pointers
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;


//define port G pointers
volatile unsigned char* port_g = (unsigned char*) 0x34;
volatile unsigned char* ddr_g = (unsigned char*) 0x33;
volatile unsigned char* pin_g = (unsigned char*) 0x32;
//end of DC motor code


int in = 52;
int out = 50; 

int state = LOW;      // output state
int reading;          // input reading
int previous = HIGH;  // previous reading

long time = 0;
long db = 200;  //debounce time

void setup( )
{
  Serial.begin( 9600);
  adc_init();
  myservo.attach(13);
  myservo.write(90);
  lcd.begin(16, 2);

  *ddr_l |= B00000100;
  *ddr_b |= B00000100;
  *ddr_b |= B00000001;
  *ddr_l |= B00000001;
  *ddr_l &= B11111011;
  
  *port_l |= B00000001;

 //DC motor setup
 //Set PE5 to output
  *ddr_e |= 0x20;
//set PE3 to output
  *ddr_e |= 0x08;
//set PG5 to output
  *ddr_g |= 0x20;

  pinMode(in, INPUT);
  pinMode(out, OUTPUT);

  //RTC error
 if (! rtc.begin()) {
   Serial.println("Couldn't find RTC");
   while (1);
 }
 if (! rtc.isrunning()) {
   Serial.println("RTC is NOT running!");
 }
}

void loop( )
{  
  float tp;
  float hd;
  reading = digitalRead(in);

  if (reading == HIGH && previous == LOW && millis() - time > db) {
    if (state == HIGH){
      state = LOW;
      Serial.print("OFF\n");
    }
    else{
      Serial.print("ON\n");
      state = HIGH;
    }
    time = millis();    
  }

  digitalWrite(out, state);

  previous = reading;

  if(state == HIGH){
  if( me(&tp, &hd) == true )
  {
    timeStamp();
    Serial.print( "tp: " );
    float op1 = tp * 1.8;
    float op2 = op1 + 32; //C to F
    tp = op2;
    Serial.print(tp, 1);
    Serial.print("\n");
    Serial.print( "deg. F, hd = " );
    Serial.print(hd, 1);
    Serial.print("%\n");
  }
    //water sensor loop code:
    int num = adc_read(adc_id); // get adc value
    
    //red LED water level low loop
    if(num < 50){ errorLED(num); }
    
    if(((hiv>=num) && ((hiv - num) > 10)) || ((hiv<num) && ((num - hiv) > 10)))
    {
      sprintf(bf,"Water level is %d\n", num);
      Serial.print(bf);
      hiv = num;
    } //end of water sensor loop code

    //servo loops:
  servoLoop();
  
if(tp > 0){
  lcdScreen(tp, hd);
}

  mot(tp,num);
  }
  else if(state == LOW){
  lcd.setCursor(0, 0);
  lcd.print("STATUS: ");
  lcd.setCursor(0, 1);
  lcd.print("IDLE...");


  *ddr_b  |= B00000001;
   *ddr_l &= B11111011;
  *port_l |= B00000001;
  *port_b &= B11111011;
    
  }
}

void mot(float tp, float value){
      if(tp > 76 && value > 50){
  //write a 1 to the enable bit on PE3
  *port_e |= 0x08;
  *port_b |= B00000001;
  *port_l &= 11111010;
  *port_b &= B11111011;
  }
  
  if(tp < 76){
  *port_e &= 0x00;
  *port_b &= B11111110; //blue
  *port_l &= B11111010; //red & yellow
  *port_b |= B00000100; //green
  }
//write a 1 to PE5
*port_e |= 0x20;

//write a 0 to PG5
*port_g &= 0x20;

  }

void errorLED(int waterLevel){
  *port_b &= B11111010;
  *port_l |= B00000100;  
  lcd.setCursor(0, 0);
  lcd.print("Error!");
  lcd.setCursor(0, 1);
  lcd.print("WATER TOO LOW!");
  waterLevel = adc_read(adc_id);
  *port_e &= 0x00;
  delay(4000);
  if(waterLevel < 50){
  errorLED(waterLevel);
  }
}

void lcdScreen(float tp, float hd){
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 0);
  lcd.print("hd: ");
  lcd.print(hd, 1);
  lcd.print("%");
  
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print("Fahrenheit: ");
  lcd.print(tp, 1);
}

void servoLoop(){
  int v = adc_read(j);//read voltage from POT
  int turn = v/5.7;//Scale down analog input to be between 180 and 0
  myservo.write(turn);// move servos   
}

void timeStamp(){
//temperatute reading
 DateTime now = rtc.now();
 Serial.print("\nTime: ");
 Serial.print(now.month(), DEC);
 Serial.print('/');
 Serial.print(now.day(), DEC);
 Serial.print(" ");
 int hour = now.hour();
 hour -= 4;
 Serial.print(hour, DEC);
 Serial.print(':');
 Serial.print(now.minute(), DEC);
 Serial.print(':');
 Serial.print(now.second(), DEC);
 Serial.println();
 delay(3000); 
}

static bool me( float *tp, float *hd )
{
  static unsigned long mt = millis( );

  if( millis( ) - mt > 3000ul )
  {
    if( dht_sensor.measure( tp, hd ) == true )
    {
      mt = millis( );
      return( true );
    }
  }

  return( false );
}

//water sensor & servo
void adc_init()
{

  // setup the A register
  *my_ADCSRA |= B10000000;
  *my_ADCSRA &= B11110111;
  *my_ADCSRA &= B11011111;
  
  // setup the B register
  *my_ADCSRB &= B11111000;

  // setup the MUX Register
  *my_ADMUX |= (1<<REFS0);
}

//water sensor & servo
unsigned int adc_read(unsigned int adc_channel_num)
{
   
  int ch;
  // clear bits (MUX 4:0)
  *my_ADMUX &= B11100000;

  // clear bits (MUX 5)
  *my_ADCSRB &= B11110111;

  //Assign using MUX 5:0
  if (adc_channel_num < 8) {
    *my_ADMUX |= adc_channel_num;
  }
  else if ((adc_channel_num > 7) && (adc_channel_num < 16)) {
     ch = (adc_channel_num - 8);
     *my_ADCSRB |= B00001000;
     *my_ADMUX |= ch;
  }

  //start conversion
  *my_ADCSRA |= B01000000;
  
  //conversion complete
  while ((*my_ADCSRA & 0x40) != 0);
  
  //return to ADC
  return (*my_ADC_DATA & 0x03FF);
}
