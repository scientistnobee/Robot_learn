#include <Wire.h>
#include <SPI.h>
//#include <KX022.h>
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library
#include  <avr/dtostrf.h>
#include <Si114.h>
PulsePlug<> pulse(Wire);

#include <BLEPeripheral.h>
#include "BLESerial.h"
BLESerial bleSerial(10, 2, 9);
unsigned long long lastSent = 0;
#define OLED_WIDTH 64
#define OLED_HEIGHT 32
MicroOLED oled(OLED_RST, OLED_DC, OLED_CS); // (pin_rst, pin_dc, pin_cs)

float xyz[3];
uint32_t tPage;
bool B1_isPressed = false;
uint8_t page_num = 0;
const uint8_t page_count = 2;
void draw_page(uint8_t idx = 0);
uint32_t t = millis();
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
const unsigned long period = 1000;  //the value is a number of milliseconds

void setup()
{
  Wire.begin();

 bleSerial.setLocalName("UART");
 bleSerial.begin();
 // Serial.begin(9600);
 // Serial.println(__FILE__);
  pinMode(PIN_BUTTON1, INPUT_PULLUP);
  oled.setScreenSize(OLED_WIDTH, OLED_HEIGHT);
  oled.begin();
  draw_page(page_num++);
  tPage = millis();

    startMillis = millis();  //initial start time
 if (millis() - t < 1000) // 20ms = 50Hz
  {
  yield();
  }

   while (pulse.isPresent() == false)
   {
   //   Serial.println(F("No SI114x foundxxx"));
      //delay(1000);
   }
   
  // Serial.println(F("SI114x Pulse Sensor found"));
   pulse.init();
}

void loop()
{
if (!B1_isPressed & !digitalRead(PIN_BUTTON1)) // timer used for button debounce
  {
    page_num = (page_num + 1 < page_count)?page_num+1:0;
  }
  B1_isPressed = !digitalRead(PIN_BUTTON1);
  
  if (millis() - tPage > 1000) // 20ms = 50Hz
  {
    tPage = millis();
    draw_page(page_num);
  }


 if (currentMillis - startMillis > 2000) // 20ms = 50Hz
  {
  yield();


  oled.clear(PAGE); // Clear the display's internal memory
  oled.clear(ALL);  // Clear the library's display buffer
  }

    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.

 // yield();
 // delay(1000); // show splash for 3s

}




void float2chars(float &in, char (&out)[5])
{
  bool sign_bit = (in < 0);
  uint16_t tmp = sign_bit ? (-in * 10) : (in * 10);
  out[0] = (sign_bit) ? '-' : ' ';
  out[1] = char('0' + (tmp / 10));
  out[2] = '.';
  out[3] = char('0' + (tmp % 10));
  out[4] = '\0';
}

void draw_page(uint8_t idx)
{
  switch(idx)
  {
    case 1:
      page_accelerometer(); break;
    default:
      page_startup();
    break;
  }
}

void page_startup()
{
  oled.clear();
  oled.display();
}



void page_accelerometer()
{
    pulse.readSensor(3); 

   float total=pulse.led_red+pulse.led_ir1+pulse.led_ir2;
    
    char charVal[10]; 
    
    oled.clear();
    oled.setCursor(5, 4); // points cursor to x=27 y=0
    oled.print("Vis");
    oled.setCursor(25, 4); // points cursor to x=27 y=0
    dtostrf(pulse.als_vis,5,0,charVal);
    oled.print(charVal);
    
    oled.setCursor(5, 20); // points cursor to x=27 y=0
    oled.print("IR");
    oled.setCursor(25, 20); // points cursor to x=27 y=0
    dtostrf(pulse.als_ir,5,0,charVal);
    oled.print(charVal);
    oled.display();  


    bleSerial.print("V");
    bleSerial.print(pulse.led_red);
    bleSerial.print(",");
    bleSerial.print("I");
    bleSerial.print(pulse.led_ir1);
}


