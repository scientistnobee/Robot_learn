#include <SPI.h>
#include <stdint.h>
#include <BLEPeripheral.h>
//#include "Adafruit_GFX.h"
#include <nrf_nvic.h>//interrupt controller stuff
#include <nrf_sdm.h>
#include <nrf_soc.h>
#include <WInterrupts.h>
//#include <Adafruit_SSD1306.h>
#include <TimeLib.h>
#include <nrf.h>
#include <Wire.h>
//  sd_nvic_SystemReset();

#define wdt_reset() NRF_WDT->RR[0] = WDT_RR_RR_Reload
#define wdt_enable(timeout) \
  NRF_WDT->CONFIG = NRF_WDT->CONFIG = (WDT_CONFIG_HALT_Pause << WDT_CONFIG_HALT_Pos) | ( WDT_CONFIG_SLEEP_Pause << WDT_CONFIG_SLEEP_Pos); \
  NRF_WDT->CRV = (32768*timeout)/1000; \
  NRF_WDT->RREN |= WDT_RREN_RR0_Msk;  \
  NRF_WDT->TASKS_START = 1


//Adafruit_SSD1306 display(128, 32, &SPI, 28, 4, 29);

#define BUTTON_PIN              30
#define BATTERY_PIN             3


#define sleepDelay 10000
#define sleepDelay1 50000
#define refreshRate 100

bool fingerin;
int menu;
byte wert[65];
volatile byte posi = 64;
long startbutton;

BLEPeripheral                   blePeripheral           = BLEPeripheral();
BLEService                      batteryLevelService     = BLEService("180A");
BLEUnsignedLongCharacteristic   batteryLevelChar        = BLEUnsignedLongCharacteristic("2A19", BLERead | BLENotify);
BLEDescriptor                   batteryLevelDescriptor  = BLEDescriptor("2901", "Battery Level 0 - 100%");
BLEUnsignedLongCharacteristic   writeChar        = BLEUnsignedLongCharacteristic("2A15", BLERead | BLEWrite);
BLEDescriptor                   writeDescriptor  = BLEDescriptor("2901", "Write UnixTime");

volatile bool buttonPressed = false;
volatile int16_t level;
volatile uint8_t batt;

unsigned long sleepTime, displayRefreshTime, rpmTime;
volatile bool sleeping = false;

volatile int before1, before2, before3;
volatile float accX, accY, accZ;
volatile char reson;
volatile bool readacc = true, afeoff, foot;
byte sds;

void buttonHandler() {
  if (!sleeping) buttonPressed = true;
  powerUp();
}

void charge() {
  powerUp();
}
void acchandler() {
  readacc = true;
}

void updateBatteryLevel() {
  level = analogRead(BATTERY_PIN);
  batt = map(level, 370, 420, 0, 100);
  batteryLevelChar.setValue(batt);
}

void blePeripheralConnectHandler(BLECentral& central) {
  powerUp();
  before1 = 1;
}

void blePeripheralDisconnectHandler(BLECentral& central) {
  before1 = 0;
}

void characteristicWritten(BLECentral& central, BLECharacteristic& characteristic) {
  unsigned long pctime;
  pctime = writeChar.value();
  if ( pctime >= 1549906231) {
    setTime(pctime);
  } else {
    setTime(1549906231);
  }
}



float reads[4], sum;
long int now1, ptr;
float last, rawRate, start;
float first, second1, third, before, print_value;
bool rising;
int rise_count;
int n;
long int last_beat;
int lastHigh;

void setup() {
  //Serial.begin(115200);
  // pinMode(16, OUTPUT);
  // digitalWrite(16, LOW);
  pinMode(BUTTON_PIN, INPUT);
  if (digitalRead(BUTTON_PIN) == LOW) {
    NRF_POWER->GPREGRET = 0x01;
    sd_nvic_SystemReset();
  }
  wdt_enable(5000);
  blePeripheral.setLocalName("ATCDSD6");
  blePeripheral.setAdvertisedServiceUuid(batteryLevelService.uuid());
  blePeripheral.setAdvertisingInterval(400);
  blePeripheral.setAppearance(0x0000);
  blePeripheral.setConnectable(true);
  blePeripheral.setDeviceName("ATCDSD6");
  blePeripheral.addAttribute(batteryLevelService);
  blePeripheral.addAttribute(batteryLevelChar);
  blePeripheral.addAttribute(batteryLevelDescriptor);
  blePeripheral.addAttribute(writeChar);
  blePeripheral.addAttribute(writeDescriptor);
  writeChar.setEventHandler(BLEWritten, characteristicWritten);
  blePeripheral.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  blePeripheral.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  blePeripheral.begin();
  pinMode(25, OUTPUT);
  digitalWrite(25, HIGH);
  pinMode(26, OUTPUT);
  digitalWrite(26, HIGH);
  pinMode(BATTERY_PIN, INPUT);
  //attachInterrupt(digitalPinToInterrupt(22), acchandler, FALLING);
  attachInterrupt(digitalPinToInterrupt(2), charge, RISING);
  NRF_GPIO->PIN_CNF[2] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[2] |= ((uint32_t)GPIO_PIN_CNF_SENSE_High << GPIO_PIN_CNF_SENSE_Pos);

  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonHandler, FALLING);


  digitalWrite(25, LOW);
  Wire.begin();
  delay(100);
  Pah8001_Configure();

  for (int i = 0; i < 4; i++)reads[i] = 0;
  sum = 0;
  ptr = 0;
}


void disableWakeupByInterrupt(uint32_t pin) {
  detachInterrupt(digitalPinToInterrupt(pin));
  NRF_GPIO->PIN_CNF[pin] &= ~((uint32_t)GPIO_PIN_CNF_SENSE_Msk);
  NRF_GPIO->PIN_CNF[pin] |= ((uint32_t)GPIO_PIN_CNF_SENSE_Disabled << GPIO_PIN_CNF_SENSE_Pos);
}


void loop() {
  blePeripheral.poll();
  wdt_reset();
  if (sleeping) {
    sd_nvic_ClearPendingIRQ(SD_EVT_IRQn);
    sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    sd_app_evt_wait();
  } else {
    if (millis() - displayRefreshTime > refreshRate) {
      displayRefreshTime = millis();
      switch (menu) {
        case 0:
   //       displayMenu0();
          break;
        case 1:
     //     displayMenu1();
          break;
        case 2:
      //    displayMenu2();
          break;
        case 3:
      //    displayMenu3();
          break;
        case 4:
    //      displayMenu4();
          break;
      }
    }

    if (buttonPressed) {
      buttonPressed = false;
      switch (menu) {
        case 0:
          menu = 1;
          break;
        case 1:
          menu = 2;
          break;
        case 2:
          menu = 3;
          break;
        case 3:
          menu = 4;
          break;
        case 4:
          startbutton = millis();
          while (!digitalRead(BUTTON_PIN)) {}
          if (millis() - startbutton > 1000) {
            delay(100);
            int err_code = sd_power_gpregret_set(0x01);
            sd_nvic_SystemReset();
            while (1) {};
            break;
          } else {
            menu = 0;
          }
      }
    }
  }

  uint8_t buffer[13];
  Pah8001_ReadRawData(buffer);
  if (buffer[0]) {
    if (buffer[8] == 128) {
      rawRate = map(buffer[6], 0, 255, 1023, 0);
      fingerin = true;
    } else fingerin = false;
  }
  if (millis() - rpmTime >= 20) {
    rpmTime = millis();

    sum -= reads[ptr];
    sum += rawRate;
    reads[ptr] = rawRate;
    last = sum / 4;

    if (last > before)
    {
      rise_count++;
      if (!rising && rise_count > 5)
      {
        if (!rising)lastHigh = rawRate;
        rising = true;
        first = millis() - last_beat;
        last_beat = millis();
        print_value = 60000. / (0.4 * first + 0.3 * second1 + 0.3 * third);
        third = second1;
        second1 = first;
      }
    }
    else
    {
      rising = false;
      rise_count = 0;
    }
    before = last;

    ptr++;
    ptr %= 4;

  
int v9 = buffer[4];
v9 = buffer[3] | (v9 << 8);
v9 = buffer[2] | (v9 << 8);
v9 = buffer[1] | (v9 << 8);
Serial.println(v9);
    
    /*Serial.print((int)print_value);
      Serial.print(",");
      Serial.print(rawRate);
      Serial.println();*/
  }
}

void PrintHex8(uint32_t *data, uint8_t length) // prints 8-bit data in hex with leading zeroes
{
  char tmp[16];
  for (int i = 0; i < length; i++) {
    sprintf(tmp, "%08X", data[i]);
 //   display.print(tmp); display.println(" ");
  }
}



void powerUp() {
  if (sleeping) {
    sleeping = false;
    delay(5);
  }
  sleepTime = millis();
}

void powerDown() {
  if (!sleeping) {
    menu = 0;

    sleeping = true;

    NRF_SAADC ->ENABLE = 0; //disable ADC
    NRF_PWM0  ->ENABLE = 0; //disable all pwm instance
    NRF_PWM1  ->ENABLE = 0;
    NRF_PWM2  ->ENABLE = 0;
  }
}

void writeRegister(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(0x6b);
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t addr)
{
  Wire.beginTransmission(0x6b);
  Wire.write(addr);
  Wire.endTransmission();
  Wire.requestFrom(0x6b, 1);
  return Wire.read();
}

static const struct {
  uint8_t reg;
  uint8_t value;
} config[] =
{
  { 0x27u, 0xFFu },
  { 0x28u, 0xFAu },
  { 0x29u, 0x0Au },
  { 0x2Au, 0xC8u },
  { 0x2Bu, 0xA0u },
  { 0x2Cu, 0x8Cu },
  { 0x2Du, 0x64u },
  { 0x42u, 0x20u },
  { 0x48u, 0x00u },
  { 0x4Du, 0x1Au },
  { 0x7Au, 0xB5u },
  { 0x7Fu, 0x01u },
  { 0x07u, 0x48u },
  { 0x23u, 0x40u },
  { 0x26u, 0x0Fu },
  { 0x2Eu, 0x48u },
  { 0x38u, 0xEAu },
  { 0x42u, 0xA4u },
  { 0x43u, 0x41u },
  { 0x44u, 0x41u },
  { 0x45u, 0x24u },
  { 0x46u, 0xC0u },
  { 0x52u, 0x32u },
  { 0x53u, 0x28u },
  { 0x56u, 0x60u },
  { 0x57u, 0x28u },
  { 0x6Du, 0x02u },
  { 0x0Fu, 0xC8u },
  { 0x7Fu, 0x00u },
  { 0x5Du, 0x81u }
};

void Pah8001_Configure()
{
  uint8_t value;
  writeRegister(0x06, 0x82);
  delay(10);
  writeRegister(0x09,0x5A);
  writeRegister(0x05,0x99);
  value = readRegister (0x17);
  writeRegister(0x17, value | 0x80 );
  for (size_t i = 0; i < sizeof(config) / sizeof(config[0]); i++)
  {
    writeRegister(config[i].reg, config[i].value);
  }
  value = readRegister(0x00);
}


void Pah8001_ReadRawData(uint8_t buffer[9])
{
  uint8_t tmp[4];
  uint8_t value;
  writeRegister(0x7F, 0x01);

  buffer[0] = readRegister(0x68) & 0xF;

  if (buffer[0] != 0) //0 means no data, 1~15 mean have data
  {
    Wire.beginTransmission(0x6b);
    Wire.write(0x64);
    Wire.endTransmission();
    Wire.requestFrom(0x6b, 4);

    buffer[1] = Wire.read() & 0xFF;
    buffer[2] = Wire.read() & 0xFF;
    buffer[3] = Wire.read() & 0xFF;
    buffer[4] = Wire.read() & 0xFF;

    Wire.beginTransmission(0x6b);
    Wire.write(0x1a);
    Wire.endTransmission();
    Wire.requestFrom(0x6b, 4);

    buffer[5] = Wire.read() & 0xFF;
    buffer[6] = Wire.read() & 0xFF;
    buffer[7] = Wire.read() & 0xFF;

    writeRegister(0x7F, 0x00);
    buffer[8] = readRegister(0x59) & 0x80;
  }
  else
  {
    writeRegister(0x7F, 0x00);
  }
}

void Pah8001_PowerOff(void)
{
  writeRegister(0x7F, 0x00);
  writeRegister(0x06, 0x0A);
}

void Pah8001_PowerOn(void)
{
  writeRegister(0x7F, 0x00);
  writeRegister(0x06, 0x02);
  writeRegister(0x05, 0x99);
}
