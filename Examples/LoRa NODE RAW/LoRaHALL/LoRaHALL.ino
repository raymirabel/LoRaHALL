/**************************************************************************
 @FILE:         LoRaHALL.ino
 @AUTHOR:       Raimundo Alfonso
 @COMPANY:      Ray Ingeniería Electronica, S.L.
 @DESCRIPTION:  Ejemplo de uso para el nodo LoRaHALL y emoncms (https://emoncms.org)
                Example of use for the LoRaHALL node and emoncms (https://emoncms.org)
  
 @LICENCE DETAILS:
  Este sketch está basada en software libre. Tu puedes redistribuir
  y/o modificar esta sketch bajo los términos de licencia GNU.

  Esta programa se distribuye con la esperanza de que sea útil,
  pero SIN NINGUNA GARANTÍA, incluso sin la garantía implícita de
  COMERCIALIZACIÓN O PARA UN PROPÓSITO PARTICULAR.
  Consulte los términos de licencia GNU para más detalles:
                                                                       
  http://www.gnu.org/licenses/gpl-3.0.txt

  This sketch is based on free software. You can redistribute
  and/or modify this library under the terms of the GNU license.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY, even without the implied warranty of
  MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU license terms for more details:   
  
  http://www.gnu.org/licenses/gpl-3.0.txt

 @VERSIONS:
  17-07-2019 - v1.00 : Primera versión
  
**************************************************************************/

#define  FIRMWARE_VERSION "1.00"
#define  HARDWARE_VERSION "190510"

#include <SPI.h>
#include <RHReliableDatagram.h>
#include <RH_RF95.h>
#include "LowPower.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <PinChangeInt.h>

// Defines what sensors are available in the hardware...
//#define DS18B20
//#define LDR10K
//#define PIR

// Structure of default configuration parameters. This parameters are stored in internal eeprom...
typedef struct {
  int     sendTime        = 30;     // [1...9999] Send time delay (minutes)
  byte    rfPower         = 14;     // [5...23]   RF power (5:min...23:max)
  byte    rfRetries       = 3;      // [0...20]   Data send retries (0 = no retries)
  byte    rfNode          = 0;      // [0...250]  Node id
  byte    rfPan           = 100;    // [0...250]  PAN id (Only nodes with this same number are visible)
  boolean dipsw           = true;   // [on|off]   rfNode dipswitch mode:
                                    //            - true:  rfNode = dipswitch + rfPan (the rfNode parameter is ignored)
                                    //            - false: rfNode = rfNode + rfPan 
  boolean led             = true;   // [on|off]   led test mode:
                                    //            - true:  led test mode turn on when the node reads sensors and transmits payload.
                                    //            - false: led test is always off                                    
} stConfig;
stConfig config;


// Identify the node...
//#define DEVICE_ID      5      // 5 = LoRaSDL
//#define DEVICE_ID      6      // 6 = LoRaTH
//#define DEVICE_ID      7      // 7 = LoRaX1
#define DEVICE_ID      8      // 8 = LoRaHALL

// Hardware definitions...
#define SERVER_ADDRESS 250
#define RX        0
#define TX        1
#define RFM95_INT 2
#define HALL_INT  3
#define RFM95_CS  4
#define RFM95_GP  5
#define PIR_PIN   6
#define EN_SENS   8
#define LED       9
#define DIPSW0    A0
#define DIPSW1    A1
#define DIPSW2    A2
#define DIPSW3    A3
#define PIN_DS18B20     A4
#define VBAT      A6
#define LDR_PIN   A7
#define DIR_EEPROM_CFG  10

// Global variable
boolean modoCMD = false;        // flag command line entry
volatile uint16_t pirCount = 0; // Count pir events
  
// Payload structure...  
typedef struct {
  byte        pan_id;
  byte        device_id = DEVICE_ID;
  int         status;         // bit 0:  LDR sensor OK or preset
                              // bit 1:  DS18B20 temperature sensor OK or present       
                              // bit 2:  PIR sensor OK or preset
                              // ...
                              // bit 15: device timeout
  int         rssi;           // x1   - dB
  int         temperature;    // x10  - ºC
  int         reserved;       
  int         hall;           // x1   - on/off
  int         pir;            // x1   - pir event count        
  int         light;          // x1   - %
  int         battery;        // x100 - V
} Payload;
volatile Payload theData;

// RFM95 transceiver configuration and instances...
#define RF95_FREQ 868.0
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHReliableDatagram manager(rf95, 0);

#ifdef DS18B20
  // DS18B20 instances...
  OneWire oneWire(PIN_DS18B20);
  DallasTemperature ds18b20(&oneWire);
#endif

/**************************************************************************
 * SETUP
 *************************************************************************/  
void setup(){
  pinMode(RFM95_GP, INPUT);
  pinMode(EN_SENS,  OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(DIPSW0,INPUT);
  pinMode(DIPSW1,INPUT);  
  pinMode(DIPSW2,INPUT);  
  pinMode(DIPSW3,INPUT);  
  pinMode(HALL_INT, INPUT);
  pinMode(PIR_PIN, INPUT);

  sens_off();
  led_test_off();

  // Check if the eeprom is empty...
  if(EEPROM.read(0) == 123){
    // Read the configuration parameters...
    EEPROM.get(DIR_EEPROM_CFG, config);
  }else{
    // If it is empty, it saves the configuration parameters by default...
    EEPROM.write(0,123);
    EEPROM.put(DIR_EEPROM_CFG, config);
  }

  // Init serial port...
  while (!Serial);
  Serial.begin(9600);

#ifdef DS18B20  
  // Init temperature sensor
  ds18b20.begin();
  ds18b20.setWaitForConversion(true);
  ds18b20.setResolution(12);
#endif

  // Init payload struct...
  theData.pan_id   = config.rfPan;
  theData.hall = 0;
  theData.temperature = 0;
  theData.light = 0;
  theData.pir = 0;
  theData.status = 0; 

  // init RFM95 module...
  radioInit();
  rf95.sleep();
  delay(10);  



  // Check the RX pin to see if you have to enter command line mode...
  if(digitalRead(RX)){  
    modoCMD = true;
    led_commandLine();
    commandLine();
    modoCMD = false;
  }
  led_init();

  // Configure HALL sensor interrupt...
  attachInterrupt(digitalPinToInterrupt(HALL_INT), hall_int, CHANGE);

#ifdef PIR  
  // Configure PIR sensor interrupt...
  attachPinChangeInterrupt(PIR_PIN, pir_int, FALLING);
#endif

  // Sleep 8 seconds before transmitting the first frame after power-up...
  LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
}

/**************************************************************************
 * LOOP
 *************************************************************************/ 
void loop(){
  // Turn on test led. This led shows us how long the node is awake...
  led_test_on();

  // Sensors read...
  lee_sensores();
      
  // Send payload to server...
  send_to_server();

  // Turn of test led...
  led_test_off();
  
  // Sleep x minutes...
  //LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);  
  duerme(config.sendTime);
}



/**************************************************************************
 * FUNCTIONS
 *************************************************************************/ 

void hall_int(void){
  delay(1);
  
  // Turn on test led. This led shows us how long the node is awake...
  led_test_on();

  // Read HALL sensor only...
  theData.hall = !digitalRead(HALL_INT);
        
  // Send payload to server...
  detachInterrupt(digitalPinToInterrupt(HALL_INT));
  interrupts();
  send_to_server();

  // Turn of test led...
  led_test_off();  

  attachInterrupt(digitalPinToInterrupt(HALL_INT), hall_int, CHANGE);
}

void lee_sensores(void){ 
  // Read battery level...
  lee_bateria();

#ifdef LDR10K  
  // Read light level...
  lee_ldr();
#else
  theData.status &= ~0x01;
#endif

#ifdef PIR 
  // Read pir events...
  theData.pir = pirCount;
  theData.status |= 0x04;
#else
  theData.status &= ~0x04;
#endif  

  // Read hall...
  theData.hall = !digitalRead(HALL_INT);
  
#ifdef DS18B20 
  // Read temperature sensor...
  leeDS18B20();
#else
  theData.status &= ~0x02;
#endif  

}

void sens_on(void){
  digitalWrite(EN_SENS, HIGH);
  delay(10);
}

void sens_off(void){
  digitalWrite(EN_SENS, LOW);
}

#ifdef LDR10K
void lee_ldr(void){
  sens_on();
  delay(1);
  theData.light = 100 - (unsigned int)(((unsigned long)(analogRead(LDR_PIN)) * 100L) / 1023L);
  sens_off();
  theData.status |= 0x01;
}
#endif

#ifdef PIR
void pir_int() {
  // Increment variable when PIR sensor event...
  pirCount++;
  
};
#endif

#ifdef DS18B20
void leeDS18B20(void){
  float temperature;

  sens_on();
  ds18b20.requestTemperatures(); 
  temperature = ds18b20.getTempCByIndex(0);

  if((int)temperature >= 85){
    theData.temperature = 0;
    theData.status &= ~0x02;
  }else{
    theData.temperature = (int)(temperature * 10);
    theData.status |= 0x02;
  }
  sens_off();
}
#endif

void lee_bateria(void){
  unsigned long valores = 0;
  byte n;

  delay(1);
  // Lee 5 muestras...
  for(n=0;n<5;n++){
     valores += (unsigned int)(((unsigned long)(analogRead(VBAT)) * 330L) / 1023L);
     delay(1);
  }
  theData.battery = valores / 5;
}






