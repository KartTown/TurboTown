#ifndef ETH_PHY_MDC
#define ETH_PHY_TYPE ETH_PHY_LAN8720
#if CONFIG_IDF_TARGET_ESP32
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   23

#define ETH_PHY_MDIO  18
#define ETH_PHY_POWER -1
#define ETH_CLK_MODE  ETH_CLOCK_GPIO17_OUT
#elif CONFIG_IDF_TARGET_ESP32P4
#define ETH_PHY_ADDR  0
#define ETH_PHY_MDC   31
#define ETH_PHY_MDIO  52
#define ETH_PHY_POWER 51
#define ETH_CLK_MODE  EMAC_CLK_EXT_IN
#endif
#endif

#include <ETH.h>

#include <WiFiClient.h>
#include <WiFiServer.h>

#include <SPI.h>

#include <Wire.h>

#define DEBUG_MODE 0

#define EN_COM 0
#define EN_LAPS 1
#define EN_MEDIA 1

#define MAX_COM_LEN 110
#define MAX_MEDIA_LEN 10

#define PIN_START 36

#define PIN_LED_RED 2
#define PIN_LED_GREEN 15

#define I2C_SDA 4
#define I2C_SCL 16

#define REV_TIME 10000 
#define RACE_TIME 300000
#define END_TIME 60000

struct Device{
  uint8_t port;
  byte IP[4];
  byte gateway[4];
  byte subnet[4];
  byte MAC[];
};

enum State{
  AD,
  REV,
  RACE,
  END,
};

enum Light{
  OFF,
  RED_ON,
  GREEN_ON,  
};

static State state = State::AD;
static Light light = Light::OFF;

static unsigned long timeRev = 0;
static unsigned long timeFirstRev = 0;

static unsigned long timeRace = 0;
static unsigned long timeFirstRace = 0;

static unsigned long timeEnd = 0;
static unsigned long timeFirstEnd = 0;

static const byte I2C_LAP_ADDRESS = 0x50;

Device serverInfo = {
  80,  
  {192, 168, 2, 10},                      // Network Address
  {192, 168, 2, 1},                       // Gateway Address
  {255, 255, 0, 0},                       // Subnet Mask
  {0x90, 0xA2, 0xDA, 0x0D, 0x2D, 0x3F},   // MAC Address
};
#if EN_MEDIA == 1
Device mediaInfo = {
  80,  
  {192, 168, 2, 11},                      // Network Address
  {192, 168, 2, 1},                       // Gateway Address
  {255, 255, 0, 0},                       // Subnet Mask
  {0x90, 0xA2, 0xDA, 0x0D, 0x2D, 0x3F},   // MAC Address
};
static char file[MAX_MEDIA_LEN] = "ad.mp4"; 
static char command[MAX_COM_LEN] = "{\"jsonrpc\":\"2.0\",\"method\":\"Player.Open\",\"id\":1,\"params\":{\"item\":{\"file\":\"%s\"}}}";
static char buffer[MAX_COM_LEN];
#endif

NetworkServer server(serverInfo.port);                            // Set Server Port
NetworkClient media;                         

void setup()
{ 
  #if DEBUG_MODE == 1
  Serial.begin(9600);
  #endif

  //Set up input pin
  pinMode(PIN_START, INPUT_PULLUP);

  //Set up output pins
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);

  //Set up I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  
  //Set up ethernet
  Network.onEvent(onEvent);

  ETH.begin(
    ETH_PHY_TYPE,
    ETH_PHY_ADDR,
    ETH_PHY_MDC,
    ETH_PHY_MDIO,
    ETH_PHY_POWER,
    ETH_CLK_MODE);
  ETH.config(
    IPAddress(serverInfo.IP),
    IPAddress(serverInfo.subnet));

  server.begin(80);            

  #if EN_MEDIA == 1
  media.connect(mediaInfo.IP,mediaInfo.port);      
  #endif                                                                             
}                                                   

void loop()
{
  CHECK_STATE();
                     
  LED_ACTION();       

  #if EN_MEDIA == 1
  MEDIA_ACTION();
  #endif    
}     

void onEvent(arduino_event_id_t event)
{
  switch(event){
    case ARDUINO_EVENT_ETH_START:
    #if DEBUG_MODE == 1
    Serial.println("[Eth:Start]");
    #endif
    ETH.setHostname("A2");
    break;
    case ARDUINO_EVENT_ETH_CONNECTED:
    #if DEBUG_MODE == 1
    Serial.println("[Eth:Connect]");
    #endif
    break;
    case ARDUINO_EVENT_ETH_GOT_IP:
    #if DEBUG_MODE == 1
    Serial.println("[Eth:IP]");
    #endif
    break;
    case ARDUINO_EVENT_ETH_DISCONNECTED:
    #if DEBUG_MODE == 1
    Serial.println("[Eth:Disconnect]");
    #endif
    break;
    case ARDUINO_EVENT_ETH_STOP:
    #if DEBUG_MODE == 1
    Serial.println("[Eth:Stop]");
    #endif
    break;
    default:
    break;
  }
}

 #if EN_COM == 1
bool isValid(char* command)
{
  int mode = command[2] - '0';
  return (((command[0] == 0x52) || (command[0] == 0x47)) && mode < 3); 
}
#endif

 #if EN_COM == 1
void handleCommand(NetworkClient* client, char* command)
{
  //Byte Guide:
  //0: 0x30
  //1: 0x31
  //2: 0x32
  //R: 0x52
  //G: 0x47
  //[]:0x5B,0x5D
  #if DEBUG_MODE == 1
  Serial.print(command);
  Serial.print(":");
  #endif
  char response[4] = "[0]";

  if(!isValid(command)){
    #if DEBUG_MODE == 1
    Serial.println(response);
    #endif
    response[1] = 0x30;   
    client.println(response);  
    return;    
  }

  int pin = (command[0] == 0x52) ? PIN_LED_RED : PIN_LED_GREEN;
  switch(command[2]){
    case 0x30:
    state = State::OFF;
    break;
    case 0x31:
    state = (command[0] == 0x52) ? State::RED_ON : State::GREEN_ON;
    break;
    case 0x32:
    state = (command[0] == 0x52) ? State::RED_FLASH : State::GREEN_FLASH;
    break;
    default:
    state = State::OFF;
    break;         
  }

  response[1] = 0x31;
  
  #if DEBUG_MODE == 1
  Serial.println(response);
  #endif

  client.println(response);
}
#endif

void CHECK_STATE()
{
  switch(state)
  {
    case State::AD:
    AD_RACE();
    break;
    case State::REV:
    READY_RACE();
    break;
    case State::RACE:
    START_RACE();
    break;
    case State::END:
    END_RACE();
    break;
    default:
    AD_RACE();
    break;
  } 
}

void AD_RACE()
{
  state = (digitalRead(PIN_START)  == LOW)? State::REV : State::AD;    
  
  //Still in ad mode
  if(state != State::REV)
    return;
  
  //Entered rev mode
  timeFirstRev = millis();

  #if EN_MEDIA == 1  
  memcpy(&file[0],"rev.mp4",3);
  #endif
}

void READY_RACE()
{
  timeRev = millis() - timeFirstRev;
  
  //Stay in rev mode until timer runs out
  if(timeRev < REV_TIME) 
    return;

  //Rev time is done, start race
  state = State::RACE;
  timeFirstRace = millis();
  
  #if EN_MEDIA == 1  
  memcpy(&file[0],"race.mp4",3); 
  #endif
}

void START_RACE()
{
  #if EN_LAPS == 1
  LAPS_ACTION();
  #endif

  timeRace = millis() - timeFirstRace;

  //Stay in race mode until timer runs out
  if(timeRace < RACE_TIME)
    return;

  //Entered end mode
  state = State::END;
  timeFirstEnd = millis();
  
  #if EN_MEDIA == 1  
  memcpy(&file[0],"end.mp4",3);
  #endif
}

void END_RACE()
{
  timeEnd = millis() - timeFirstEnd;
  
  //Stay in end mode until timer runs out
  if(timeEnd < END_TIME)
    return;

  //Entered ad mode 
  state = State::AD;  
    
  #if EN_MEDIA == 1  
  memcpy(&file[0],"ad.mp4",3);
  #endif   
}

void LED_ACTION()
{
  switch(light)
  {
    case Light::OFF:
    LED_OFF(PIN_LED_GREEN);
    LED_OFF(PIN_LED_RED);    
    break;
    case Light::RED_ON:
    LED_OFF(PIN_LED_GREEN);
    LED_ON(PIN_LED_RED);    
    break;
    case Light::GREEN_ON:
    LED_OFF(PIN_LED_RED);
    LED_ON(PIN_LED_GREEN);     
    break;
    default:
    LED_OFF(PIN_LED_GREEN);
    LED_OFF(PIN_LED_RED);      
    break;
  }
}

void LED_OFF(int pin)
{
  light = Light::OFF;
  digitalWrite(pin, LOW);
}

void LED_ON(int pin)
{
  light = (pin == PIN_LED_RED) ? Light::RED_ON : Light::GREEN_ON;
  digitalWrite(pin, HIGH);
}

#if EN_LAPS == 1
void LAPS_ACTION(){
  byte receivedData;
  int bytesToRead = 1;
  Wire.requestFrom(I2C_LAP_ADDRESS, bytesToRead);

  if(Wire.available()){
    receivedData = Wire.read();
  }
}
#endif

#if EN_MEDIA == 1
void MEDIA_ACTION()
{
  //If connected, take appropriate action
  if(media.connected()){
    if(file[0] != '0'){
      sprintf(buffer,command,file);
      media.println(buffer);
      file[0] = '0';
    }
  } 
  //Try to reconnect
  else{
    //Reconnect if connection is lost
    if(media.connect(mediaInfo.IP, mediaInfo.port)) {
    } 
    else{
    }
  }  
}
#endif