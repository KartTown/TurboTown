#include <Wire.h>
#include <Ethernet.h>

#define DEBUG_MODE 0

#define EN_MEDIA 1

#define MAX_COM_LEN 110
#define MAX_MEDIA_LEN 10

#define PIN_START 36

#define PIN_LED_RED 2
#define PIN_LED_GREEN 15

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

Device server = {
  80,  
  {192, 168, 2, 10},                      // Network Address
  {192, 168, 2, 1},                       // Gateway Address
  {255, 255, 0, 0},                       // Subnet Mask
  {0x90, 0xA2, 0xDA, 0x0D, 0x2D, 0x3F},   // MAC Address
};
#if EN_MEDIA == 1
Device media = {
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

EthernetClient client;                           

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
 
  Ethernet.begin(server.MAC, server.IP, server.gateway, server.subnet);          

  #if EN_MEDIA == 1
  if(client.connect(media.IP,media.port)){
    
  }      
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

#if EN_MEDIA == 1
void MEDIA_ACTION()
{
  //If connected, take appropriate action
  if(client.connected()){
    if(file[0] != '0'){
      sprintf(buffer,command,file);
      client.println(buffer);
      file[0] = '0';
    }
  } 
  //Try to reconnect
  else{
    //Reconnect if connection is lost
    if(client.connect(media.IP, media.port)) {
    } 
    else{
    }
  }  
}
#endif