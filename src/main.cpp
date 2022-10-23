
#include <Arduino.h>
#include "Q2HX711.h"
#include <main.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiSettings.h>
#include <PubSubClient.h>
#include "ArduinoNvs.h"
#include <ArduinoOTA.h>
#include <movingAvgFloat.h>


/*Hardware Pinouts:
 * AirBlowerSwitch gpio36
 * Spare Input gpio39
 * Influent Overflow  gpio34
 * Effluent Overflow gpio35
 * Reactor Overflow gpio32
 * Equaliser Tank Overflow gpio33
 * Influent Pump output gpio25
 * Flow Equaliser Level Sensor Tx gpio26
 * Spare Input2 gpio27
 * Was Solenoid OutPut gpio14
 * Decant Solenoid Ouptut gpio12
 * Blower Output GPIO13
 * Effluent Level Sensor GPIO23
 * scl IIC GPIO22
 * Spare2 Ouput GPIO1 also serialTX
 * sda IIC GPIO21
 * Effluent Level Sensor SCK GPIO19
 * Influent Level Sensor sck GPIO18
 * Flow Equaliser Level Sensor SCK GPIO5
 * Influent Level Sensor tx GPIO17
 * Reactor Level Sensor TX GPIO16
 * Mixer Output GPIO4
 * Spare 1 Output GPIO0
 * Pressure Pump Output GPIO2
 * Reactor Level Sensor SCK GPIO15
 */
static uint8_t AirBlowerSwitch=36;
static uint8_t SpareInput  =39;
static uint8_t InfluentOverflow =34;
static uint8_t EffluentOverflow =35;
static uint8_t ReactorOverflow =32;
static uint8_t EqualiserTankOverflow =33;
static uint8_t InfluentPumpOutput =25;
static uint8_t FlowEqualiserLevelTx =26;
static uint8_t SpareInput2 =27;
static uint8_t WasSolenoid =13; //this is swapped with what is on pcb to allow 12v connection of air solenoid
static uint8_t DecantSolenoid =12;
static uint8_t BlowerOutput =14; // this is swappted with WAS solenoid for 12v air connection
static uint8_t EffluentLevelTx =23;
static uint8_t sclIIC =22;
static uint8_t Spare2Output = 1;// this one has cut track
static uint8_t sdaIIC =21;
static uint8_t EffluentLevelSck =19;
static uint8_t InfluentLevelTx =18;
static uint8_t FlowEqualiserLevelSck =5;
static uint8_t InfluentLevelSck =17;
static uint8_t ReactorLeveltx =16;
static uint8_t Mixer =4;
static uint8_t SpareOutput1 =0;
static uint8_t PressurePumpOutput =2;
static uint8_t ReactorLevelSck =15;
static uint8_t off=0;
static uint8_t on=1;
static uint8_t Auto=2;
 float settleStartLevel;
 float InfluentPitStartLevel;
 float InfluentPitStopLevel;
 float DecantStopLevel;
 float effluentStopLevel;
unsigned long decantIdleTime,decantStopTime,decantStartTime,cycle1Timer,cycle2Timer,cycle3Timer,cycle4Timer,influentPumpTimer,WasTimer,WasTimePeriod;
unsigned long timer,lastReconnectAttempt,influentEqTimer;
ArduinoNvs nvss;
#define influentEqDelay 20*60*1000   //delay time of influent pump in equalistoin mode.
#define influentEqRunTime 30*1000   //run time of influent pump in eq mode.
#define influentPitEmergencyPumpAmount 200.0  //amount from the top of the influent pit to pump out in emergency mode
  String host = WiFiSettings.string("MQTT Server", "10.0.0.215");
  int    port = WiFiSettings.integer("MQTT Port", 0, 65535, 1883);
  boolean highLevelInfluentPump;
 
 int output,influentState,y;
void influentEq();
void printInputs();
void wirelessConnect();
void readReactorLevel();
void readEqualisationLevel();
void readInfluentLevel();
void readEffluentLevel();
stateMachine MachineCycle;
  decanta Decanter;
 Q2HX711 ReactorLevel(ReactorLeveltx,ReactorLevelSck); 
 Q2HX711 InfluentLevel(InfluentLevelTx,InfluentLevelSck); 
 Q2HX711 EqualisationTankLevel(FlowEqualiserLevelTx,FlowEqualiserLevelSck);
 Q2HX711 EffluentLevel(EffluentLevelTx,EffluentLevelSck);


uint8_t pinsout[7]={BlowerOutput,InfluentPumpOutput,PressurePumpOutput,Spare2Output,SpareOutput1,Mixer,WasSolenoid};
struct {
    uint8_t Blower;
    uint8_t InfluentPump;
    uint8_t PressurePump;
    uint8_t Spare2;
    uint8_t Spare1;
    uint8_t Mixer;
    uint8_t Decant;
    uint8_t Was;
}outbuffer;

struct {
    uint8_t Blower;
    uint8_t InfluentPump;
    uint8_t PressurePump;
    uint8_t Spare2;
    uint8_t Spare1;
    uint8_t Mixer;
    uint8_t Decant;
    uint8_t Was;
}outstate;  //used for auto manual. 0 is off, 1 is on 2 is auto.
void influentPump();
movingAvgFloat reactorAvg(10);    // use 10 data points for the moving average
movingAvgFloat influentAvg(10);
movingAvgFloat equalisationAvg(10);
movingAvgFloat effluentAvg(10);

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);


void readnvss(void){
  decantIdleTime=(unsigned long)(nvss.getFloat("dctIdleTime")*60000.0);
  decantStartTime=(unsigned long)(nvss.getFloat("dctStartTime")*60000.0);
    //decantStartTime=1*60*1000; //1 min start decant
    decantStopTime=(unsigned long)(nvss.getFloat("dctStopTime")*60000.0);
   //decantStopTime=1*60*1000;  //1 min stop decant
   cycle1Timer=(unsigned long)(nvss.getFloat("cycle1Timer")*60000.0);
    //cycle1Timer=1*60*1000;  //2 hours of aeration
    cycle2Timer=(unsigned long)(nvss.getFloat("cycle2Timer")*60000.0);
    //cycle2Timer=1*60*1000;  //45mins in anoxic
     cycle3Timer=(unsigned long)(nvss.getFloat("cycle3Timer")*60000.0);
   // cycle3Timer=1*60*1000; //10mins for 2nd aeration//in prep for decant cycle...
    cycle4Timer=(unsigned long)(nvss.getFloat("cycle4Timer")*60000.0);
   // cycle4Timer=1*60*1000; //settle timer
    WasTimePeriod=(unsigned long)(nvss.getFloat("wasTimePeriod")*60000.0);
   // WasTimePeriod=30*1000;
    ReactorLevel.span=nvss.getFloat("reactorSpan"); //0.000347486
    ReactorLevel.zero=(unsigned long)(nvss.getFloat("reactorZero")*1000.0);
  //ReactorLevel.zero=9541000;//9541000
  ReactorLevel.setGain(128);
  ReactorLevel.offset=nvss.getFloat("reactorOffset"); //0.000347486
  //ReactorLevel.offset=1000.0;


 InfluentLevel.span=nvss.getFloat("influentSpan");
 //InfluentLevel.span=0.000347486;
  InfluentLevel.zero=(unsigned long)(nvss.getFloat("influentZero")*1000.0);
 // InfluentLevel.zero=9541000;
  InfluentLevel.setGain(128);
   InfluentLevel.offset=nvss.getFloat("influentOffset");
  //InfluentLevel.offset=0;


  EqualisationTankLevel.span=nvss.getFloat("equalSpan");
  //EqualisationTankLevel.span=0.000347486;
  EqualisationTankLevel.zero=(unsigned long) (nvss.getFloat("equalZero")*1000.0);
 // EqualisationTankLevel.zero=9541000;
 EqualisationTankLevel.setGain(128);
 EqualisationTankLevel.offset=nvss.getFloat("equalOffset");
 //EqualisationTankLevel.offset=0;

  EffluentLevel.span=nvss.getFloat("effluentSpan");
  //EffluentLevel.span=60.000347486;
  EffluentLevel.zero=(unsigned long) (nvss.getFloat("effluentZero")*1000.0);
  //EffluentLevel.zero=9541000;
  EffluentLevel.setGain(128);
  EffluentLevel.offset=nvss.getFloat("effluentOffset" );
  //EffluentLevel.offset=0;

  settleStartLevel=nvss.getFloat("stleStLvl");
  InfluentPitStartLevel=nvss.getFloat("InflStrtLvl");
  InfluentPitStopLevel=nvss.getFloat("InflStpLvl");
  DecantStopLevel=nvss.getFloat("DctStpLvl");
  effluentStopLevel=nvss.getFloat("effStpLvl");
}

void callback(char* topic, byte *payload, unsigned int length){
  //Serial.println("------new message from broker -------");
   String messageTemp;
   int b;
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
  

  if (String(topic) == "greywater/Decant/idleTime") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("dctIdleTime", f, true);
    readnvss();
  }
   if (String(topic) == "greywater/Decant/startTime") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("dctStartTime", f, true);
    readnvss();
  }

  if (String(topic) == "greywater/Decant/stopTime") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("dctStopTime", f, true);
    readnvss();
  }
  if (String(topic) == "greywater/CycleTimes/cycle1Timer") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("cycle1Timer", f, true);
    readnvss();
  }
  if (String(topic) == "greywater/CycleTimes/cycle2Timer") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("cycle2Timer", f, true);
    readnvss();
  }
  if (String(topic) == "greywater/CycleTimes/cycle3Timer") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("cycle3Timer", f, true);
    readnvss();
  }
  if (String(topic) == "greywater/CycleTimes/cycle4Timer") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("cycle4Timer", f, true);
    readnvss();
  }
  if (String(topic) == "greywater/wasTime") {
   float f;
    f=messageTemp.toFloat();
     nvss.setFloat("wasTimePeriod", f, true);
    readnvss();
  }
   if (String(topic) == "greywater/reactor/leveltx/span") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("reactorSpan", f, true);
    readnvss();
  } //end topic

  if (String(topic) == "greywater/reactor/leveltx/zero") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("reactorZero", f, true);
   readnvss();
  } //end topic

  if (String(topic) == "greywater/reactor/leveltx/offset") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("reactorOffset", f, true);
   readnvss();
  } //end topic

   if (String(topic) == "greywater/effluent/leveltx/span") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("effluentSpan", f, true);
    readnvss();
  } //end topic

  if (String(topic) == "greywater/effluent/leveltx/zero") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("effluentZero", f, true);
    readnvss();
  } //end topic

  if (String(topic) == "greywater/effluent/leveltx/offset") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("effluentOffset", f, true);
    readnvss(); 
  } //end topic
    if (String(topic) == "greywater/influent/leveltx/span") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("influentSpan", f, true);
    readnvss();
  } //end topic

  if (String(topic) == "greywater/influent/leveltx/zero") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("influentZero", f, true);
  readnvss();
  } //end topic

  if (String(topic) == "greywater/influent/leveltx/offset") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("influentOffset", f, true);
    readnvss(); 
  } //end topic

    if (String(topic) == "greywater/equalisation/leveltx/span") { 
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("equalSpan", f, true);
    readnvss();
  } //end topic

  if (String(topic) == "greywater/equalisation/leveltx/zero") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("equalZero", f, true);
    readnvss();
  } //end topic

  if (String(topic) == "greywater/equalisation/leveltx/offset") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("equalOffset", f, true);
  readnvss();
  } //end topic

  if (String(topic) == "greywater/decantState") {
  
    b=messageTemp.toInt();
    outstate.Decant=b;
  } //end topic

  if (String(topic) == "greywater/wasState") {
  
    b=messageTemp.toInt();
    outstate.Was=b;
  } //end topic

  if (String(topic) == "greywater/influentState") {
    b=messageTemp.toInt();
    outstate.InfluentPump=b;
  } //end topic

  if (String(topic) == "greywater/pressurePumpState") {
  
    b=messageTemp.toInt();
    outstate.PressurePump=b;
  } //end topic

   if (String(topic) == "greywater/spare1State") {
    b=messageTemp.toInt();
    outstate.Spare1=b;
  } //end topic

   if (String(topic) == "greywater/spare2State") {
    b=messageTemp.toInt();
    outstate.Spare2=b;
  } //end topic

   if (String(topic) == "greywater/mixerState") {
    b=messageTemp.toInt();
    outstate.Mixer=b;
  } //end topic

   if (String(topic) == "greywater/blowerState") {
    b=messageTemp.toInt();
    outstate.Blower=b;
  } //end topic

   if (String(topic) == "greywater/reactor/settleStartLevel") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("stleStLvl", f, true);
    readnvss(); 
  } //end topic

 

   if (String(topic) == "greywater/influent/startLevel") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("InflStrtLvl", f, true);
    readnvss(); 
  } //end topic

   if (String(topic) == "greywater/influent/stopLevel") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("InflStpLvl", f, true);
    readnvss(); 
  } //end topic

  if (String(topic) == "greywater/reactor/settleStopLevel") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("DctStpLvl", f, true);
    readnvss(); 
  } //end topic

   if (String(topic) == "greywater/effluent/stopLevel") {
    float f;
    f=messageTemp.toFloat();
    nvss.setFloat("effStpLvl", f, true);
    readnvss(); 
  } //end topic

   if (String(topic) == "greywater/reset") {
    if (messageTemp==String("reset")) ESP.restart();
  } //end topi
}




void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);

    //delay(10000);
    NVS.begin();
    nvss.begin("upstore"); 
    pinMode(AirBlowerSwitch, INPUT);
    pinMode(SpareInput,INPUT);
    pinMode(SpareInput2,INPUT);
    pinMode(InfluentOverflow,INPUT);
    pinMode(EffluentOverflow,INPUT);
    pinMode(ReactorOverflow,INPUT_PULLUP);
    pinMode(EqualiserTankOverflow,INPUT);
    pinMode(BlowerOutput,OUTPUT);
    pinMode(InfluentPumpOutput,OUTPUT);
    pinMode(PressurePumpOutput,OUTPUT);
    pinMode(SpareOutput1,OUTPUT);
    pinMode(Spare2Output,OUTPUT);
    pinMode(Mixer,OUTPUT);
    pinMode(WasSolenoid,OUTPUT);
    pinMode(DecantSolenoid,OUTPUT);
    timer=millis();

    //float fl = 0.000347486;    
    //nvss.setFloat("decantIdleTime",5.0,true);
           
    //res = nvss.setFloat("reactorSpan", fl, true);
   
    //decantIdleTime=1*60*1000; //no longer than 5 mins for decant
    readnvss();
    
     
   MachineCycle.cycle=1; //aertion mode.
   MachineCycle.timer=millis()+cycle1Timer;
   WasTimer=millis()+WasTimePeriod;
   Decanter.state=0;
  
  String greeting =  "\n";
  printf("%s", greeting.c_str());

  SPIFFS.begin(true);  // On first run, will format after failing to mount
  //To reset stored credentials, run SPIFFS.format()
  //SPIFFS.format();
  // Use stored credentials to connect to your WiFi access point.
  // If no credentials are stored or if the access point is out of reach,
  // an access point will be started with a captive portal to configure WiFi.

   WiFiSettings.connect(true, 30);
 lastReconnectAttempt = 0;
  outstate.Blower=Auto;
  outstate.Decant=Auto;
  outstate.InfluentPump=Auto;
  outstate.Mixer=Auto;
  outstate.PressurePump=Auto;
  outstate.Spare1=Auto;
  outstate.Spare2=Auto;
  outstate.Was=Auto;
 

reactorAvg.begin();
influentAvg.begin();
equalisationAvg.begin();
effluentAvg.begin();

for (int i=0;i<10;i++) readReactorLevel();
for (int i=0;i<10;i++) readInfluentLevel();
for (int i=0;i<10;i++) readEqualisationLevel();
for (int i=0;i<10;i++) readEffluentLevel();
     // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}


boolean reconnect() {
  if (mqttClient.connect("arduinoClient")) {

    mqttClient.subscribe("greywater/Decant/idleTime");
  mqttClient.subscribe("greywater/Decant/startTime");
  mqttClient.subscribe("greywater/Decant/stopTime");
  mqttClient.subscribe("greywater/CycleTimes/cycle1Timer");
   mqttClient.subscribe("greywater/CycleTimes/cycle2Timer");
    mqttClient.subscribe("greywater/CycleTimes/cycle3Timer");
     mqttClient.subscribe("greywater/CycleTimes/cycle4Timer");
      mqttClient.subscribe("greywater/wasTime");
      mqttClient.subscribe("greywater/reactor/leveltx/span");
      mqttClient.subscribe("greywater/reactor/leveltx/zero");
      mqttClient.subscribe("greywater/reactor/leveltx/offset");
      mqttClient.subscribe("greywater/influent/leveltx/span");
      mqttClient.subscribe("greywater/influent/leveltx/zero");
      mqttClient.subscribe("greywater/influent/leveltx/offset");
      mqttClient.subscribe("greywater/effluent/leveltx/span");
      mqttClient.subscribe("greywater/effluent/leveltx/zero");
      mqttClient.subscribe("greywater/effluent/leveltx/offset");
      mqttClient.subscribe("greywater/equalisation/leveltx/span");
      mqttClient.subscribe("greywater/equalisation/leveltx/zero");
      mqttClient.subscribe("greywater/equalisation/leveltx/offset");  
      mqttClient.subscribe("greywater/decantState");   
        mqttClient.subscribe("greywater/wasState"); 
        mqttClient.subscribe("greywater/blowerState"); 
          mqttClient.subscribe("greywater/mixerState"); 
            mqttClient.subscribe("greywater/influentState"); 
              mqttClient.subscribe("greywater/pressurePumpState"); 
                mqttClient.subscribe("greywater/spare1State"); 
                  mqttClient.subscribe("greywater/spare2State"); 
    mqttClient.subscribe("greywater/reactor/settleStartLevel");
    mqttClient.subscribe("greywater/influent/startLevel");
    mqttClient.subscribe("greywater/influent/stopLevel");
    mqttClient.subscribe("greywater/reactor/settleStopLevel");
    mqttClient.subscribe("greywater/effluent/stopLevel");
    mqttClient.subscribe("greywater/reset");

  }
  return mqttClient.connected();
}

void wirelessConnect(){
 
  
mqttClient.setServer(host.c_str(), port);
  mqttClient.setCallback(callback);
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (reconnect()) {
        lastReconnectAttempt = 0;
      }
    }

 }else {

 mqttClient.loop();
  

}
}


void InfluentPump(){
 
  switch (influentState){
    case 0:  //turn on pump
            digitalWrite(InfluentPumpOutput,on);
            influentPumpTimer=millis();
            influentState++;
           break;
    case 1: //wait for time to turn off pump
          if (millis()>influentPumpTimer+5000) influentState++;
          break;
    case 2: //turn off pump
          digitalWrite(InfluentPumpOutput,off);
          if (millis()>influentPumpTimer+7000) influentState++; //wait 2 seconds
          break;
    case 3: digitalWrite(InfluentPumpOutput,on);
          influentState=4;
          break;

    case 4:; //do nothing.

    default:
          break;
 
  }

}
void loop() {
wirelessConnect();//
 ArduinoOTA.handle();

//Lets check for high level alarm on influent
if (digitalRead(InfluentOverflow)==0) {
  if (MachineCycle.cycle==1) MachineCycle.cycle=4; //put straight into settle mode if in aeration mode.
  if (MachineCycle.cycle==2) MachineCycle.cycle=3; //put back into short aeration mode and then settle.
};


//Lets check for high level alam on tank
if (digitalRead(ReactorOverflow)==1) {
  if (MachineCycle.cycle==1) MachineCycle.cycle=4; //put straight into settle mode if in aeration mode.
  if (MachineCycle.cycle==2) {MachineCycle.cycle=3; //put back into short aeration mode and then settle.
                                        MachineCycle.timer=millis()+cycle3Timer;
          }
};
readReactorLevel();


        if (ReactorLevel.level>settleStartLevel) {                    // see if reactor vessel level is high enough to start settle/decant op.
          if (MachineCycle.cycle==1) MachineCycle.cycle=4; //put straight into settle mode if in aeration mode.
          if (MachineCycle.cycle==2) {MachineCycle.cycle=3; //put back into short aeration mode and then settle.
                                        MachineCycle.timer=millis()+cycle3Timer;
          }
        };


//Lets check level tx of influent
    readInfluentLevel();
           

//this part just runs if we are maxing out the pit
        if (InfluentLevel.level>InfluentPitStartLevel) {  //used if pit becomes too full
          if ((MachineCycle.cycle>=1) && (MachineCycle.cycle<4)) outbuffer.InfluentPump=1;
            highLevelInfluentPump=true; //a flag so we know we are pumping down the pit a little
        };

        if ((highLevelInfluentPump==true)  && (InfluentLevel.level<(InfluentPitStartLevel-influentPitEmergencyPumpAmount))) {
                                                                                                                      outbuffer.InfluentPump=0;
                                                                                                                      highLevelInfluentPump=false;
                                                                                                                      } ;   
        if (InfluentLevel.level<InfluentPitStopLevel) {
          outbuffer.InfluentPump=0;
        };



  //Lets check level tx of effluent
      readEffluentLevel();
           
          if (EffluentLevel.level>(effluentStopLevel+10.0)) outbuffer.PressurePump=1; else if (EffluentLevel.level<effluentStopLevel) outbuffer.PressurePump=0; 
      
       

 //Lets check level tx of flow equlasiation Tanks
 readEqualisationLevel();
            
       

//set ouputs according to current state machine

//are we in decant 
if (MachineCycle.cycle==0){
        outbuffer.Mixer=0;
        outbuffer.Blower=0;
        outbuffer.InfluentPump=0;

      if(Decanter.state==0) {
          outbuffer.Decant=1; //starting decant
          if (millis()>Decanter.timer) {
            Decanter.state++;
            Decanter.timer=millis()+decantIdleTime;
          };
      };

      if(Decanter.state==1) {
        outbuffer.Decant=0;//decant idle time
        if ((millis()>Decanter.timer) || (ReactorLevel.level<DecantStopLevel)) {
          Decanter.state++;
          Decanter.timer=millis()+decantStopTime;
          };
        
        };

      if(Decanter.state==2) {
        outbuffer.Decant=1; //stop decant
        if (millis()>Decanter.timer) {
          Decanter.state++;
          Decanter.timer=millis()+decantStartTime;
        };
      };
        
      if(Decanter.state==3) {
        outbuffer.Decant=0;
        MachineCycle.cycle=1;//exit decant.
        MachineCycle.timer=millis()+cycle1Timer; //setup of how long the aeration is.
        WasTimer=millis()+WasTimePeriod;
        Decanter.state=0; //reset
      };
}; //end decant machine cycle



//are we in aeration
if (MachineCycle.cycle==1){
        outbuffer.Mixer=1;
        outbuffer.Blower=1;
        if (millis()<WasTimer){ outbuffer.Was=1;} else {outbuffer.Was=0;}
        //outbuffer.Was=1;
        influentEq();

        if (millis()>MachineCycle.timer){
          MachineCycle.timer=millis()+cycle2Timer;
          MachineCycle.cycle++;
        };
};



//are we in anoxic
if (MachineCycle.cycle==2){
       outbuffer.Mixer=1;
        outbuffer.Blower=0;
        influentEq();
        if (millis()>MachineCycle.timer){
          MachineCycle.timer=millis()+cycle1Timer;
          MachineCycle.cycle=1; //will go back to aerobic cycle now. we rely on level tx to get to next machine state.
          WasTimer=millis()+WasTimePeriod;
        };
};


//are we in 2nd aeration
if (MachineCycle.cycle==3){
     outbuffer.Mixer=1;
        outbuffer.Blower=1;
        outbuffer.InfluentPump=0;
     if (millis()>MachineCycle.timer){
          MachineCycle.timer=millis()+cycle4Timer;
          MachineCycle.cycle++;
        };

};



//are we in settle
if (MachineCycle.cycle==4){
     outbuffer.Mixer=0;
      outbuffer.Blower=0;
      outbuffer.Blower=0;
      outbuffer.InfluentPump=0;
      if (millis()>MachineCycle.timer){
        // no need to set timer as the decant will.
          MachineCycle.cycle=0;//decant now
          Decanter.timer=decantStartTime+millis();
          MachineCycle.timer=millis()+decantStartTime+decantStopTime+decantIdleTime;
        };
};
if (MachineCycle.cycle>4)MachineCycle.cycle=1; //catch put back into aeration mode.


//now move outputs from reg into physical pins.


if (outstate.Blower==Auto)  digitalWrite(BlowerOutput,outbuffer.Blower); else if (outstate.Blower==off) digitalWrite(BlowerOutput,off); else if (outstate.Blower==on) digitalWrite(BlowerOutput,on);
if ((outstate.InfluentPump==Auto) && (outbuffer.InfluentPump==1)) InfluentPump(); 
else if ((outstate.InfluentPump==off) || (outbuffer.InfluentPump==0)){ 
                                                                      digitalWrite(InfluentPumpOutput,off);
                                                                      influentState=0;//reset pump startingstate;
                                                                      } 

    else if (outstate.InfluentPump==1) digitalWrite(InfluentPumpOutput,1);
if (outstate.PressurePump==Auto) digitalWrite(PressurePumpOutput,outbuffer.PressurePump);else if (outstate.PressurePump==off) digitalWrite(PressurePumpOutput,off); else if (outstate.PressurePump==on) digitalWrite(PressurePumpOutput,on);
if (outstate.Spare2==Auto) digitalWrite(Spare2Output,outbuffer.Spare2); else if (outstate.Spare2==off) digitalWrite(Spare2Output,off); else if (outstate.Spare2==on) digitalWrite(Spare2Output,on);

if (outstate.Spare1==Auto) digitalWrite(SpareOutput1,outbuffer.Spare1); else if (outstate.Spare1==off) digitalWrite(SpareOutput1,off); else if (outstate.Spare1==on) digitalWrite(SpareOutput1,on);
if (outstate.Mixer==Auto) digitalWrite(Mixer,outbuffer.Mixer);else if (outstate.Mixer==off) digitalWrite(Mixer,off); else if (outstate.Mixer==on) digitalWrite(Mixer,on);
if (outstate.Decant==Auto) digitalWrite(DecantSolenoid,outbuffer.Decant); else if (outstate.Decant==off) digitalWrite(DecantSolenoid,off); else if (outstate.Decant==on) digitalWrite(DecantSolenoid,on);
if (outstate.Was==Auto) digitalWrite(WasSolenoid,outbuffer.Was); else if (outstate.Was==off) digitalWrite(WasSolenoid,off); else if (outstate.Was==on) digitalWrite(WasSolenoid,on);


//report alarms and mqtt

  char reading[120];
  switch (y)
  {
  case 0:{
        unsigned long timer = (MachineCycle.timer-millis())/1000;
        int cycleTimer=(int)timer;
        snprintf(reading, sizeof(reading), "%d", cycleTimer);
        mqttClient.publish("greywater/cycleTimer",reading, false);
        y=25;//change this to suit number of publish topics.
  }
    break;
  
  case 1:
      snprintf(reading,sizeof(reading), "%f", InfluentLevel.level);
        mqttClient.publish("greywater/influentLevel",reading,false);
        y--;
  
  break;

  case 2:
      snprintf(reading,sizeof(reading), "%f", ReactorLevel.level);
        mqttClient.publish("greywater/reactorLevel",reading,false);
        y--;
  break;

  case 3:
        snprintf(reading,sizeof(reading), "%f", EffluentLevel.level);
          mqttClient.publish("greywater/effluentLevel",reading,false);
          y--;
  break;

  case 4:
            snprintf(reading, sizeof(reading), "%d", MachineCycle.cycle);
          mqttClient.publish("greywater/cycleState",reading, false);
          y--;
  break;

  case 5:
        snprintf(reading, sizeof(reading), "%d",outbuffer.Blower);
          mqttClient.publish("greywater/Blower",reading, false);
          y--;
  break;

  case 6:
        snprintf(reading, sizeof(reading), "%d",outbuffer.InfluentPump);
          mqttClient.publish("greywater/InfluentPump",reading, false);
          y--;
  break;

  case 7:
          snprintf(reading, sizeof(reading), "%d",outbuffer.Mixer);
          mqttClient.publish("greywater/Mixer",reading, false);
          y--;
  break;

  case 8:
          snprintf(reading, sizeof(reading), "%d",outbuffer.Decant);
            mqttClient.publish("greywater/Decant",reading, false);
            y--;
  break;

  case 9:
        snprintf(reading, sizeof(reading), "%d",outbuffer.PressurePump);
          mqttClient.publish("greywater/PressurePump",reading, false);
          y--;
  break;

  case 10:
        snprintf(reading, sizeof(reading), "%d",outbuffer.Spare1);
          mqttClient.publish("greywater/Spare1",reading, false);
          y--;
  break;

  case 11:
        snprintf(reading, sizeof(reading), "%d",outbuffer.Spare2);
          mqttClient.publish("greywater/Spare2",reading, false);
          y--;
  break;

  case 12:
        snprintf(reading, sizeof(reading), "%d",outbuffer.Was);
          mqttClient.publish("greywater/WAS",reading, false);
          y--;
  break;

 case 13:
        snprintf(reading, sizeof(reading), "%f",EqualisationTankLevel.level);
          mqttClient.publish("greywater/equalisationLevel",reading, false);
          y--;
  break;
  case 14:
        snprintf(reading, sizeof(reading), "%d",EqualisationTankLevel.raw/1000);
          mqttClient.publish("greywater/equalisationLevelRaw",reading, false);
          y--;
  break;
  case 15:
        snprintf(reading, sizeof(reading), "%d",InfluentLevel.raw/1000);
          mqttClient.publish("greywater/influentLevelRaw",reading, false);
          y--;
  break;
  case 16:
        snprintf(reading, sizeof(reading), "%d",EffluentLevel.raw/1000);
          mqttClient.publish("greywater/effluentLevelRaw",reading, false);
          y--;
  break;
  case 17:
        snprintf(reading, sizeof(reading), "%d",ReactorLevel.raw/1000);
          mqttClient.publish("greywater/reactorLevelRaw",reading, false);
          y--;
  break;

  case 18:
        snprintf(reading, sizeof(reading), "%d",outstate.Blower);
          mqttClient.publish("greywater/blowerStateOut",reading, false);
          y--;
  break;

  case 19:
        snprintf(reading, sizeof(reading), "%d",outstate.Decant);
          mqttClient.publish("greywater/decantStateOut",reading, false);
          y--;
  break;
  
  case 20:
        snprintf(reading, sizeof(reading), "%d",outstate.InfluentPump);
          mqttClient.publish("greywater/influentPumpStateOut",reading, false);
          y--;
  break;

  case 21:
        snprintf(reading, sizeof(reading), "%d",outstate.Mixer);
          mqttClient.publish("greywater/mixerStateOut",reading, false);
          y--;
  break;

  case 22:
        snprintf(reading, sizeof(reading), "%d",outstate.PressurePump);
          mqttClient.publish("greywater/PressurePumpStateOut",reading, false);
          y--;
  break;

  case 23:
        snprintf(reading, sizeof(reading), "%d",outstate.Spare1);
          mqttClient.publish("greywater/spare1StateOut",reading, false);
          y--;
  break;

  case 24:
        snprintf(reading, sizeof(reading), "%d",outstate.Spare2);
          mqttClient.publish("greywater/spare2StateOut",reading, false);
          y--;
  break;

  case 25:
        snprintf(reading, sizeof(reading), "%d",outstate.Was);
          mqttClient.publish("greywater/wasStateOut",reading, false);
          y--;
  break;
  default:
  y=25;
    break;
  }//end switch


}// end loop



void printInputs() {
  Serial.println(digitalRead(ReactorOverflow));
}


void influentEq(){
        if ((millis()>influentEqTimer) && (InfluentLevel.level>InfluentPitStopLevel)) {
          outbuffer.InfluentPump=1;
          }
        if (millis()>(influentEqTimer+influentEqRunTime)) { 
          outbuffer.InfluentPump=0;
           influentEqTimer=millis()+influentEqDelay;
           }
}

void readReactorLevel(){
  //Lets check level tx of reactor
  
        long  raw = ReactorLevel.read();
        ReactorLevel.raw=raw; //save for calibration
        raw=raw-ReactorLevel.zero;
          float f=(float) raw;
          f= f*ReactorLevel.span;
          f=f+ReactorLevel.offset;
          if ((f<1000.0) && (f>650.0)) ReactorLevel.level=reactorAvg.reading(f);
}

void readInfluentLevel(){
  //Lets check level tx of reactor
  
        long  raw = InfluentLevel.read();
        InfluentLevel.raw=raw; //save for calibration
        raw=raw-InfluentLevel.zero;
          float f=(float) raw;
          f= f*InfluentLevel.span;
          f=f+InfluentLevel.offset;
          if ((f<1000.0) && (f>0.0)) InfluentLevel.level=influentAvg.reading(f);
}

void readEqualisationLevel(){
  //Lets check level tx of reactor
   long raw = EqualisationTankLevel.read();
             EqualisationTankLevel.raw=raw;
             raw=raw-EqualisationTankLevel.zero;
          float f=(float)(raw);
           f=  f*EqualisationTankLevel.span;
           f=f+EqualisationTankLevel.offset;
          if ((f<1000.0) && (f>=0.0)) EqualisationTankLevel.level=equalisationAvg.reading(f);
}


void readEffluentLevel(){
  long raw = EffluentLevel.read();
           EffluentLevel.raw=raw;
          raw=raw-EffluentLevel.zero;
           float f=(float) raw;
           f = f*EffluentLevel.span;
           f=f+EffluentLevel.offset;
             if ((f<1000.0) && (f>=0.0)) EffluentLevel.level=effluentAvg.reading(f);
}