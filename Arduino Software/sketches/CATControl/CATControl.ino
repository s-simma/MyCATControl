#define PRGVERSION "v1.0"

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Use Arduino IDE: 1.6.13
// copy lib-folder into documents\arduino folder (do not update lib's)
//
// Download WEB-page to MKR1000 (use my TCPSendFile.jar -> enter ip of the MKR1000 -> select html File)
// Download Cleaning Program.txt file to MKR1000 (use my TCPSendFile.jar -> enter ip of the MKR1000 -> select txt File)
// Use Firefox as Browser http://<ip of the MKR1000>:8080
// 
// -------- You have to adjust these values and download sketch afterwards -----------------------
// Date / Time
#define TIMEOFFSET 1                    // Time offset (h) +/- for your time-zone
#define USEDST true                     // Use sommer/wintertime (adjust time in summer +1h)

// WIFI
#define WLAN_SSID "WLANSSID"	        // WLAN SSID
#define WLAN_PASSWD "123456"	        // WLAN Password
#define NET_IP "192.168.1.17"           // 0.0.0.0 for DHCP (can be changed with Browser)
#define NET_SNM "255.255.255.0"
#define NET_GW "192.168.1.1"
#define NET_DNS "0.0.0.0"               // Not used

// MQTT
#define MQTT_SERVER_IP "0.0.0.0"              // MQTT Server IP Adresse
#define MQTT_ID "catgenie"                    // MQTT ID
#define MQTT_PUBLISH "/home/openhab/cat"      // Publish to
#define MQTT_SUBSCRIBE "/home/cat"            // Subscribe from


// ===============================================================================================
// ===============================================================================================
// I2C
#include <Wire.h> 

// Realtime clock / watchdog
#include <DS1388.h>

// WIFI
#include <WiFi101.h>

// MQTT
#include <PubSubClient.h>

// Threads
#include <Thread.h>
#include <ThreadController.h>

// Bounce inputs
#include <Bounce2.h>

//Handle buttons AUTO/START
#include <OneButton.h>   

// CATGenie
#include <CATGenie.h>

// DS1388 object
rtcDS1388 rtc1388;

// EEPROM
extEEPROM eep(kbits_512,1,64,EEP_ADDR);  // device size, number of devices, page size

// Threads
ThreadController threadController = ThreadController();
Thread thread1 = Thread();
Thread thread2 = Thread();

//Handle Long/Short press AUTO/START buttons
OneButton btnAuto(BUTTON_AUTO, true); 
OneButton btnStart(BUTTON_START, true);

//Input CATSensor activated 
OneButton inpCATSensor(CATSENSOR, false);


#define WEBSERVER_PORT 8080               // Port to listen for show WEB Page
#define SAVEWEB_PORT 7002                 // Port to listen (for save WEB Page to EEProm)

// WEB Server
WiFiServer WEBserver(WEBSERVER_PORT);

// WEB Page
WiFiServer EEPServer(SAVEWEB_PORT);      // receive WEB-page over tcp -> EEProm

// MQTT Client
WiFiClient mqttnet;                      // For send data to server
PubSubClient mqttclient(mqttnet);        // MQTT client

// RS485 receive command buffer
int rs485Pointer = 0;
char rs485Buffer[80];

// General variables
boolean doRestart = false;              // Restart MKR1000 (command from WEB-Browser)
uint16_t wbTypeId = 0;                  // command from the WEB-Browser
boolean wbAutoRefresh = false;          // autorefresh from the WEB-Browser
uint8_t restartCounter = 0;             // second Counter for autostart cleaning after power up or restart
volatile boolean runCatGenie = false;   // Run CATGenie Object in Loop (Set in Timer4 Interrupt) 

boolean catSensInp = false;             // CAT sensor input (OneButton) 
boolean catIn = false;                  // CAT is in
unsigned long catInTime = 0;            // CAT In Timer
unsigned long catOutTime = 0;           // CAT Out Timer

unsigned long lastCleaningTime = 0;     // Last cleaning time
boolean cleanLocked = false;            // Start cleaning locked by other CAT Genie
uint8_t cleanLockedTime = 0;            // For reset Clean-Lock after MAXCLEANLOCK
boolean childLock = false;              // Lock panel
uint8_t cleanProgram = 0;               // Actual cleaning program
uint16_t cleanDelay = 0;                // After delay expired start clean (triggered by CATSENSOR)
boolean cleanPending = false;           // Clean Pending
uint8_t inService = 0;                  // 0=Not in service mode, 1=scopper, 2=BOWL....
uint8_t inServiceState = 0;             // last actor state when in service (0,1,2)
uint8_t catError = 0;                   // Error status 1=Maintenence, 2=Water fill problem, 3=Drain problem, 4=Dryer Fuse, 98=unknown error
int progId;
boolean sendMQTTServer = false;
boolean checkDryerFuse = false;
uint8_t lastCleaningState = 0;          // last cleaning state 0=not clening, 1=scooping, 2=washing, 3=drying (for handle power restart)
uint8_t servWatSens = 0;                // Minute counter for service water sensor

// System Configuration stored in EEProm
// If this structure is changed -> delete config area in EEProm (0-1023)
// Use Browser to enter settings again
typedef struct {
  uint16_t lenstruct;                     // !!!! must be the first

  char ssid[32] = WLAN_SSID;              // your network SSID (name)
  char pass[16] = WLAN_PASSWD;            // your network password
  char ip_adr[16] = NET_IP;               // IP
  char subnet[16] = NET_SNM;              // Subnet
  char gateway_ip[16] = NET_GW;           // Gateway
  char dnsserver_ip[16] = NET_DNS;        // DNS Server

  char mqttid[32] = MQTT_ID;                         // ID
  char topic_Publish[32] = MQTT_PUBLISH;             // Publish to
  char topic_Subscribe[32] = MQTT_SUBSCRIBE;         // Subscribe to

  int utcTimeoffs = TIMEOFFSET;
  boolean useDST = USEDST;

  uint16_t catTimeIn = CATINTIME;
  uint16_t cleanDelay = CLEANDELAY;
  uint16_t forceCleaning = FORCECLEAN;

  char mqttserver[16] = MQTT_SERVER_IP;
  uint8_t lockHHfrom = 0;
  uint8_t lockMMfrom = 0;
  uint8_t lockHHto = 0;
  uint8_t lockMMto = 0;

  uint8_t lock2HHfrom = 0;
  uint8_t lock2MMfrom = 0;
  uint8_t lock2HHto = 0;
  uint8_t lock2MMto = 0;

  uint8_t taskl_pow = 0;

  uint8_t prog_man = PROGM;
  uint8_t prog_1 = PROG1;
  uint8_t prog_2 = PROG2;
  uint8_t prog_3 = PROG3;
  uint8_t prog_4 = PROG4;
  uint8_t prog_cat = PROGC;
} confStruct;
confStruct confData[1];


// ----------------------------------------------- SETUP BEGIN ----------------------------------------------------------------
void setup() {

  int i;
  byte ipadr[4];
  
  //Init BUZZER
  pinMode(BUZZER,OUTPUT);
  digitalWrite(BUZZER,HIGH);

  //Initialize serial and wait for port to open
  Serial.begin(9600);
  delay(2000);

  // Init Interface between multiple CATGenie's (RS485)
  // Switch RS485 to read data
  pinMode(RS485RW,OUTPUT);
  digitalWrite(RS485RW,LOW);
  Serial1.begin(9600);
  delay(2000);  

  // Set I2C as a Bus-Master
  Wire.begin();

  // Configure Threads
  thread1.onRun(Monitor_1);
  thread1.setInterval(1000);          // 1 Sekunde
  threadController.add(&thread1);
  thread2.onRun(Monitor_60);
  thread2.setInterval(1*60*1000);     // 1 Minute
  threadController.add(&thread2);

  // Test Time is valid (if not -> set default time)
  if (!rtc1388.isrunning()) {
    Serial.println(F("Realtime is not valid -> set default time to compile time"));  
    rtc1388.adjust(DateTime(__DATE__, __TIME__));
    digitalWrite(BUZZER,LOW);
    delay(2000);  
    digitalWrite(BUZZER,HIGH);
  }

  Serial.println(F("Actual date/time: "));    
  DateTime actDate = rtc1388.now(confData[0].utcTimeoffs, confData[0].useDST);      
  Serial.print(actDate.day(), DEC);
  Serial.print('.');
  Serial.print(actDate.month(), DEC);
  Serial.print('.');
  Serial.print(actDate.year(), DEC);
  Serial.print(' ');
  Serial.print(actDate.hour(), DEC);
  Serial.print(':');
  Serial.print(actDate.minute(), DEC);
  Serial.print(':');
  Serial.print(actDate.second(), DEC);
  Serial.println();  
 
  // Enable watchdog
  Serial.println(F("Enable Watchdog: 30s"));
  rtc1388.startWatchdogTimer(30,0);  
 
  // Enable Trickle charger
  Serial.println(F("Enable Trickle Charger 250R / no Diode"));
  rtc1388.setTrickleCharger(1);  
    
  // Get CATGenie Configuration from EEProm
  i = eep.read(0, (byte*)confData, 2);
  // Do we have a valid configuration
  if ((i == 0) && (confData[0].lenstruct == sizeof(confData))) {
    i = eep.read(0, (byte*)confData, sizeof(confData));
    if (i != 0) {
      Serial.println(F("Error: Could not read configuration from EEProm -> use default ........"));    
    }
  } else {
    Serial.println(F("Error: No valid configuration in EEProm -> use default ........"));        
    confData[0].lenstruct = sizeof(confData);    
    i = eep.write(0, (byte*)confData, sizeof(confData));
    Serial.print(F("Configuration file written size: "));
    Serial.println(sizeof(confData));
  }

  // Initialize CATGenie object (must be done before using CATGenie object)
  myCATGenie.begin();
            

  // Wait 3 seconds for Auto/Start buttons together -> Set default IP and disable server
  unsigned long startWait = millis();
  while (getDiffTime(startWait, millis()) < 3000) {
    if ((digitalRead(BUTTON_AUTO) == LOW) && (digitalRead(BUTTON_START) == LOW)) {
      Serial.println(F("......Erease EEProm............"));  
      myCATGenie.setOutput(LED_ERROR);
      eeErase(64, 0, 64 * 1024 - 1);
      boolean isOn = false;
      while (true) {
        if (isOn) {
          myCATGenie.resetOutput(LED_ERROR);
          isOn = false;                    
        } else {
          myCATGenie.setOutput(LED_ERROR);
          isOn = true;          
        }
        delay(500);
      }      
    }
  }
 
  // link the AUTO-button to the Handler
  btnAuto.setDebounceTicks(50);
  btnAuto.setClickTicks(400);
  btnAuto.attachClick(AutoClick);
  btnAuto.attachDoubleClick(AutoDoubleClick);
  btnAuto.attachLongPressStart(AutoLongPress);

  // link the START-button to the Handler
  btnStart.setDebounceTicks(50);
  btnStart.setClickTicks(400);
  btnStart.attachClick(StartClick);
  btnStart.attachDoubleClick(StartDoubleClick);
  btnStart.attachLongPressStart(StartLongPress);

  // link the START-button to the Handler
  inpCATSensor.setDebounceTicks(20);
  inpCATSensor.attachClick(CatSensorActiv);

  //Control CATLED output for CATSENSOR input
  tc5Configure(69000); //configure the timer to run at <34500*2>Hertz
  tc5StartCounter(); //starts the timer
  
  // Initilize CAT-Sensor input
  pinMode(CATSENSOR, INPUT_PULLUP);

  // Timer4 to handle CATGenie object timers
  tc4Configure(10); //configure the timer to run at 0,1s = 10
  tc4StartCounter(); //starts the timer

  updatePanel(0);

  // Initilize WIFI + MQTT
  myCATGenie.setOutput(LED_ERROR);
  // Give Alarm if WIFI not available
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println(F("!!!!!!!!!!WiFi module not present"));
    digitalWrite(BUZZER,LOW);
    delay(4000);  
    digitalWrite(BUZZER,HIGH);
  }

  // Connect to WIFI
  if (strlen(confData[0].ssid) > 0) {          
    ConnectToWIFI();
  }

  // Keine Übertragung zu MQTT Server
  parseBytes(confData[0].mqttserver, '.', ipadr, 4, 10);
  if (((ipadr[0] > 0) && (ipadr[0] < 255))) {
    mqttclient.setCallback(mqtt_callback);
    mqttclient.setServer(confData[0].mqttserver, 1883);
    ConnectToMQTT();  
  }
  lastCleaningTime = millis();
  
  catError = 99;    // Restart
  myCATGenie.setOutput(LED_ERROR);

  // Get the state where the last cleaning was interrupted by power down (Byte 0 from Timer-EEProm)
  lastCleaningState = rtc1388.EEPROMRead(0);
  
  sendToMQTTServer();

  // Setup complete set BUZZER 0,5s
  digitalWrite(BUZZER,LOW);
  delay(500);  
  digitalWrite(BUZZER,HIGH);

  // During waiting WIFI101 calls doCATGenie()
  WiFi.setWaitCallback(doCATGenie);
  // During waiting Pubsubclient calls doCATGenie()
  mqttclient.setWaitCallback(doCATGenie);
}

// ----------------------------------------------- SETUP END ----------------------------------------------------------------


// ----------------------------------------------- LOOP BEGINN ----------------------------------------------------------------
void loop() {
  
  // Restart requested from WEB-Browser
  if (doRestart == true) {
    doRestart = false;
    Serial.println(F("......Wait for watchdog to restart MKR1000............"));  
    myCATGenie.resetOutputAll();    
    myCATGenie.setOutput(LED_ERROR);
    while (true) {}      
  }
  
  // Trigger Watchdog
  rtc1388.resetWatchdogTimer();  
  
  // Handle commands from RS485
  rs485Handler();
  
  // Handle buttons AUTO and START
  btnAuto.tick();
  btnStart.tick();

  // Save the actual cleaning state for power restart (Byte 0 in Timer-EEProm)
  // 0=not cleaning, 1=scooping, 2=washing, 3=drying
  if (catError != 99) {
    uint8_t isState = myCATGenie.isCleaning();
    if (isState != lastCleaningState) {
      lastCleaningState = isState;
      // write new state to EEProm
      rtc1388.EEPROMWrite(0,lastCleaningState);                  
    }
  }
  
  // Handle CATSensor
  inpCATSensor.tick();  
  if (catSensInp == true) {
    // Restart clean delay if already running
    if (cleanDelay != 0) {
      cleanDelay = confData[0].cleanDelay;
    } else {
      // The first time sets CAT is in
      if (catIn == false) {
        catIn = true;
        catInTime = millis();
        catOutTime = catInTime;
        updatePanel(0);
        if (confData[0].catTimeIn == 0) {
          cleanDelay = confData[0].cleanDelay;      
          sendMQTTServer = true;                          
        }
      } else {
        catOutTime = millis();
        if (getDiffTime(catInTime,catOutTime) >= (confData[0].catTimeIn * 1000)) {
          cleanDelay = confData[0].cleanDelay;      
          updatePanel(0);
          sendMQTTServer = true;                
        }
      }
    }
    catSensInp = false;
  } else if ((catIn == true) && (getDiffTime(catOutTime, millis()) > (CATMONTIME*1000))) {      
    clearCatIn();
    updatePanel(0);
  }

  // Call CatGenie Object as often as possible
  doCATGenie();
  
  // Handle tasks
  threadController.run();

  // Handle WEB Server
  handleWEBServer();

  // Handle receive WEB-page
  receiveEEPData();

  // Update MQTT server
  mqttclient.loop();
  if (sendMQTTServer == true) {
    sendMQTTServer = false;
    sendToMQTTServer();    
  }
}
// ----------------------------------------------- LOOP END ----------------------------------------------------------------



//////////////////////////////////////////////////////////////////////////
//
// Task Monitor (every second)
//
//////////////////////////////////////////////////////////////////////////
void Monitor_1() {
  uint16_t output;
  
  // Monitor DRYERFUSE if Dryer is ON (after 1 second)
  if (myCATGenie.getOutput(DRYER) == HIGH) {
    if ((checkDryerFuse == true) && (digitalRead(DRYERFUSE) != LOW)) {
      setError(4);    
    }
    checkDryerFuse = true;
  } else {
    checkDryerFuse = false;    
  }
    
  // Update actual state of output in service mode
  if (inService > 0) {
    switch (inService) {
      case 1:
        output = DRAIN_PUMP;
        break;
      case 2:
        output = BOWL;
        break;
      case 3:
        output = SCOPPER;
        break;
      case 4:
        output = DRYER;
        break;
      case 5:
        output = WATERON;
        break;
      case 6:
        output = DOSAGE_PUMP;
        break;
      default:
        return;
    }
    updatePanel(output);    
  }
}

//////////////////////////////////////////////////////////////////////////
//
// Task Monitor (every minute)
//
//////////////////////////////////////////////////////////////////////////
void Monitor_60(){

  char charBuf[64];
  byte ipadr[4];
  boolean reconnect = false;
  IPAddress ip;
  
  // Autostart cleaning after power up or restart
  if (catError == 99) {
    if ((lastCleaningState != 0) && (confData[0].taskl_pow != 0)) {
      restartCounter = restartCounter + 1;
      if (restartCounter >= 3) {
        setCleanProg(5);
        cleanDelay = 1;     // Start power on cleaning process
      }
    } else {
      setCleanProg(5);
      resetError();
      Serial.println(F("Cleaning was not interrupted or no tasklist spezified for power up"));      
    }
  }
  
  // Force clean after n hours not started
  if ((cleanProgram == 5) && (confData[0].forceCleaning != 0)) {
    if (getDiffTime(lastCleaningTime, millis()) >= (confData[0].forceCleaning * 3600 * 1000)) {
        if (cleanDelay == 0) {
          cleanDelay = 1;     // Start cleaning process
        }
    }
  }
  
  // Start Clean process after delay (triggered by CATSENSOR)
  if (cleanDelay > 0) {
    cleanDelay = cleanDelay - 1;
    if (cleanDelay == 0) {
      if ((timeLocked() == false) && (cleanLocked == false)) {
        startCleaning();
      } else {
        cleanPending = true;
        updatePanel(0);        
      }
    }    
  } else {
    if ((cleanPending == true) && (cleanLocked == false) && (timeLocked() == false)) {
      startCleaning();
    }
  }
  
  // Reset Clean locked after Max Time
  if (cleanLocked == true) {
    cleanLockedTime = cleanLockedTime + 1;
    if (cleanLockedTime > MAXCLEANLOCK) {
      resetCleanLock();  
    }
  }

  // Monitor Watersensor -> Check if it needs cleaning
  if ((catError != 99) && (myCATGenie.isCleaning() == 0) && (inService == 0)) {        
    if (myCATGenie.getWaterSensor() > SENSSERVICE) {
      if (servWatSens < SENSSERVTIME) {
        servWatSens = servWatSens + 1;
        if (servWatSens == SENSSERVTIME) {
          setError(1);
        }        
      }
    } else {
      servWatSens = 0;
      if (catError == 1) {
        resetError();                      
      }
    }
  }

  // Test WIFI available
  if (strlen(confData[0].ssid) > 0) {          
    // Only if cleaning process not running
    if (myCATGenie.isCleaning() == 0) {      
      if (WiFi.status() != WL_CONNECTED) {
        Serial.println(F("WIFI is not connected!...try to connect"));
        ConnectToWIFI();
      }       

      // Test ping to gateway
      if (WiFi.status() == WL_CONNECTED) {
        ip = WiFi.gatewayIP();
        if ((ip[0] > 0) && (ip[0] < 255)) {
          if (WiFi.ping(ip) < 0) {
            Serial.println(F("No ping to gateway possible!...try to connect"));
            ConnectToWIFI();
          }
        }
              
        // Test MQTT Connection
        parseBytes(confData[0].mqttserver, '.', ipadr, 4, 10);
        if ((ipadr[0] > 0) && (ipadr[0] < 255)) {      
          if (WiFi.status() == WL_CONNECTED) {
            if (!mqttclient.connected()) {
              Serial.println(F("MQTT is not connected!...try to connect"));
              ConnectToMQTT();
            }
          }
        }
      }
    }  
    // Send actual state to MQTT server
    sendMQTTServer = true;
  }
/*
  byte *ptr = (byte*)malloc(5000);
  if (ptr == NULL) {
    Serial.println("malloc failed..........................");
  } else {
    Serial.println("malloc succeeded.......................");
    free(ptr);
  }
*/  
}


///////////////////////////////////////////////////////
//  Run CatGenie Object every 100ms (Timer4)
///////////////////////////////////////////////////////
void doCATGenie() {
  if (runCatGenie == true) {
    runCatGenie = false;

    // Wait for cleaning process finished
    if (myCATGenie.runL() == true) {
      int i = myCATGenie.getError();
      if (i != 0) {    
        myCATGenie.resetOutputAll();    
        switch (i) {
          case 1:
            setError(2);   // Water fill problem
            break;
          case 2:
            setError(3);   // Water discharge problem
            break;
          default:
            setError(9);   // unknown error
            break;      
        }
      }
      setIdleState();
    }
  }
  return;      
}

///////////////////////////////////////////////////////
//  Set CATGenie is idle
///////////////////////////////////////////////////////
void setIdleState() {

  servWatSens = 0;
  clearCatIn();
  cleanPending = false;
  cleanDelay = 0;
  lastCleaningTime = millis();

  // Send "free" to other CATGenie's
  digitalWrite(RS485RW,HIGH);   // Set RS485 write enable
  Serial1.println("status=free");
  Serial1.flush();
  digitalWrite(RS485RW,LOW);   // Set RS485 read enable
  Serial.println(F("Cleaning FINISHED..............."));
  updatePanel(0);
  sendMQTTServer = true;      
}


//////////////////////////////////////////////////////////////////////////
// Determines if we are inside a global time lock for cleaning 
//////////////////////////////////////////////////////////////////////////
boolean timeLocked() {
  uint16_t fromTime;
  uint16_t toTime;
  uint16_t actTime;
  
  DateTime thisDT = rtc1388.now(confData[0].utcTimeoffs, confData[0].useDST);
  actTime = thisDT.hour()*60 + thisDT.minute();

  // Locktime 1
  fromTime = confData[0].lockHHfrom*60 + confData[0].lockMMfrom;
  toTime = confData[0].lockHHto*60 + confData[0].lockMMto;  
  if (fromTime != toTime) {
    if (toTime > fromTime) {
      if ((actTime >= fromTime) && (actTime < toTime)) {
        return true; 
      }
    } else {
      if ((actTime >= fromTime) || (actTime < toTime)) {
        return true; 
      }    
    }
  }
  
  // Locktime 2
  fromTime = confData[0].lock2HHfrom*60 + confData[0].lock2MMfrom;
  toTime = confData[0].lock2HHto*60 + confData[0].lock2MMto;  
  if (fromTime != toTime) {
    if (toTime > fromTime) {
      if ((actTime >= fromTime) && (actTime < toTime)) {
        return true; 
      }
    } else {
      if ((actTime >= fromTime) || (actTime < toTime)) {
        return true; 
      }    
    }
  }
  return false;
}


//////////////////////////////////////////////////////////////////////////
// CAT-Sensor input activated
// (OneButton callback)
//////////////////////////////////////////////////////////////////////////
void CatSensorActiv() {
  // Activate CAT sensor if program 5 and not cleaning
  if ((cleanProgram == 5) && (myCATGenie.isCleaning() == 0)) {
    catSensInp = true;
  }
}

//////////////////////////////////////////////////////////////////////////
void clearCatIn() {
  catIn = false;
  catInTime = 0;    
  catOutTime = 0;
  catSensInp = false;      
}


//////////////////////////////////////////////////////////////////////////
// Interrupt SR: Timer4 Interrupt, Handle Timers in CATGenie object
// this function gets called by the interrupt of Timer4
// every 100ms. CATGenie object itself is called every 100ms
//////////////////////////////////////////////////////////////////////////
void TC4_Handler() {

  NVIC_DisableIRQ(TC4_IRQn);
  interrupts();
  
  //YOUR CODE HERE (next line will be execoted before interrupts are enabled !!!!)
  runCatGenie = true;
    
  // Running the CATGenie object from timer interrupt (Decrement timer values).
  myCATGenie.runI();    // Handle Timer in CATGenie Object
  
  // END OF YOUR CODE
  TC4->COUNT16.INTFLAG.bit.MC0 = 1;         //Reenable Timer4 Interrupt      
  NVIC_EnableIRQ(TC4_IRQn);
}



//////////////////////////////////////////////////////////////////////////
//
// Interrupt SR: Control CATLED (34,5kHZ) pulses by Timer5 Interrupt
// this function gets called by the interrupt of Timer5
// every 349us 6 pulses with 34,5khz
//////////////////////////////////////////////////////////////////////////
void TC5_Handler (void) {
  static boolean state = false;
  static int countPulses = 0;
  static int waitPulses = 0;
  
  //YOUR CODE HERE 
  if (waitPulses == 0) { 
    if(state == false) {
      digitalWrite(CATLED,HIGH);
      state = true;
    } else {
      digitalWrite(CATLED,LOW);
      state = false;
      countPulses = countPulses + 1;
      if (countPulses >= 6) {
        waitPulses = 6;
      }
    }
  } else {
    waitPulses = waitPulses - 1;
    if (waitPulses == 0) {
      state = false;
      countPulses = 0;
    }
  }
  
  // END OF YOUR CODE
  TC5->COUNT16.INTFLAG.bit.MC0 = 1; //don't change this, it's part of the timer code
}



///////////////////////////////////////////////////////
// AUTO button click
///////////////////////////////////////////////////////
void AutoClick(){
  
  if ((childLock == true) || (digitalRead(BUTTON_START) == LOW)) {
    return;
  }
  
  // Reset Error
  if (catError != 0) {
    resetError();
    return;
  }

  // If not in switch to next program
  if (inService == 0) {  
    if ((cleanDelay > 0) || (catIn == true)) {
      clearCatIn();
      cleanDelay = 0;
    } else {
      cleanProgram = cleanProgram + 1;
      if (cleanProgram > 5) {
        cleanProgram = 0;
      }
    }
    setCleanProg(cleanProgram);

  // if in service switch to next output
  } else {
    inService = inService + 1;
    if (inService > 6) {
      inService = 1;    
    }
    inServiceState = 0;
    updatePanel(0);
  }
}


///////////////////////////////////////////////////////
// Auto button Double-Click
///////////////////////////////////////////////////////
void AutoDoubleClick(){
  if (childLock == true) {
    return;
  }
}

///////////////////////////////////////////////////////
// AUTO button long press
///////////////////////////////////////////////////////
void AutoLongPress(){
  if ((inService > 0) && digitalRead(BUTTON_START == HIGH)) {
    // Exit service mode
    resetService();
  } else {  
    if (btnStart.isLongPressed()) {
      // Keylock on/off
      if (childLock == false) {
        setChildLock();
      } else {
        resetChildLock();
      }    
    }
  }
}


///////////////////////////////////////////////////////
// START button click
///////////////////////////////////////////////////////
void StartClick(){
  if ((childLock == true) || (digitalRead(BUTTON_AUTO) == LOW)) {
    return;
  }

  // Reset Error
  if (catError != 0) {
    resetError();
    return;
  }

  // Switch on/off actors in service mode
  if (inService > 0) {
    serviceSwichtOutput();
    return;    
  }

  // Otherwise start the cleaning process if not locked and not running
  if (myCATGenie.isCleaning() == 0) {
    if (cleanLocked == false) {
      if (startCleaning() != 0) {
        digitalWrite(BUZZER,LOW);
        delay(2000);  
        digitalWrite(BUZZER,HIGH);
      }
    } 
  } else {
    // Stop cleaning, move scopper up
    myCATGenie.stopCleaning();    
    myCATGenie.setOutput(SCOPPER,MAXSCOPTIME);
    setIdleState();
  }
}


///////////////////////////////////////////////////////
// START button Double-Click
///////////////////////////////////////////////////////
void StartDoubleClick(){
  if (childLock == true) {
    return;
  }

  // Test enter Service mode
  if (btnAuto.isLongPressed()) {
    // !!!!!!!!    Finish clean cycle if running !!!!!!!!!!!!!!!!
    setService();

  // Unlock CATGenie
  } else if (digitalRead(BUTTON_AUTO) != LOW) {
    if (cleanLocked == false) {
      setCleanLock();              
    } else {
      resetCleanLock();              
    }    
  }
}


///////////////////////////////////////////////////////
// START button long press (Enter service mode if AUTO_BUTTON longpressed else set/reset cleanLocked)
///////////////////////////////////////////////////////
void StartLongPress(){
  if (btnAuto.isLongPressed()) {
    if (childLock == false) {
      setChildLock();
    } else {
      resetChildLock();
    }    
  } else {
    if (inService != 0) {
      myCATGenie.resetOutputAll();    
      myCATGenie.setOutput(LED_1);  
      myCATGenie.setOutput(LED_2);  
      myCATGenie.setOutput(LED_3);  
      myCATGenie.setOutput(LED_4);  
      myCATGenie.setOutput(LED_CAT);    
      myCATGenie.setOutput(LED_LOCKED);    
      myCATGenie.setOutput(LED_CARTRIDGE);    
      myCATGenie.setOutput(LED_ERROR);
      while (1) {}                
    }    
  }
}

///////////////////////////////////////////////////////
// Start Cleaning proceass
///////////////////////////////////////////////////////
int startCleaning() {
  int i;
  uint8_t tasklist;
  
  // Dont't start if in service
  if (inService != 0) {
    Serial.println(F("Error -1: CATGenie is in Service Mode"));
    return -1;
  }
  // Don't start if CAT-Error
  if ((catError > 1) && (catError < 99)) {
    Serial.println(F("Error -2: catError 2 - 98"));
    return -2;
  }
  // Dont't start if already running
  if (myCATGenie.isCleaning() != 0) {
    Serial.println(F("Error -3: Cleaning alreay active"));
    return -3;
  }

  // Set cleaning pending if locked at this time
  if (cleanLocked == true) {
    cleanPending = true;
    Serial.println(F("Error: Cleaning is locked"));
    return 0;
  }    

  tasklist = 99;   // No valid tasklist
  if (catError != 99) {
   // Determine the tasklist to use for the cleaning program
    switch (cleanProgram) {
      case 0:
        tasklist = confData[0].prog_man;
        break;
      case 1:
        tasklist = confData[0].prog_1;
        break;
      case 2:
        tasklist = confData[0].prog_2;
        break;
      case 3:
        tasklist = confData[0].prog_3;
        break;
      case 4:
        tasklist = confData[0].prog_4;
        break;
      case 5:
        tasklist = confData[0].prog_cat;
        break;
    }
  } else {
    // Get the state where the last cleaning was interrupted by power down (Byte 0 from Timer-EEProm)
    switch (lastCleaningState) {
      // Scooping was interrupted
      case 1:
        tasklist = confData[0].prog_cat;
        break;
      // Washing was interrupted
      case 2:
        tasklist = confData[0].taskl_pow;
        break;
      // Drying was interrupted
      case 3:
        tasklist = confData[0].taskl_pow;
        break;
    }
  }
  
  if ((tasklist < 1) || (tasklist > EEP_TASKLISTMAX)) {
      setError(5);
      Serial.println(F("Error -5: Tasklist not 1-max"));
      return -5;      
  }

  // Start the cleaning process
  i = myCATGenie.startCleaning(tasklist);    

  if (i != 0) {    
    switch (i) {
      case 1:
        Serial.println(F("Error: Tasklist-Id > max"));
        setError(6);
        break;
      case 2:
        Serial.println(F("Error: Can't read tasklist from EEProm"));
        setError(6);
        break;
      case 3:
        Serial.println(F("Error: Tasklist data in EEProm not valid"));
        setError(6);
        break;
      default:
        Serial.println(F("Error: myCATGenie.startCleaning(tasklist)"));
        setError(10);
        break;      
    }
    return i;
  }
  
  // Reset Maintenance / power up warning
  resetError();    

  detachInterrupt(CATSENSOR);
  clearCatIn();
  cleanPending = false;
  cleanDelay = 0;
  updatePanel(0);
        
  // Send "lock" to other CATGenie's
  digitalWrite(RS485RW,HIGH);   // Set RS485 write enable
  Serial1.println("status=lock");
  Serial1.flush();
  digitalWrite(RS485RW,LOW);   // Set RS485 read enable

  myCATGenie.setOutput(BUZZER,5);  // 0,5sec
  sendMQTTServer = true;    
  Serial.println(F("Cleaning STARTED..............."));
  return 0;
}


///////////////////////////////////////////////////////
// Set CAT-Error
///////////////////////////////////////////////////////
void setError(int errorNo){

  if ((catError > 1) && (catError < 99)) {
    return;
  }
  
  catError = errorNo;
  myCATGenie.flashOutput(LED_ERROR,5);
  if (errorNo != 1) {     // No Buzzer to show Maintenance Message
    myCATGenie.setOutput(BUZZER,30);   // intervall is 30 seconds
  }
  updatePanel(0);
  
  // Reset all Outputs if Error-Code <= 9
  if ((catError > 1) && (catError <= 9)) {
    cleanPending = false;
    cleanDelay = 0;
    clearCatIn();
    myCATGenie.stopCleaning();    
    myCATGenie.resetOutputAll();    
  }
  sendMQTTServer = true;    
}

///////////////////////////////////////////////////////
// Reset CAT-Error
///////////////////////////////////////////////////////
void resetError() {

  if (catError == 0) {
    return;
  }
    
  myCATGenie.resetOutput(LED_ERROR);
  myCATGenie.resetOutput(BUZZER);
  if ((catError > 1) && (catError <= 9)) {
    cleanPending = false;
    cleanDelay = 0;
    clearCatIn();
  }
  servWatSens = 0;
  catError = 0;
  updatePanel(0);    
  sendMQTTServer = true;    
}


///////////////////////////////////////////////////////
// Set Child Lock
///////////////////////////////////////////////////////
void setChildLock() {
  if (inService != 0) {       // We are in service
    return;
  }
  childLock = true;
  myCATGenie.setOutput(LED_LOCKED);
  sendMQTTServer = true;    
}

///////////////////////////////////////////////////////
// Reset Child Lock
///////////////////////////////////////////////////////
void resetChildLock() {
  if (inService != 0) {       // We are in service
    return;
  }
  childLock = false;
  myCATGenie.resetOutput(LED_LOCKED);
  sendMQTTServer = true;    
}

///////////////////////////////////////////////////////
// Set Clean Lock
///////////////////////////////////////////////////////
void setCleanLock() {
  if (inService != 0) {       // We are in service
    return;
  }
  cleanLocked = true;
  cleanLockedTime = 0;
  myCATGenie.setOutput(LED_CARTRIDGE);
  sendMQTTServer = true;    
}

///////////////////////////////////////////////////////
// Reset Clean Lock
///////////////////////////////////////////////////////
void resetCleanLock() {
  if (inService != 0) {       // We are in service
    return;
  }
  cleanLocked = false;
  cleanLockedTime = 0;
  myCATGenie.resetOutput(LED_CARTRIDGE);
  sendMQTTServer = true;    
}

///////////////////////////////////////////////////////
// Set Clean Program
///////////////////////////////////////////////////////
void setCleanProg(uint8_t prg) {

  if (inService != 0) {       // We are in service
    return;
  }
  cleanProgram = prg;
  cleanDelay = 0;
  cleanPending = false;        
  clearCatIn();
  // Activate CAT sensor if program 5
  if (cleanProgram == 5) {
    clearCatIn();
    lastCleaningTime = millis();
  }
  updatePanel(0);
  sendMQTTServer = true;    
}


///////////////////////////////////////////////////////
// Set Service Mode
///////////////////////////////////////////////////////
void setService() {
  myCATGenie.stopCleaning();
  cleanProgram = 0;
  inService = 1;        
  clearCatIn();
  inServiceState = 0;
  resetError();
  myCATGenie.resetOutputAll();
  myCATGenie.flashOutput(LED_CARTRIDGE,5);     //flush 0,5s
  digitalWrite(WATERLED,HIGH);
  updatePanel(0);
  sendMQTTServer = true;    
}

///////////////////////////////////////////////////////
// Reset Service Mode
///////////////////////////////////////////////////////
void resetService() {
  inService = 0;    
  myCATGenie.resetOutputAll();
  myCATGenie.resetOutput(LED_CARTRIDGE);
  cleanDelay = 0;
  cleanPending = false;        
  clearCatIn();
  digitalWrite(WATERLED,LOW);
  updatePanel(0);
  if (cleanLocked == true) {
    myCATGenie.setOutput(LED_CARTRIDGE);    
  }
  sendMQTTServer = true;    
}


///////////////////////////////////////////////////////
// Update LED Status on the panel
///////////////////////////////////////////////////////
void updatePanel(uint16_t output) {
  uint16_t showOutput = output;
  
  myCATGenie.resetOutput(LED_1);  
  myCATGenie.resetOutput(LED_2);  
  myCATGenie.resetOutput(LED_3);  
  myCATGenie.resetOutput(LED_4);  
  myCATGenie.resetOutput(LED_CAT);
  
  // show LEDs for service mode
  if (inService != 0) {
    switch (inService) {
      case 1:
        myCATGenie.setOutput(LED_1);
        showOutput = DRAIN_PUMP;                
        break;
      case 2:
        myCATGenie.setOutput(LED_2);
        showOutput = BOWL;                
        break;
      case 3:
        myCATGenie.setOutput(LED_3);
        showOutput = SCOPPER;                
        break;
      case 4:
        myCATGenie.setOutput(LED_4);
        showOutput = DRYER;                
        break;
      case 5:
        myCATGenie.setOutput(LED_1);
        myCATGenie.setOutput(LED_2);
        showOutput = WATERON;                
        break;
      case 6:
        myCATGenie.setOutput(LED_1);
        myCATGenie.setOutput(LED_3);
        showOutput = DOSAGE_PUMP;                
        break;
      default:
        myCATGenie.resetOutput(LED_CAT);
        showOutput = 0;
    }                
    // Show the state of this output in the CAT-Led (in service mode)
    if (showOutput != 0) {
      if (myCATGenie.getOutput(showOutput) == HIGH) {
        myCATGenie.setOutput(LED_CAT);        
      } else {
        myCATGenie.resetOutput(LED_CAT);            
      }
    }      

  // show LEDs for CAT Error
  } else if (catError != 0) {
      switch (catError) {
        case 0:
          break;
        case 1:             // CATGenie needs maintenance
          myCATGenie.setOutput(LED_1);
          break;
        case 2:             // Water fill problem
          myCATGenie.setOutput(LED_2);
          break;
        case 3:             // Water discharge problem
          myCATGenie.setOutput(LED_3);
          break;
        case 4:             // Dryer Fuse
          myCATGenie.setOutput(LED_4);
          break;
        case 5:             // No tasklist
          myCATGenie.setOutput(LED_4);
          myCATGenie.setOutput(LED_1);
          break;
        case 6:             // EEProm tasklist
          myCATGenie.setOutput(LED_4);
          myCATGenie.setOutput(LED_2);
          break;
        case 7:             // EEProm tasklist
          myCATGenie.setOutput(LED_4);
          myCATGenie.setOutput(LED_3);
          break;
        default:
          break;
      }    

  // Show LEDs for default mode
  } else {
    switch (cleanProgram) {
      case 1:
        myCATGenie.setOutput(LED_1);
        break;
      case 2:
        myCATGenie.setOutput(LED_2);
        break;
      case 3:
        myCATGenie.setOutput(LED_3);
        break;
      case 4:
        myCATGenie.setOutput(LED_4);
        break;
      case 5:
        if ((cleanDelay != 0) || (cleanPending == true)) {
          myCATGenie.flashOutput(LED_CAT,5);
        } else if (catIn == true) {
          myCATGenie.flashOutput(LED_CAT,20);
        } else {
          myCATGenie.setOutput(LED_CAT);          
        }
        break;
    }    
  }
}

///////////////////////////////////////////////////////
// Switch outputs in service mode
///////////////////////////////////////////////////////
void serviceSwichtOutput() {
  if (inService == 0) {
    return;
  }
  switch (inService) {

    // Drain-Pump
    case 1:
      switch (inServiceState) {
        case 0:
            myCATGenie.setOutput(DRAIN_PUMP,MAXDRAINTIME);
            inServiceState = 1;
            break;
        case 1:
            myCATGenie.resetOutput(DRAIN_PUMP);
            inServiceState = 0;
            break;
      }
      updatePanel(DRAIN_PUMP);
      break;

    // Bowl
    case 2:
      myCATGenie.resetOutput(BOWL);
      myCATGenie.resetOutput(BOWL_CW);
      switch (inServiceState) {
        case 0:
            myCATGenie.setOutput(BOWL,MAXBOWLTIME);
            inServiceState = 1;
            break;
        case 1:
            inServiceState = 2;
            break;
        case 2:
            myCATGenie.setOutput(BOWL_CW);
            myCATGenie.setOutput(BOWL,MAXBOWLTIME);
            inServiceState = 3;
            break;
        case 3:
            inServiceState = 0;
            break;
      }
      updatePanel(BOWL);
      break;

    // Scopper
    case 3:       // Scopper
      myCATGenie.resetOutput(SCOPPER);
      myCATGenie.resetOutput(SCOPPER_DOWN);
      switch (inServiceState) {
        case 0:
            myCATGenie.setOutput(SCOPPER,MAXSCOPTIME);
            inServiceState = 1;
            break;
        case 1:
            inServiceState = 2;
            break;
        case 2:
            myCATGenie.setOutput(SCOPPER_DOWN);
            myCATGenie.setOutput(SCOPPER,MAXSCOPTIME);
            inServiceState = 3;
            break;
        case 3:
            inServiceState = 0;
            break;
      }
      updatePanel(SCOPPER);
      break;

    //Dryer
    case 4:
      switch (inServiceState) {
        case 0:
            myCATGenie.setOutput(DRYER,MAXDRYERTIME);
            inServiceState = 1;
            break;
        case 1:
            myCATGenie.resetOutput(DRYER);
            inServiceState = 0;
            break;
      }
      updatePanel(DRYER);
      break;

    // Water
    case 5:
      switch (inServiceState) {
        case 0:
            myCATGenie.setOutput(WATERON,MAXWATERTIME);
            inServiceState = 1;
            break;
        case 1:
            myCATGenie.resetOutput(WATERON);
            inServiceState = 0;
            break;
      }
      updatePanel(WATERON);
      break;

    // Dosage-Pump
    case 6:
      switch (inServiceState) {
        case 0:
            myCATGenie.setOutput(DOSAGE_PUMP,MAXDOSTIME);
            inServiceState = 1;
            break;
        case 1:
            myCATGenie.resetOutput(DOSAGE_PUMP);
            inServiceState = 0;
            break;
      }
      updatePanel(DOSAGE_PUMP);
      break;
  }
}

///////////////////////////////////////////////////////
// Handle commands from RS485
///////////////////////////////////////////////////////
void rs485Handler() {
  char tempBuf[16];
  int i;
  
  if (rs485Read(rs485Buffer, 80) == true) {
    Serial.print("Received from RS485: <");
    Serial.print(rs485Buffer);
    Serial.println(">");    
    if (getParValue(rs485Buffer,(char*)"status=",tempBuf,16)) {
      if (strcmp(tempBuf, "lock") == 0) {
        setCleanLock();      
      } else if (strcmp(tempBuf, "free") == 0) {
        resetCleanLock();            
      }
    } else if (getParValue(rs485Buffer,(char*)"function=",tempBuf,16)) {
      if (strcmp(tempBuf, "submit") == 0) {
        confData[0].lenstruct = sizeof(confData);    
        i = eep.write(0, (byte*)confData, sizeof(confData));
        if (i == 0) {
          Serial1.println("->OK");          
        } else {
          Serial1.println("->ERROR");                    
        }
      }            
    } else {
      // SSID
      if (getParValue(rs485Buffer,(char*)"ssid=",tempBuf,16)) {
        confData[0].ssid[0] = '\0';
        strcat(confData[0].ssid, tempBuf);
        Serial1.println("->OK");
      }
      // Password
      if (getParValue(rs485Buffer,(char*)"passw=",tempBuf,16)) {
        confData[0].pass[0] = '\0';
        strcat(confData[0].pass, tempBuf);
        Serial1.println("->OK");
      }
      // IP-Adr
      if (getParValue(rs485Buffer,(char*)"ipadr=",tempBuf,16)) {
        confData[0].ip_adr[0] = '\0';
        strcat(confData[0].ip_adr, tempBuf);
        Serial1.println("->OK");
      }
      // Subnet
      if (getParValue(rs485Buffer,(char*)"subn=",tempBuf,16)) {
        confData[0].subnet[0] = '\0';
        strcat(confData[0].subnet, tempBuf);
        Serial1.println("->OK");
      }
      // Gateway
      if (getParValue(rs485Buffer,(char*)"gatew=",tempBuf,16)) {
        confData[0].gateway_ip[0] = '\0';
        strcat(confData[0].gateway_ip, tempBuf);
        Serial1.println("->OK");
      }
      // DNS
      if (getParValue(rs485Buffer,(char*)"dns=",tempBuf,16)) {
        confData[0].dnsserver_ip[0] = '\0';
        strcat(confData[0].dnsserver_ip, tempBuf);
        Serial1.println("->OK");
      }
    }
    rs485Buffer[0] = '\0';
    rs485Pointer = 0;
  }
}

///////////////////////////////////////////////////////
// Read from serial interface 1 (1 line)
///////////////////////////////////////////////////////
boolean rs485Read(char *buffer, int len) {
  int rc;

  while (Serial1.available() > 0) {
    rc = Serial1.read();      
    switch (rc) {
      case '\0':    // Ignore Null
        break;
      case '\n':    // Ignore new-lines
        break;
      case '\r':    // Return on CR
        rs485Pointer = 0;    // Reset position index ready for next lime
        return true;
      default:
        if (rs485Pointer < len-1) {
          buffer[rs485Pointer] = rc;
          rs485Pointer = rs485Pointer + 1;
          buffer[rs485Pointer] = '\0';
        } else {
          rs485Pointer = 0;
          buffer[0] = '\0';
          return false;
        }
    }
  }
  // No end of line has been found, so return false
  return false;
}



//////////////////////////////////////////////////////////////////////////
//
// Start WIFI Functions
//
//////////////////////////////////////////////////////////////////////////

// Connect / reconnect to WIFI
boolean ConnectToWIFI() {
  byte ipadr[4];

  if (strlen(confData[0].ssid) == 0) {
    return false;    
  }
  
  mqttclient.disconnect();
  WiFi.disconnect();

  Serial.print(F("Attempting to connect to WPA SSID: "));
  Serial.println(confData[0].ssid);
  parseBytes(confData[0].ip_adr, '.', ipadr, 4, 10);
  if ((ipadr[0] > 0) && (ipadr[0] < 255)) {                         // Statische IP verwenden   
    IPAddress ip(ipadr[0],ipadr[1],ipadr[2],ipadr[3]);              // IP-Adresse
    parseBytes(confData[0].dnsserver_ip, '.', ipadr, 4, 10);
    IPAddress dns(ipadr[0],ipadr[1],ipadr[2],ipadr[3]);             // DNS
    parseBytes(confData[0].gateway_ip, '.', ipadr, 4, 10);
    IPAddress gateway(ipadr[0],ipadr[1],ipadr[2],ipadr[3]);         // Gateway
    parseBytes(confData[0].subnet, '.', ipadr, 4, 10);
    IPAddress subnet(ipadr[0],ipadr[1],ipadr[2],ipadr[3]);          // Subnet

    WiFi.config(ip, dns, gateway, subnet);
  }

  // Connect to WPA/WPA2 network:    
  if (WiFi.begin(confData[0].ssid, confData[0].pass) == WL_CONNECTED) {
    printCurrentNet();
    printWifiData();
    Serial.println();

    // Save WEB-Page to EEProm
    EEPServer.begin();    

    // Start WEB Server
    WEBserver.begin();
       
    return true;
      
  } else {
    WiFi.disconnect();
    Serial.print(F("Failed to connect to the WIFI-SSID:"));      
    Serial.print(confData[0].ssid);
    Serial.print(F(" Status="));      
    Serial.println(WiFi.status());
    return false;
  }
}

void printCurrentNet() {
  // you're connected now, so print out the data:
  Serial.print(F("You're connected to the WIFI network"));
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print(F("BSSID: "));
  Serial.print(bssid[5], HEX);
  Serial.print(F(":"));
  Serial.print(bssid[4], HEX);
  Serial.print(F(":"));
  Serial.print(bssid[3], HEX);
  Serial.print(F(":"));
  Serial.print(bssid[2], HEX);
  Serial.print(F(":"));
  Serial.print(bssid[1], HEX);
  Serial.print(F(":"));
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI):"));
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print(F("Encryption Type:"));
  Serial.println(encryption, HEX);
}

void printWifiData() {
  IPAddress ip;
  char s[16];

  // print your WiFi shield's IP address:
  ip = WiFi.localIP();
  sprintf(s, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);    
  Serial.print(F("IP: "));
  Serial.println(s);
  ip = WiFi.subnetMask();
  sprintf(s, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);    
  Serial.print(F("Subnet:"));
  Serial.println(s);
  ip = WiFi.gatewayIP();
  sprintf(s, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);    
  Serial.print(F("Gateway:"));
  Serial.println(s);
 
  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print(F("MAC address: "));
  Serial.print(mac[5], HEX);
  Serial.print(F(":"));
  Serial.print(mac[4], HEX);
  Serial.print(F(":"));
  Serial.print(mac[3], HEX);
  Serial.print(F(":"));
  Serial.print(mac[2], HEX);
  Serial.print(F(":"));
  Serial.print(mac[1], HEX);
  Serial.print(F(":"));
  Serial.println(mac[0], HEX);
}
// End WIFI Functions
//////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////
//
// WEB-Server
//
//////////////////////////////////////////////////////////////////////////
void handleWEBServer() {

  #define PAGE_BUF_SZ 512
  #define REQ_BUF_SZ 80

  // Adresse mit WEB-Page Länge (WEB Page selbst = +64) 
  char HTTP_req[REQ_BUF_SZ] = {0};
  char req_index = 0;                      // index into HTTP_req buffer

  char data[PAGE_BUF_SZ];
  int i, i1, i2, value;
  uint32_t FileSize;
  char tempBuf[32];
  char tempBuf1[16];
  IPAddress ip;

  // listen for incoming clients
  WiFiClient WEBclient = WEBserver.available();
  
  if (WEBclient) {  // got client?
      Serial.println();
      Serial.println(F("WEB-client connected to Server.........."));

      boolean currentLineIsBlank = true;

      // reset buffer index and all buffer elements to 0
      req_index = 0;
      StrClear(HTTP_req, REQ_BUF_SZ);

      unsigned long start = millis();
      while (WEBclient.connected()) {
        if ((millis() - start) > 1000) {
          break;
        }
        
        if (WEBclient.available()) {    // client data available to read
          start = millis();
          char c = WEBclient.read();    // read 1 byte (character) from client
                
          // buffer first part of HTTP request in HTTP_req array
          // leave last element in array as 0 to null terminate string (REQ_BUF_SZ - 1)
          if (req_index < (REQ_BUF_SZ - 1)) {
            HTTP_req[req_index] = c;          // save HTTP request character
            req_index++;
          }
                
          // last line of client request is blank and ends with \n
          // respond to client only after last line received

          // Handle Request for favicon.ico
          //===============================
          if ((c == '\n') && currentLineIsBlank && StrContains(HTTP_req, (char*)"GET") && StrContains(HTTP_req, (char*)"favicon.ico"))  {
            Serial.println();
            Serial.println(HTTP_req);

            Serial.println(F("WEB-Server: Received GET favicon.ico -> Respond with Standard Header"));

            // send a standard http response header
            WEBclient.println("HTTP/1.1 200 OK");                    
            WEBclient.println("Content-Type: text/html");
            WEBclient.println("Connection: close");
            //WEBclient.println("Connection: keep-alive");
            WEBclient.println();
            // reset buffer index and all buffer elements to 0
            req_index = 0;
            StrClear(HTTP_req, REQ_BUF_SZ);
            break;                  
          }

          // Browser requests XML-Data from server
          //========================================================
          if ((c == '\n') && currentLineIsBlank && StrContains(HTTP_req, (char*)"GET") && StrContains(HTTP_req, (char*)"ajax_data"))  {
            Serial.println();
            Serial.println(F("WEB-Server: Received GET request for XML-Data"));
            Serial.print(F("LEN HTTP-Header: "));
            Serial.println(strlen(HTTP_req));
            Serial.println(HTTP_req);
            // send a standard http response header
            WEBclient.println("HTTP/1.1 200 OK");                    
            WEBclient.println("Content-Type: text/xml");
            WEBclient.println("Connection: close");
            //WEBclient.println("Connection: keep-alive");
            WEBclient.println();
            Serial.println(F("Respond with Standard Header"));
            Serial.println(F("Send XML Data"));

            // ----- Start send xml-data --------
            doCATGenie();

            //send XML file containing input states
            WEBclient.println("<?xml version = \"1.0\" ?>");

            // Start XML
            WEBclient.println("<data>");

            // Program version
            data[0] = '\0';
            //open tag
            strcat(data,"<prgver>");
            strcat(data,PRGVERSION);
            //close tag
            strcat(data,"</prgver>");
            WEBclient.println(data);

            // Auto Refresh
            data[0] = '\0';
            //open tag
            strcat(data,"<autoref>");
            if (wbAutoRefresh == true) {
              sprintf(tempBuf,"%s","1");                                    
            } else {
              sprintf(tempBuf,"%s","0");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</autoref>");
            WEBclient.println(data);

            // Error
            data[0] = '\0';
            //open tag
            strcat(data,"<error>");
            if (catError > 1) {
              sprintf(tempBuf,"%s","1");                                    
            } else {
              sprintf(tempBuf,"%s","0");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</error>");
            WEBclient.println(data);

            // Clean Watersensor
            data[0] = '\0';
            //open tag
            strcat(data,"<clean>");
            if (catError == 1) {
              sprintf(tempBuf,"%s","1");                                    
            } else {
              sprintf(tempBuf,"%s","0");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</clean>");
            WEBclient.println(data);

            // Cleaning locked
            data[0] = '\0';
            //open tag
            strcat(data,"<locked>");
            if (cleanLocked > 0) {
              sprintf(tempBuf,"%s","1");                                    
            } else {
              sprintf(tempBuf,"%s","0");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</locked>");
            WEBclient.println(data);

            // Clean Program
            data[0] = '\0';
            //open tag
            strcat(data,"<cleanprg>");
            sprintf(tempBuf,"%d",cleanProgram);                                    
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</cleanprg>");
            WEBclient.println(data);

            // Child lock
            data[0] = '\0';
            //open tag
            strcat(data,"<chlock>");
            if (childLock == true) {
              sprintf(tempBuf,"%s","1");                                    
            } else {
              sprintf(tempBuf,"%s","0");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</chlock>");
            WEBclient.println(data);

            // Service mode
            data[0] = '\0';
            //open tag
            strcat(data,"<service>");
            if (inService > 0) {
              sprintf(tempBuf,"%s","1");                                    
            } else {
              sprintf(tempBuf,"%s","0");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</service>");
            WEBclient.println(data);

            // Drain-Pump-Status
            data[0] = '\0';
            //open tag
            strcat(data,"<drain>");
            if (myCATGenie.getOutput(DRAIN_PUMP) == HIGH) {
              sprintf(tempBuf,"%s","ON");                      
            } else {
              sprintf(tempBuf,"%s","OFF");                                                          
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</drain>");
            WEBclient.println(data);

            // BOWL-Status
            data[0] = '\0';
            //open tag
            strcat(data,"<bowl>");
            if (myCATGenie.getOutput(BOWL_CW) == HIGH) {
              sprintf(tempBuf,"%s","CW");                      
            } else if (myCATGenie.getOutput(BOWL) == HIGH) {
              sprintf(tempBuf,"%s","CCW");                                            
            } else {
              sprintf(tempBuf,"%s","OFF");                                                          
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</bowl>");
            WEBclient.println(data);

            // Scopper-Status
            data[0] = '\0';
            //open tag
            strcat(data,"<sco>");
            if (myCATGenie.getOutput(SCOPPER_DOWN) == HIGH) {
              sprintf(tempBuf,"%s","DOWN");                      
            } else if (myCATGenie.getOutput(SCOPPER) == HIGH) {
              sprintf(tempBuf,"%s","UP");                                            
            } else {
              sprintf(tempBuf,"%s","OFF");                                                          
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</sco>");
            WEBclient.println(data);

            // Dryer-Status
            data[0] = '\0';
            //open tag
            strcat(data,"<dryer>");
            if (myCATGenie.getOutput(DRYER) == HIGH) {
              sprintf(tempBuf,"%s","ON");                      
            } else {
              sprintf(tempBuf,"%s","OFF");                                                          
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</dryer>");
            WEBclient.println(data);

            // Water-Status
            data[0] = '\0';
            //open tag
            strcat(data,"<water>");
            if (myCATGenie.getOutput(WATERON) == HIGH) {
              sprintf(tempBuf,"%s","ON");                      
            } else {
              sprintf(tempBuf,"%s","OFF");                                                          
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</water>");
            WEBclient.println(data);

            // Dosage-Status
            data[0] = '\0';
            //open tag
            strcat(data,"<dos>");
            if (myCATGenie.getOutput(DOSAGE_PUMP) == HIGH) {
              sprintf(tempBuf,"%s","ON");                      
            } else {
              sprintf(tempBuf,"%s","OFF");                                                          
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</dos>");
            WEBclient.println(data);

            data[0] = '\0';
            //open tag
            strcat(data,"<watval>");
            sprintf(tempBuf,"%d",myCATGenie.getWaterSensor());
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</watval>");
            WEBclient.println(data);

            // Water valve
            data[0] = '\0';
            //open tag
            strcat(data,"<watvalve>");
            if (digitalRead(WATERVALVE) == HIGH) {
              sprintf(tempBuf,"%s","0");                                    
            } else {
              sprintf(tempBuf,"%s","1");                                                  
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</watvalve>");
            WEBclient.println(data);
            
            // Info
            data[0] = '\0';
            //open tag
            strcat(data,"<info>");
            appendInfo(data);    
            //close tag
            strcat(data,"</info>");
            WEBclient.println(data);

            // Send actual Date/Time
            DateTime thisDT = rtc1388.now(confData[0].utcTimeoffs, confData[0].useDST);
            // Hour
            data[0] = '\0';
            //open tag
            strcat(data,"<hour>");
            sprintf(tempBuf,"%d",thisDT.hour());                      
            if (thisDT.hour() < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</hour>");
            WEBclient.println(data);

             // Minute
            data[0] = '\0';
            //open tag
            strcat(data,"<min>");
            sprintf(tempBuf,"%d",thisDT.minute());                      
            if (thisDT.minute() < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</min>");
            WEBclient.println(data);

             // Day
            data[0] = '\0';
            //open tag
            strcat(data,"<day>");
            sprintf(tempBuf,"%d",thisDT.day());                      
            if (thisDT.day() < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</day>");
            WEBclient.println(data);

             // Month
            data[0] = '\0';
            //open tag
            strcat(data,"<month>");
            sprintf(tempBuf,"%d",thisDT.month());                      
            if (thisDT.month() < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</month>");
            WEBclient.println(data);

             // Year
            data[0] = '\0';
            //open tag
            strcat(data,"<year>");
            sprintf(tempBuf,"%d",thisDT.year());                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</year>");
            WEBclient.println(data);

            // Timezone offset
            data[0] = '\0';
            //open tag
            strcat(data,"<tzoffs>");
            sprintf(tempBuf,"%d",confData[0].utcTimeoffs);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</tzoffs>");
            WEBclient.println(data);

            // Use DST
            data[0] = '\0';
            //open tag
            strcat(data,"<dst>");
            if (confData[0].useDST == true) {
              strcat(data,"1");              
            } else {
              strcat(data,"0");                            
            }
            //close tag
            strcat(data,"</dst>");
            WEBclient.println(data);

            // CAT in time
            data[0] = '\0';
            //open tag
            strcat(data,"<intime>");
            sprintf(tempBuf,"%d",confData[0].catTimeIn);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</intime>");
            WEBclient.println(data);

            // Delay start Cleaning
            data[0] = '\0';
            //open tag
            strcat(data,"<delay>");
            sprintf(tempBuf,"%d",confData[0].cleanDelay);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</delay>");
            WEBclient.println(data);

            // Force cleaning
            data[0] = '\0';
            //open tag
            strcat(data,"<force>");
            sprintf(tempBuf,"%d",confData[0].forceCleaning);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</force>");
            WEBclient.println(data);

             // Lock from HH
            data[0] = '\0';
            //open tag
            strcat(data,"<lhfrom>");
            sprintf(tempBuf,"%d",confData[0].lockHHfrom);                      
            if (confData[0].lockHHfrom < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</lhfrom>");
            WEBclient.println(data);

             // Lock from MM
            data[0] = '\0';
            //open tag
            strcat(data,"<lmfrom>");
            sprintf(tempBuf,"%d",confData[0].lockMMfrom);                      
            if (confData[0].lockMMfrom < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</lmfrom>");
            WEBclient.println(data);

             // Lock to HH
            data[0] = '\0';
            //open tag
            strcat(data,"<lhto>");
            sprintf(tempBuf,"%d",confData[0].lockHHto);                      
            if (confData[0].lockHHto < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</lhto>");
            WEBclient.println(data);

             // Lock to MM
            data[0] = '\0';
            //open tag
            strcat(data,"<lmto>");
            sprintf(tempBuf,"%d",confData[0].lockMMto);                      
            if (confData[0].lockMMto < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</lmto>");
            WEBclient.println(data);
            
             // Lock2 from HH
            data[0] = '\0';
            //open tag
            strcat(data,"<l2hfrom>");
            sprintf(tempBuf,"%d",confData[0].lock2HHfrom);                      
            if (confData[0].lock2HHfrom < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</l2hfrom>");
            WEBclient.println(data);

             // Lock2 from MM
            data[0] = '\0';
            //open tag
            strcat(data,"<l2mfrom>");
            sprintf(tempBuf,"%d",confData[0].lock2MMfrom);                      
            if (confData[0].lock2MMfrom < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</l2mfrom>");
            WEBclient.println(data);

             // Lock2 to HH
            data[0] = '\0';
            //open tag
            strcat(data,"<l2hto>");
            sprintf(tempBuf,"%d",confData[0].lock2HHto);                      
            if (confData[0].lock2HHto < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</l2hto>");
            WEBclient.println(data);

             // Lock2 to MM
            data[0] = '\0';
            //open tag
            strcat(data,"<l2mto>");
            sprintf(tempBuf,"%d",confData[0].lock2MMto);                      
            if (confData[0].lock2MMto < 10) {
              strcat(data, "0");
            }
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</l2mto>");
            WEBclient.println(data);

             // Tasklist Power-ON
            data[0] = '\0';
            //open tag
            strcat(data,"<tasklpow>");
            sprintf(tempBuf,"%d",confData[0].taskl_pow);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</tasklpow>");
            WEBclient.println(data);

             // Tasklist MAN
            data[0] = '\0';
            //open tag
            strcat(data,"<tasklman>");
            sprintf(tempBuf,"%d",confData[0].prog_man);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</tasklman>");
            WEBclient.println(data);

             // Tasklist 1
            data[0] = '\0';
            //open tag
            strcat(data,"<taskl1>");
            sprintf(tempBuf,"%d",confData[0].prog_1);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</taskl1>");
            WEBclient.println(data);

             // Tasklist 2
            data[0] = '\0';
            //open tag
            strcat(data,"<taskl2>");
            sprintf(tempBuf,"%d",confData[0].prog_2);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</taskl2>");
            WEBclient.println(data);

             // Tasklist 3
            data[0] = '\0';
            //open tag
            strcat(data,"<taskl3>");
            sprintf(tempBuf,"%d",confData[0].prog_3);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</taskl3>");
            WEBclient.println(data);

             // Tasklist 4
            data[0] = '\0';
            //open tag
            strcat(data,"<taskl4>");
            sprintf(tempBuf,"%d",confData[0].prog_4);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</taskl4>");
            WEBclient.println(data);

             // Tasklist CAT
            data[0] = '\0';
            //open tag
            strcat(data,"<tasklcat>");
            sprintf(tempBuf,"%d",confData[0].prog_cat);                      
            strcat(data, tempBuf);
            //close tag
            strcat(data,"</tasklcat>");
            WEBclient.println(data);

            // SSID
            data[0] = '\0';
            //open tag
            strcat(data,"<ssid>");
            strcat(data, confData[0].ssid);
            //close tag
            strcat(data,"</ssid>");
            WEBclient.println(data);

            // Password
            data[0] = '\0';
            //open tag
            strcat(data,"<passw>");
            strcat(data, confData[0].pass);
            //close tag
            strcat(data,"</passw>");
            WEBclient.println(data);

            // IP Address
            data[0] = '\0';
            //open tag
            strcat(data,"<ipadr>");
            strcat(data, confData[0].ip_adr);
            //close tag
            strcat(data,"</ipadr>");
            WEBclient.println(data);

            // Subnet
            data[0] = '\0';
            //open tag
            strcat(data,"<subn>");
            strcat(data, confData[0].subnet);
            //close tag
            strcat(data,"</subn>");
            WEBclient.println(data);

            // Gateway
            data[0] = '\0';
            //open tag
            strcat(data,"<gatew>");
            strcat(data, confData[0].gateway_ip);
            //close tag
            strcat(data,"</gatew>");
            WEBclient.println(data);

            // DNS
            data[0] = '\0';
            //open tag
            strcat(data,"<dns>");
            strcat(data, confData[0].dnsserver_ip);
            //close tag
            strcat(data,"</dns>");
            WEBclient.println(data);

            // MQTT server
            data[0] = '\0';
            //open tag
            strcat(data,"<mqttsrv>");
            strcat(data, confData[0].mqttserver);
            //close tag
            strcat(data,"</mqttsrv>");
            WEBclient.println(data);

            // MQTT-ID
            data[0] = '\0';
            //open tag
            strcat(data,"<mqttid>");
            strcat(data, confData[0].mqttid);
            //close tag
            strcat(data,"</mqttid>");
            WEBclient.println(data);

            // MQTT publish to
            data[0] = '\0';
            //open tag
            strcat(data,"<mqttpub>");
            strcat(data, confData[0].topic_Publish);
            //close tag
            strcat(data,"</mqttpub>");
            WEBclient.println(data);

            // MQTT subscribe from
            data[0] = '\0';
            //open tag
            strcat(data,"<mqttsub>");
            strcat(data, confData[0].topic_Subscribe);
            //close tag
            strcat(data,"</mqttsub>");
            WEBclient.println(data);

            // End XML
            WEBclient.println("</data>");
                                      
            doCATGenie();

            // ----- End send xml-data --------
            Serial.println(F("XML Data sent"));

            // reset buffer index and all buffer elements to 0
            req_index = 0;
            StrClear(HTTP_req, REQ_BUF_SZ);
            break;
          }

          // Handle POST Request from Browser (wait for posted data)
          //========================================================
          if ((c == '\n') && currentLineIsBlank && StrContains(HTTP_req, (char*)"POST") && StrContains(HTTP_req,(char*)"outputChange"))  {
            Serial.println();
            Serial.println(F("WEB-Server: Received POST request"));
            Serial.print(F("LEN HTTP-Header: "));
            Serial.println(strlen(HTTP_req));
            Serial.println(HTTP_req);

            data[0] = 0;
            i = 0;
            while (WEBclient.available()) {
              c = WEBclient.read();                  // read byte (character) from client
              if (i < PAGE_BUF_SZ) {
                data[i] = c;
                i = i + 1;
                data[i] = 0;
              }                    
            }

            Serial.print(F("LEN Data: "));
            Serial.println(strlen(data));
            Serial.println(data);

            // -------  Start handle received data ------------------------
            doCATGenie();

            Serial.println(F("Handle received data from WEB-Browser"));
            handlePostData(data);
            
            doCATGenie();
            // -------  End handle received data ------------------------
            
            // Get Server IP-Adress
            ip = WiFi.localIP();
            sprintf(tempBuf, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
            sprintf(tempBuf1, ":%d", WEBSERVER_PORT);
            strcat(tempBuf,tempBuf1);    
            if (wbTypeId >= 22) {
              sprintf(tempBuf1, "%s", "/#tab4");
            } else if (wbTypeId >= 20) {
              sprintf(tempBuf1, "%s", "/#tab3");                 
            } else if (wbTypeId >= 10) {
              sprintf(tempBuf1, "%s", "/#tab2");                 
            } else {
              sprintf(tempBuf1, "%s", "/#tab1");              
            }
            strcat(tempBuf,tempBuf1);    
            Serial.print(F("Redirecting to IP: "));
            Serial.println(tempBuf);
                  
            // Redirect client to this Home-Page
            WEBclient.println("HTTP/1.1 301 Moved Permanently");                    
            WEBclient.print("location: http://");
            WEBclient.println(tempBuf);
            WEBclient.println();

            // reset buffer index and all buffer elements to 0
            req_index = 0;
            StrClear(HTTP_req, REQ_BUF_SZ);
            break;
          }

          // Browser requests WEB-Page
          //============================
          if (((c == '\n') && currentLineIsBlank && (StrContains(HTTP_req, (char*)"GET"))))  {
            Serial.println();
            Serial.println(F("WEB-Server: Received GET request for WEB-Page"));
            Serial.print(F("LEN HTTP-Header: "));
            Serial.println(strlen(HTTP_req));
            Serial.println(HTTP_req);

            // send a standard http response header
            Serial.println(F("Send http response"));
                    
            WEBclient.println("HTTP/1.1 200 OK");                    
            WEBclient.println("Content-Type: text/html");
            WEBclient.println("Connection: close");
            //WEBclient.println("Connection: keep-alive");
            WEBclient.println();
                                                
            // Read WEB page from EEProm + send it to client                        
            eep.read(EEP_WEBPAGE, (byte*)data, 8);   // get WEB page length
            FileSize = atol((char*)data);            
            Serial.print(F("Send WEB-Page with size: "));
            Serial.println(FileSize);
            i = 0;
            while (FileSize > 0) {
              ////////////////////////////////////////////////////////////////////
              // Handle the CATGenie Object (for speed reasons)
              doCATGenie();
              ///////////////////////////////////////////////////////////////////
              if (FileSize >= PAGE_BUF_SZ) {
                 eep.read(EEP_WEBPAGE+64+i, (byte*)data, PAGE_BUF_SZ);
                 WEBclient.write(data, PAGE_BUF_SZ);      // send web page to client                             
              } else {
                 eep.read(EEP_WEBPAGE+64+i, (byte*)data, FileSize);
                 WEBclient.write(data, FileSize);      // send web page to client                             
                 break;                                                         
              }
              FileSize = FileSize - PAGE_BUF_SZ;
              i = i + PAGE_BUF_SZ;
            }
            // reset buffer index and all buffer elements to 0
            req_index = 0;
            StrClear(HTTP_req, REQ_BUF_SZ);
            break;                
          }

          
          // every line of text received from the client ends with \r\n
          if (c == '\n') {
            // starting new line with next character read
            currentLineIsBlank = true;
          } else if (c != '\r') {
            // a text character was received from client
            currentLineIsBlank = false;
          }
        } else {
          doCATGenie();
        } // end if client.available()
      } // end while client.connected()

      delay(2);         // give the web browser time to receive the data
      WEBclient.stop(); // close the connection
      Serial.println(F("WEB-Client disconnected from server.........."));
  } // end if WEBclient
}


// Append Info to data
void appendInfo(char* data) {
  char tempBuf[16];

  if ((catError > 1) && (catError < 99)) {
    strcat(data, "Error: ");              
    sprintf(tempBuf,"%d",catError);                      
    strcat(data, tempBuf);
  } else if (inService != 0) {
    strcat(data, "Service mode !!!!");              
  } else if (myCATGenie.isCleaning() != 0) {
    strcat(data, "Cleaning - time to go (min): ");              
    sprintf(tempBuf,"%d",myCATGenie.getRemainingTime()/60);                      
    strcat(data, tempBuf);
    switch (lastCleaningState) {
      case 1:
        strcat(data, " - scooping");
        break;
      case 2:
        strcat(data, " - washing");
        break;
      case 3:
        strcat(data, " - drying");
        break;
    }    
  } else if (timeLocked() == true) {
    strcat(data, "Outside global cleaning times");              
  } else if (cleanLocked == true) {
    strcat(data, "Cleaning is locked");              
  } else if (cleanPending == true) {
    strcat(data, "Start cleaning pending");                            
  } else if (cleanDelay != 0) {
    strcat(data, "Start cleaning in: ");              
    sprintf(tempBuf,"%d",cleanDelay);                      
    strcat(data, tempBuf);
    strcat(data, " Minutes");              
  } else if (catError == 99) {
    strcat(data, "Restart after power-on");              
  } else {
    strcat(data, "Ready");                            
  }  
}


// Handle data from WEB-Browser
void handlePostData(char *thisdata) {
  int i;
  char tempBuf[32];
  DateTime newDT;
  uint8_t newHour;
  uint8_t newMin;
  uint8_t newDay;
  uint8_t newMonth;
  uint16_t newYear;
  byte ipadr[4];

  if (getParValue(thisdata,(char*)"typeId=",tempBuf,16)) {
    wbTypeId = atoi(tempBuf);  
  } else {
    return;
  }

  // Restart MKR1000 (Wait for watchdog)
  if (wbTypeId == 99) {
    doRestart = true;
    return;
  }

  // Auto refresh browser
  if (wbTypeId == 0) {
    if (getParValue(thisdata,(char*)"autoref=",tempBuf,16)) {
      if (strcmp(tempBuf, "on") == 0) {
        wbAutoRefresh = true;
      }    
    } else {
        wbAutoRefresh = false;
    }
    Serial.println(F("WEB-Browser set to auto-refresh"));
    return;
  }

  
  // TAB-Sheet Home
  switch (wbTypeId) {
    // Start cleaning
    case 1:
      startCleaning();
      return;

    // Set Error
    case 2:
      if (getParValue(thisdata,(char*)"error=",tempBuf,16)) {
        if (strcmp(tempBuf, "on") == 0) {
          setError(11);   // Error manually set      
        }    
      } else {
        resetError();      
      }
      return;

    // Cleaning locked
    case 3:
      if (getParValue(thisdata,(char*)"locked=",tempBuf,16)) {
        if (strcmp(tempBuf, "on") == 0) {
          setCleanLock();      
        }    
      } else {
         resetCleanLock();      
      }
      return;

    // Child-Lock
    case 4:
      if (getParValue(thisdata,(char*)"chlock=",tempBuf,16)) {
        if (strcmp(tempBuf, "on") == 0) {
          setChildLock();
        }    
      } else {
        resetChildLock();
      }
      return;
      
    // Clean Program
    case 5:
      if (getParValue(thisdata,(char*)"cleanprg=",tempBuf,16)) {
        setCleanProg(atoi(tempBuf));
      }
      return;
  }
  
  // TAB-Sheet Service
  // Service ON/OFF
  if ((wbTypeId >= 10) && (wbTypeId < 20)) {
      if (wbTypeId == 10) {
        if (getParValue(thisdata,(char*)"service=",tempBuf,16)) {
          if ((strcmp(tempBuf, "on") == 0) && (inService == 0)) {
            setService();
          }
        } else {
          resetService();
        }
        return;
      }      
      if (inService == 0) {
        return;
      }
      switch (wbTypeId) {    
        // Drain Pump
        case 11:
          if (getParValue(thisdata,(char*)"drain=",tempBuf,16)) {
            inService = 1;
            inServiceState = 0;                        
            if (strcmp(tempBuf, "ON") == 0) {
              myCATGenie.setOutput(DRAIN_PUMP,MAXDRAINTIME);        
              inServiceState = 1;        
            } else {
              myCATGenie.resetOutput(DRAIN_PUMP);              
              inServiceState = 0;        
            }
            updatePanel(DRAIN_PUMP);            
          }
          return;
    
        // Bowl-Motor
        case 12:
          if (getParValue(thisdata,(char*)"bowl=",tempBuf,16)) {
            inService = 2;
            inServiceState = 0;                        
            myCATGenie.resetOutput(BOWL);              
            myCATGenie.resetOutput(BOWL_CW);
            delay(20);              
            if (strcmp(tempBuf, "CCW") == 0) {
              myCATGenie.setOutput(BOWL,MAXBOWLTIME);        
              inServiceState = 1;                        
            } else if (strcmp(tempBuf, "CW") == 0) {
              myCATGenie.setOutput(BOWL_CW);        
              myCATGenie.setOutput(BOWL,MAXBOWLTIME);
              inServiceState = 3;                        
            }
            updatePanel(BOWL);
          }
          return;
    
        // Scopper-Motor
        case 13:
          if (getParValue(thisdata,(char*)"sco=",tempBuf,16)) {
            inService = 3;
            inServiceState = 0;                        
            myCATGenie.resetOutput(SCOPPER);              
            myCATGenie.resetOutput(SCOPPER_DOWN);              
            delay(20);              
            if (strcmp(tempBuf, "DOWN") == 0) {
              myCATGenie.setOutput(SCOPPER_DOWN);        
              myCATGenie.setOutput(SCOPPER,MAXSCOPTIME);
              inServiceState = 3;                        
            } else if (strcmp(tempBuf, "UP") == 0) {
              myCATGenie.setOutput(SCOPPER,MAXSCOPTIME);
              inServiceState = 1;                        
            }
            updatePanel(SCOPPER);
          }
          return;
    
        // Dryer
        case 14:
          if (getParValue(thisdata,(char*)"dryer=",tempBuf,16)) {
            inService = 4;
            if (strcmp(tempBuf, "ON") == 0) {
              myCATGenie.setOutput(DRYER,MAXDRYERTIME);        
              inServiceState = 1;        
            } else {
              myCATGenie.resetOutput(DRYER);              
              inServiceState = 0;        
            }
            updatePanel(DRYER);
          }
          return;
          
        // Water
        case 15:
          if (getParValue(thisdata,(char*)"water=",tempBuf,16)) {
            inService = 5;
            if (strcmp(tempBuf, "ON") == 0) {
              myCATGenie.setOutput(WATERON,MAXWATERTIME);        
              inServiceState = 1;        
            } else {
              myCATGenie.resetOutput(WATERON);              
              inServiceState = 0;        
            }
            updatePanel(WATERON);
          }
          return;
    
        // Dosage
        case 16:
          if (getParValue(thisdata,(char*)"dos=",tempBuf,16)) {
            inService = 6;
            if (strcmp(tempBuf, "ON") == 0) {
              myCATGenie.setOutput(DOSAGE_PUMP,MAXDOSTIME);        
              inServiceState = 1;        
            } else {
              myCATGenie.resetOutput(DOSAGE_PUMP);              
              inServiceState = 0;        
            }
            updatePanel(DOSAGE_PUMP);
          }
          return;
      }     
   
  // Set Date / Time
  } else if (wbTypeId == 20) {
    if (getParValue(thisdata,(char*)"tzoffs=",tempBuf,16)) {
      confData[0].utcTimeoffs = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"dst=",tempBuf,16)) {
      if (strcmp(tempBuf, "on") == 0) {
        confData[0].useDST = true;
      }    
    } else {
        confData[0].useDST = false;
    }    
    if (getParValue(thisdata,(char*)"hour=",tempBuf,16)) {
      newHour = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"min=",tempBuf,16)) {
      newMin = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"day=",tempBuf,16)) {
      newDay = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"month=",tempBuf,16)) {
      newMonth = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"year=",tempBuf,16)) {
      newYear = atoi(tempBuf);
    }
    if ((newHour >= 0) && (newHour <= 23) && (newMin >= 0) && (newMin <=59) && (newDay >= 0) && (newDay <= 31) && (newMonth >= 1) && (newMonth <= 12)) {
      newDT = DateTime(newYear,newMonth,newDay,newHour,newMin,00);
      rtc1388.adjust(newDT);
    }  

  } else if (wbTypeId == 21) {
    // CAT in time
    if (getParValue(thisdata,(char*)"intime=",tempBuf,16)) {
      confData[0].catTimeIn = atoi(tempBuf);
    }
    // Delay Time for start cleaning
    if (getParValue(thisdata,(char*)"delay=",tempBuf,16)) {
      i = atoi(tempBuf);
      if ((i < 1) || (i > 1000)) {
        confData[0].cleanDelay = CLEANDELAY;          
      } else {
        confData[0].cleanDelay = i;
      }
    }
    // Force cleaning
    if (getParValue(thisdata,(char*)"force=",tempBuf,16)) {
      i = atoi(tempBuf);
      confData[0].forceCleaning = i;
    }
    // CATGenie locked from - to
    if (getParValue(thisdata,(char*)"lhfrom=",tempBuf,16)) {
      confData[0].lockHHfrom = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"lmfrom=",tempBuf,16)) {
      confData[0].lockMMfrom = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"lhto=",tempBuf,16)) {
      confData[0].lockHHto = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"lmto=",tempBuf,16)) {
      confData[0].lockMMto = atoi(tempBuf);
    }
  
    // CATGenie locked from - to 2
    if (getParValue(thisdata,(char*)"l2hfrom=",tempBuf,16)) {
      confData[0].lock2HHfrom = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"l2mfrom=",tempBuf,16)) {
      confData[0].lock2MMfrom = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"l2hto=",tempBuf,16)) {
      confData[0].lock2HHto = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"l2mto=",tempBuf,16)) {
      confData[0].lock2MMto = atoi(tempBuf);
    }
  
    // Tasklist for programs
    if (getParValue(thisdata,(char*)"tasklpow=",tempBuf,16)) {
      confData[0].taskl_pow = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"tasklman=",tempBuf,16)) {
      confData[0].prog_man = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"taskl1=",tempBuf,16)) {
      confData[0].prog_1 = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"taskl2=",tempBuf,16)) {
      confData[0].prog_2 = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"taskl3=",tempBuf,16)) {
      confData[0].prog_3 = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"taskl4=",tempBuf,16)) {
      confData[0].prog_4 = atoi(tempBuf);
    }
    if (getParValue(thisdata,(char*)"tasklcat=",tempBuf,16)) {
      confData[0].prog_cat = atoi(tempBuf);
    }

  } else if (wbTypeId == 22) {
    // SSID
    if (getParValue(thisdata,(char*)"ssid=",tempBuf,16)) {
      confData[0].ssid[0] = '\0';
      strcat(confData[0].ssid, tempBuf);
    }
    // Password
    if (getParValue(thisdata,(char*)"passw=",tempBuf,16)) {
      confData[0].pass[0] = '\0';
      strcat(confData[0].pass, tempBuf);
    }
    // IP-Adr
    if (getParValue(thisdata,(char*)"ipadr=",tempBuf,16)) {
      confData[0].ip_adr[0] = '\0';
      strcat(confData[0].ip_adr, tempBuf);
    }
    // Subnet
    if (getParValue(thisdata,(char*)"subn=",tempBuf,16)) {
      confData[0].subnet[0] = '\0';
      strcat(confData[0].subnet, tempBuf);
    }
    // Gateway
    if (getParValue(thisdata,(char*)"gatew=",tempBuf,16)) {
      confData[0].gateway_ip[0] = '\0';
      strcat(confData[0].gateway_ip, tempBuf);
    }
    // DNS
    if (getParValue(thisdata,(char*)"dns=",tempBuf,16)) {
      confData[0].dnsserver_ip[0] = '\0';
      strcat(confData[0].dnsserver_ip, tempBuf);
    }
    // MQTT Server IP
    if (getParValue(thisdata,(char*)"mqttsrv=",tempBuf,16)) {
      confData[0].mqttserver[0] = '\0';
      strcat(confData[0].mqttserver, tempBuf);
    }
    // MQTT ID
    if (getParValue(thisdata,(char*)"mqttid=",tempBuf,16)) {
      confData[0].mqttid[0] = '\0';
      strcat(confData[0].mqttid, tempBuf);
    }
    // MQTT publish to
    if (getParValue(thisdata,(char*)"mqttpub=",tempBuf,32)) {
      convertUrl(tempBuf);
      confData[0].topic_Publish[0] = '\0';
      strcat(confData[0].topic_Publish, tempBuf);
    }
    // MQTT subscribe from
    if (getParValue(thisdata,(char*)"mqttsub=",tempBuf,32)) {
      convertUrl(tempBuf);
      confData[0].topic_Subscribe[0] = '\0';
      strcat(confData[0].topic_Subscribe, tempBuf);
    }    
  }

  // Settings have changed
  if ((wbTypeId == 20) || (wbTypeId == 21) || (wbTypeId == 22)) {
    confData[0].lenstruct = sizeof(confData);    
    i = eep.write(0, (byte*)confData, sizeof(confData));
    Serial.print(F("Configuration file written size: "));
    Serial.println(sizeof(confData));
    Serial.println(F("---------------------"));

    if (wbTypeId == 22) {
      mqttclient.disconnect();
      parseBytes(confData[0].mqttserver, '.', ipadr, 4, 10);
      if (((ipadr[0] > 0) && (ipadr[0] < 255))) {
        mqttclient.setCallback(mqtt_callback);
        mqttclient.setServer(confData[0].mqttserver, 1883);
        ConnectToMQTT();  
      }
    }
  }    
}

// sets every element of str to 0 (clears array)
void StrClear(char *str, int length){
    for (int i = 0; i < length; i++) {
        str[i] = 0;
    }
}

// searches for the string sfind in the string str
// returns 1 if string found
// returns 0 if string not found
boolean StrContains(char *str, char *sfind) {
    int found = 0;
    int index = 0;
    int len;

    len = strlen(str);    
    if (strlen(sfind) > len) {
        return 0;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                return 1;
            }
        } else {
            found = 0;
        }
        index++;
    }
    return 0;
}

// searches for the string sfind in the string str
// returns value between end of found string und next "&", "/n", "/r"
boolean getParValue(char *str, char *sfind, char *value, int valmaxlen) {
    int found = 0;
    int index = 0;
    int index1 = 0;
    int len;
    int i;

    len = strlen(str);    
    if (strlen(sfind) > len) {
        value[0] = 0;
        return false;
    }
    while (index < len) {
        if (str[index] == sfind[found]) {
            found++;
            if (strlen(sfind) == found) {
                // Parameter found -> find end
                index++;
                index1 = 0;
                value[index1] = 0;                                              
                while ((index < len) && (index1 < valmaxlen)) {
                  if ((str[index] == '&') || (str[index] == '\n') || (str[index] == '\r')) {
                    break;
                  } else {
                    value[index1] = str[index];
                    index++;
                    index1++;
                    value[index1] = '\0';
                  }
                }
                trimValue(value);
                if (strlen(value) ==  0) {
                  return false;
                } else {
                  return true;                      
                }
            }
        } else {
            found = 0;
        }
        index++;
    }
    value[0] = 0;
    return false;
}
// End WEB-Server
//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////
//
// Start MQTT Functions
// Max Message size = 128 Byte (inkl Header) in PubSubClient.h
//
//////////////////////////////////////////////////////////////////////////

// Connect (reconnect) to MQTT Server
boolean ConnectToMQTT() {  
  byte ipadr[4];

  // Keine Übertragung zu MQTT Server
  parseBytes(confData[0].mqttserver, '.', ipadr, 4, 10);
  if (!((ipadr[0] > 0) && (ipadr[0] < 255))) {
    return true;  
  }

  if (WiFi.status() == WL_CONNECTED) {
    mqttclient.disconnect();
    Serial.print(F("MQTT try register user 'catgenie'.... "));

    if (mqttclient.connect(confData[0].mqttid)) {
      Serial.println(F("registered"));

      // Subscribe to Data
      Serial.print(F("MQTT subscribe to: "));
      Serial.println(confData[0].topic_Subscribe);
      mqttclient.subscribe(confData[0].topic_Subscribe);
      return true;

    } else {
      mqttclient.disconnect();
      Serial.print(F("failed, rc="));
      Serial.println(mqttclient.state());
    }
  }
  return false;
}

// Callback Receive data from MQTT Server
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  char* bytearray = (char*)payload;
  char tempBuf[16];
  
  payload[length] = '\0';
  Serial.print(F("MQTT received Topic: "));
  Serial.println(topic);
  Serial.print(F("Data: "));
  Serial.println(bytearray);

  // We don't accept commands from MQTT while in service mode
  if (inService != 0) {
    sendMQTTServer = true;    
    return;
  }
  if (strcmp(topic, confData[0].topic_Subscribe) == 0) {
    // CAT Genie Error
    if (getParValue(bytearray,(char*)"CATError@",tempBuf,16)) {
      if (strcmp(tempBuf, "ON") == 0) {
        setError(12);   // Error manually set      
      } else if (strcmp(tempBuf, "OFF") == 0) {
        resetError();      
      }      

    // CAT Genie Locked
    } else if (getParValue(bytearray,(char*)"CATLocked@",tempBuf,16)) {
      if (strcmp(tempBuf, "ON") == 0) {
        setCleanLock();      
      } else if (strcmp(tempBuf, "OFF") == 0) {
        resetCleanLock();      
      }      

    // Child Locked
    } else if (getParValue(bytearray,(char*)"CATChildLock@",tempBuf,16)) {
      if (strcmp(tempBuf, "ON") == 0) {
        setChildLock();      
      } else if (strcmp(tempBuf, "OFF") == 0) {
        resetChildLock();      
      }      

    // Clean Program
    } else if (getParValue(bytearray,(char*)"CATProgram@",tempBuf,16)) {      
        setCleanProg(atoi(tempBuf));      

    // Start Cleaning
    } else if (getParValue(bytearray,(char*)"CATStartCleaning@",tempBuf,16)) {
      if (strcmp(tempBuf, "ON") == 0) {
        startCleaning();      
      }      
    // Request Status
    } else if (getParValue(bytearray,(char*)"_reqState",tempBuf,16)) {
      sendMQTTServer = true;    
    }
  }
}


// Send actual states to MQTT server
boolean sendToMQTTServer() {
  
  char data[128];     //buffer to send data
  char tempBuf[16];

  // Send Data to MQTT Server
  if ((WiFi.status() == WL_CONNECTED) && mqttclient.connected()) {    
    data[0] = 0x0;

    // Info (Parameter 0)
    appendInfo(data);    
    strcat(data, "@");                            

    // Cleaning Programm (Parameter 1)
    sprintf(tempBuf,"%d",cleanProgram);                                    
    strcat(data, tempBuf);
    strcat(data, "@");                            

    // Error (Parameter 2)
    if (catError > 1) {
      strcat(data, "ON");
    } else {
      strcat(data, "OFF");
    }
    strcat(data, "@");                            

    // Locked (Parameter 3)
    if (cleanLocked == true) {
      strcat(data, "ON");
    } else {
      strcat(data, "OFF");
    }
    strcat(data, "@");                            

    // Child Lock (Parameter 4)
    if (childLock == true) {
      strcat(data, "ON");
    } else {
      strcat(data, "OFF");
    }
    strcat(data, "@");                            

    // Maintenance (Parameter 5)
    if (catError == 1) {
      strcat(data, "ON");
    } else {
      strcat(data, "OFF");
    }
    strcat(data, "@");                            

    // Water sensor analog value (Parameter 6)
    sprintf(tempBuf,"%d",myCATGenie.getWaterSensor());
    strcat(data, tempBuf);
    strcat(data, "@");                            

    Serial.print(F("Publish to MQTT topic '"));
    Serial.print(confData[0].topic_Publish);
    Serial.print(F("' Data: "));
    Serial.print(data);
    if (mqttclient.publish(confData[0].topic_Publish, data)) {
      Serial.println(F("....ok"));      
    } else {
      mqttclient.disconnect();
      Serial.println(F("....error"));            
    }
  }  
}



//////////////////////////////////////////////////////////////////////////
//
// Receive WEB-Page, Cleaning programs -> Save to EEProm
// WEB-Page: 1. Line = <!DOCTYPE html>
// Cleaning Programs: 1. Line = <CATGenie>
//
//////////////////////////////////////////////////////////////////////////

void receiveEEPData() {
  int fileType = 0;
  char packetBuffer[260];  //buffer to hold incoming packets
  uint32_t FileSize = 0;
  int packetSize;
  char charBuff [20];
  byte eepStatus;
  boolean eof;
  char tempBuf[64];
  int lineNo;
  unsigned long start;
  
  // listen for incoming clients
  WiFiClient EEPClient = EEPServer.available();
  if (EEPClient) {  // got client?
    eof = false;
    FileSize = 0;      
    packetSize = 0;
    packetBuffer[packetSize] = '\0';        
    Serial.println(F("EEP-Client connected to server.........."));
    
    // Wait for first line
    start = millis();
    while (EEPClient.connected()) {
      if ((millis() - start) > 500) {
        EEPClient.stop();   // close the connection      
        Serial.println(F("Timeout - EEP-Client disconnected from server.........."));
        return;
      }
      if (EEPClient.available()) {        // client data available to read
        start = millis();
        char b = EEPClient.read();        // read 1 byte (character) from client
        if (b == 0x1c) {                  // end of file
          eof = true;
          break;
        } else {
          packetBuffer[packetSize] = b;
          packetSize = packetSize + 1;
          packetBuffer[packetSize] = '\0';
          if (b == '\n') {
            break;
          }
        }        
      }
    }
    
    // Receive WEB-Page
    if (StrContains(packetBuffer, (char*)"<!DOCTYPE html>")) {
      fileType = 1;
          
    // Receive Cleaning programs
    } else if (StrContains(packetBuffer, (char*)"<CATGenie>")) {
      fileType = 2;
    }
        
    if ((fileType != 1) && (fileType != 2)) {
      Serial.print(F("Invalid first line: "));
      Serial.println(packetBuffer);
      EEPClient.flush();
      EEPClient.stop(); // close the connection            
      return;                
    }
    
    
    // Continue receive WEB-Page
    // -------------------------
    if (fileType == 1) {
      start = millis();
      while (EEPClient.connected()) {
        if ((millis() - start) > 500) {
          EEPClient.stop();   // close the connection      
          Serial.println(F("Timeout - EEP-Client disconnected from server.........."));
          return;
        }
        if (EEPClient.available()) {        // client data available to read
          start = millis();
          char b = EEPClient.read();        // read 1 byte (character) from client
          if (b == 0x1c) {                  // end of file
            eof = true;          
          } else {
            packetBuffer[packetSize] = b;
            packetSize = packetSize + 1;
            packetBuffer[packetSize] = '\0';
          }        
        }
              
        if ((packetSize >= 256) || (eof == true)) {
          // Trigger Watchdog
          rtc1388.resetWatchdogTimer();  
  
          //Save Line to EEPROM
          if (FileSize > EEP_MAXWEBPAGE) {
            Serial.print(F("!!!!!!!!!! Max Filesize for WEB-PAge is (Byte)= "));
            Serial.println(EEP_MAXWEBPAGE);
            EEPClient.flush();
            break;          
          }
          eepStatus = eep.write(EEP_WEBPAGE + 64 + FileSize, (byte*) packetBuffer, packetSize);
          if (eepStatus != 0) {
            Serial.print(F("Error writing WEB-Page to EEPROM! Status: "));
            Serial.println(eepStatus);
            EEPClient.flush();
            break;
          }
          FileSize = FileSize + packetSize;       
          Serial.print(F("WEB-Page File size: "));
          Serial.println(FileSize);              
          packetSize = 0;
          packetBuffer[0] = '\0';
          if (eof == true) {
            Serial.println(F("End of WEB-Page File........... "));
            break;                        
          }
        }      
      }              
      EEPClient.stop(); // close the connection      

      //Save File size to EEPROM
      if (FileSize > 0) {
        for (int i=0; i < 20; i++){
          charBuff[i] = '\0';
        }
        sprintf(charBuff,"%d",FileSize);
        eepStatus = eep.write(EEP_WEBPAGE, (byte*) charBuff, 8);
        if (eepStatus != 0) {
          Serial.print(F("Error writing WEB-Page File Size to EEPROM Status: "));
          Serial.println(eepStatus);
        }
      }
      Serial.print(F("WEB-Page saved. "));    
      Serial.print(F("Size: "));
      Serial.println(FileSize);
      Serial.println(F("-----------------"));                                

    // Continue receive Cleaning program
    // ---------------------------------    
    } else if (fileType == 2) {

      Serial.println(F("Receive tasklist for cleaning program"));                        
      packetBuffer[0] = '\0';
      packetSize = 0;
      lineNo = 2;
      progId = 0;
      while (EEPClient.connected()) {
        if (EEPClient.available()) {        // client data available to read
          char b = EEPClient.read();        // read 1 byte (character) from client
          if (b == 0x1c) {                  // end of file
            eof = true;
            break;          
          } else {
            packetBuffer[packetSize] = b;
            packetSize = packetSize + 1;
            packetBuffer[packetSize] = '\0';
            
            if (b == '\n') {                // New line received
              // Trigger Watchdog
              rtc1388.resetWatchdogTimer();  
  
              packetBuffer[strlen(packetBuffer)-2] = 0x0;    // Remove CR,LF
              int i = decodeLine(lineNo, packetBuffer);

              // Save Tasklist to EEProm
              if (i > 0) {      // All done -> save
                Serial.print(F("File ok -> Save tasklist: "));                
                Serial.println(i);                
                FileSize = myCATGenie.saveProgram(i);
                if (FileSize == -1) {
                  Serial.println(F(" -> Tasklist is to long"));                        
                } else if (FileSize == -2) {
                  Serial.println(F(" -> EEProm Error !!!!!!!!!"));                        
                } else {
                  Serial.print(F("Tasklist saved. "));    
                  Serial.print("Size: ");                
                  Serial.println(FileSize);                                
                  Serial.println(F("-----------------"));                                
                }
              } else if (i < 0) {
                break;              
              }
              packetBuffer[0] = '\0';
              packetSize = 0;
              lineNo = lineNo + 1;
            }            
          }
        }        
      }
      EEPClient.stop();   // close the connection      
      Serial.println(F("EEP-Client disconnected from server.........."));
    }
  }
}

// Create Task from this line
// ---------------------------
int decodeLine(int line, char* data) {
  char tmpBuf[64];
  float f;

  if ((strlen(data) < 4) || ((data[0] == '/') && (data[1] == '/'))) {
    return 0;
  }
  Serial.print(F("Execute line: "));        
  Serial.println(line);        
  getParameter(0, ' ', data, tmpBuf);
  
  // -- program <start/end> <Number> --
  if (strcmp(tmpBuf,"tasklist") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"start") == 0) {
      getParameter(2, ' ', data, tmpBuf);
      progId = atoi(tmpBuf);
      if ((progId < 1) || (progId > 5)) {
        progId = 0;
        Serial.print(F("Parameter 2 not valid in line: "));        
        Serial.println(line);        
        Serial.println(data);
        return -1;                      
      } else {
        myCATGenie.newProgram();
        return 0;        
      }            
    } else if (strcmp(tmpBuf,"end") == 0) {
      if (progId > 0) {
        return progId;        
      } else {
        Serial.print(F("Error: tasklist end without tasklist start"));        
        Serial.println(line);        
        Serial.println(data);
        return -1;      
      }
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;      
    }
    return 0;
    
  // -- wait <time in seconds> --
  } else if (strcmp(tmpBuf,"wait") == 0) {
    getParameter(1, ' ', data, tmpBuf);    
    f = atof(tmpBuf) * 10;
    if (f != 0) {
      myCATGenie.addProgTask(1, (int)f, 0);
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;                        
    }
    
  // -- scopper <stop/up/down> -- 
  } else if (strcmp(tmpBuf,"scopper") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"up") == 0) {
      myCATGenie.addProgTask(2, SCOPPER, 1);      
    } else if (strcmp(tmpBuf,"down") == 0) {
      myCATGenie.addProgTask(2, SCOPPER_DOWN, 1);            
    } else if (strcmp(tmpBuf,"stop") == 0) {
      myCATGenie.addProgTask(2, SCOPPER, 0);            
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;            
    }

  // -- bowl <stop/cw/ccw> -- 
  } else if (strcmp(tmpBuf,"bowl") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"ccw") == 0) {
      myCATGenie.addProgTask(2, BOWL, 1);      
    } else if (strcmp(tmpBuf,"cw") == 0) {
      myCATGenie.addProgTask(2, BOWL_CW, 1);            
    } else if (strcmp(tmpBuf,"stop") == 0) {
      myCATGenie.addProgTask(2, BOWL, 0);            
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;            
    }

  // -- drainpump <on/stop> -- 
  } else if (strcmp(tmpBuf,"drainpump") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"on") == 0) {
      myCATGenie.addProgTask(2, DRAIN_PUMP, 1);      
    } else if (strcmp(tmpBuf,"stop") == 0) {
      myCATGenie.addProgTask(2,DRAIN_PUMP, 0);            
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;            
    }
    
  // -- water <on/stop/waitin/waitout/is on/is stop> -- 
  } else if (strcmp(tmpBuf,"water") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"on") == 0) {
      myCATGenie.addProgTask(2, WATERON, 1);      
    } else if (strcmp(tmpBuf,"stop") == 0) {
      myCATGenie.addProgTask(2,WATERON, 0);            
    } else if (strcmp(tmpBuf,"waitin") == 0) {
      getParameter(2, ' ', data, tmpBuf);
      f = atof(tmpBuf) * 10;
      if (f != 0) {
        myCATGenie.addProgTask(3, 1, (int)f);
      } else {
        Serial.print(F("Parameter 2 not valid in line: "));        
        Serial.println(line);        
        Serial.println(data);
        return -1;                        
      }
    } else if (strcmp(tmpBuf,"waitout") == 0) {
      getParameter(2, ' ', data, tmpBuf);
      f = atof(tmpBuf) * 10;
      if (f != 0) {
        myCATGenie.addProgTask(3, 2, (int)f);
      } else {
        Serial.print(F("Parameter 2 not valid in line: "));        
        Serial.println(line);        
        Serial.println(data);
        return -1;                        
      }
    } else if (strcmp(tmpBuf,"is") == 0) {
      getParameter(2, ' ', data, tmpBuf);
      if (strcmp(tmpBuf,"on") == 0) {
        myCATGenie.addProgTask(4, WATERON, 1);      
      } else if (strcmp(tmpBuf,"stop") == 0) {
        myCATGenie.addProgTask(4,WATERON, 0);            
      } else {
        Serial.print(F("Parameter 2 not valid in line: "));        
        Serial.println(line);        
        Serial.println(data);
        return -1;            
      }
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;            
    }

  // -- dospump <on/stop> -- 
  } else if (strcmp(tmpBuf,"dospump") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"on") == 0) {
      myCATGenie.addProgTask(2, DOSAGE_PUMP, 1);      
    } else if (strcmp(tmpBuf,"stop") == 0) {
      myCATGenie.addProgTask(2,DOSAGE_PUMP, 0);            
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;            
    }

  // -- dryer <on/stop> -- 
  } else if (strcmp(tmpBuf,"dryer") == 0) {
    getParameter(1, ' ', data, tmpBuf);
    if (strcmp(tmpBuf,"on") == 0) {
      myCATGenie.addProgTask(2, DRYER, 1);      
    } else if (strcmp(tmpBuf,"stop") == 0) {
      myCATGenie.addProgTask(2,DRYER, 0);            
    } else {
      Serial.print(F("Parameter 1 not valid in line: "));        
      Serial.println(line);        
      Serial.println(data);
      return -1;            
    }

  } else {
    Serial.print(F("Command (parameter 0) unknown in line: "));        
    Serial.println(line);        
    Serial.println(data);
    return -1;        
  }
  
  return 0;
}


// Exctract desired parameter from string
// --------------------------------------
boolean getParameter(int parno, char sep, char* line, char* tmpBuf) {
  tmpBuf[0] = 0x0;
  int i, i1, i2, i3;
  
  i2 = 0;
  if (parno > 0) {
    i1 = 1;
    for (i=0; i<strlen(line); i++) {
      if (line[i] == sep) {
        if (i1 == parno) {
          i2 = i;
          break;          
        } else {
          i1 = i1 + 1;
        }
      }    
    }
    if (i2 == 0) {
      return false;
    }
  }
  
  // Search first non blank
  while (i2 < strlen(line)) {
    if (line[i2] != ' ') {
      break;    
    }
    i2 = i2 + 1;
  }
  if (i2 >= strlen(line)) {
    return false;
  }
  
  // Copy parameter to tmpBuf
  i = 0;
  while (i2 <= strlen(line)) {
    if ((i2 >= strlen(line)) || (line[i2] == ' ')) {
      return true;
    } 
    tmpBuf[i] = line[i2];
    i = i + 1;
    tmpBuf[i] = 0x0;
    i2 = i2 + 1;
  }
  return false;
}


//////////////////////////////////////////////////////////////////////////
//
// Erease EEProm
//
//////////////////////////////////////////////////////////////////////////
//write 0xFF to eeprom, "chunk" bytes at a time
boolean eeErase(uint8_t chunk, uint32_t startAddr, uint32_t endAddr)
{
  boolean ok = true;
  
  chunk &= 0xFC;                //force chunk to be a multiple of 4
  uint8_t data[chunk];
  for (int i = 0; i < chunk; i++) data[i] = 0xFF;

  for (uint32_t a = startAddr; a <= endAddr; a += chunk) {
    if (eep.write(a, data, chunk) != 0) {
      ok = false;
    }
  }
  return ok;
}


//////////////////////////////////////////////////////////////////////////
//
// Start Generall Functions
//
//////////////////////////////////////////////////////////////////////////

void convertUrl(char* data) {
  // Create two pointers that point to the start of the data
  char* leader = data;
  char* follower = leader;
  
  // While we're not at the end of the string (current character not NULL)
  while (*leader) {
      // Check to see if the current character is a %
      if (*leader == '%') {
  
          // Grab the next two characters and move leader forwards
          leader++;
          char high = *leader;
          leader++;
          char low = *leader;
  
          // Convert ASCII 0-9A-F to a value 0-15
          if (high > 0x39) high -= 7;
          high &= 0x0f;
  
          // Same again for the low byte:
          if (low > 0x39) low -= 7;
          low &= 0x0f;
  
          // Combine the two into a single byte and store in follower:
          *follower = (high << 4) | low;
      } else {
          // All other characters copy verbatim
          *follower = *leader;
      }
  
      // Move both pointers to the next character:
      leader++;
      follower++;
  }
  *follower = 0x0;
  return;
}



// Parse string (eg ip address)
// ---------------------------
void parseBytes(const char* str, char sep, byte* bytes, int maxBytes, int base) {
    for (int i = 0; i < maxBytes; i++) {
        bytes[i] = strtoul(str, NULL, base);  // Convert byte
        str = strchr(str, sep);               // Find next separator
        if (str == NULL || *str == '\0') {
            break;                            // No more separators, exit
        }
        str++;                                // Point to next character after separator
    }
}

// AllTrim char Array
// ------------------
void trimValue(char* value) {
  int i, i1, i2;
  
  // Replace all pre "+" durch blank
  for (i=0; i<strlen(value); i++){
    if (value[i] != '+') {
      break;
    } else {
      value[i] = ' ';
    }
  }

  // No pre/post blanks
  if ((strlen(value) == 0) || ((value[0] != ' ') && (value[strlen(value)-1] != ' '))) {
    return;
  }
  
  i1 = 0;
  i2 = strlen(value);  
  // Find first
  for (i=0; i<strlen(value); i++){
    if (value[i] != ' ') {
      i1 = i;
      break;
    }
  }
  
  // All blanks
  if (i >= strlen(value)) {
    value[0] = '\0';
    return;
  }

  // Find last
  for (i=(strlen(value)-1); i>=0; i--){
    if (value[i] != ' ') {
      break;
    }
  }
  i2 = i;
    
  // Only post blanks
  if (i1 == 0) {
    value[i2+1] = '\0';
    return;
  }
    
  // Move string to begin
  i2 = i2 - i1;
  for (i=0; i<=i2; i++) {
    value[i] = value[i1];
    i1++;    
  }
  value[i] = '\0';
}

// Calculate the MS from actual to start
// -------------------------------------
unsigned long getDiffTime(unsigned long fromTime, unsigned long toTime) {
  unsigned long i;  
  
  if (toTime >= fromTime) {
    return (toTime - fromTime);
  } else {
    i = 2^32 - fromTime + toTime;
    return i;
  }
}

//////////////////////////////////////////////////////////////////////////
//
// TIMER4 SPECIFIC FUNCTIONS FOLLOW
// you shouldn't change these unless you know what you're doing
//
//////////////////////////////////////////////////////////////////////////

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once

void tc4Configure(int sampleRate) {
   // Enable GCLK for Tc4 and TC5 (timer counter input clock)
   GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
   while (GCLK->STATUS.bit.SYNCBUSY);
  
   tc4Reset(); //reset TC4
  
   // Set Timer counter Mode to 16 bits
   TC4->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
   // Set TC4 mode as match frequency
   TC4->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
   //set prescaler and enable TC4
   TC4->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV256 | TC_CTRLA_ENABLE;
   //set TC4 timer counter based off of the system clock and the user defined sample rate or waveform
   TC4->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / (256*sampleRate) - 1);
   while (tc4IsSyncing());
   
   // Configure interrupt request
   NVIC_DisableIRQ(TC4_IRQn);
   NVIC_ClearPendingIRQ(TC4_IRQn);
   //NVIC_SetPriority(TC4_IRQn,0);    // Set highest priority
   NVIC_SetPriority(TC4_IRQn,(1 << __NVIC_PRIO_BITS) -1);    // Set lowest priority -1
   NVIC_EnableIRQ(TC4_IRQn);
  
   // Enable the TC4 interrupt request
   TC4->COUNT16.INTENSET.bit.MC0 = 1;
   while (tc4IsSyncing()); //wait until TC4 is done syncing 
} 

//Function that is used to check if TC4 is done syncing
//returns true when it is done syncing
bool tc4IsSyncing() {
  return TC4->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC4 and waits for it to be ready
void tc4StartCounter() {
  TC4->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tc4IsSyncing()); //wait until snyc'd
}

//Reset TC4 
void tc4Reset() {
  TC4->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tc4IsSyncing());
  while (TC4->COUNT16.CTRLA.bit.SWRST);
}

//disable TC4
void tc4Disable() {
  TC4->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc4IsSyncing());
}

//////////////////////////////////////////////////////////////////////////
//
// TIMER5 SPECIFIC FUNCTIONS FOLLOW
// you shouldn't change these unless you know what you're doing
//
//////////////////////////////////////////////////////////////////////////

//Configures the TC to generate output events at the sample frequency.
//Configures the TC in Frequency Generation mode, with an event output once

void tc5Configure(int sampleRate) {
   // Enable GCLK for TC4 and TC5 (timer counter input clock)
   GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TC4_TC5)) ;
   while (GCLK->STATUS.bit.SYNCBUSY);
  
   tc5Reset(); //reset TC5
  
   // Set Timer counter Mode to 16 bits
   TC5->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;
   // Set TC5 mode as match frequency
   TC5->COUNT16.CTRLA.reg |= TC_CTRLA_WAVEGEN_MFRQ;
   //set prescaler and enable TC5
   TC5->COUNT16.CTRLA.reg |= TC_CTRLA_PRESCALER_DIV1 | TC_CTRLA_ENABLE;
   //set TC5 timer counter based off of the system clock and the user defined sample rate or waveform
   TC5->COUNT16.CC[0].reg = (uint16_t) (SystemCoreClock / sampleRate - 1);
   while (tc5IsSyncing());
   
   // Configure interrupt request
   NVIC_DisableIRQ(TC5_IRQn);
   NVIC_ClearPendingIRQ(TC5_IRQn);
   NVIC_SetPriority(TC5_IRQn,0); // Set highest priority
   //NVIC_SetPriority(TC5_IRQn,(1 << __NVIC_PRIO_BITS) -2); // Set lowest priority -2
   NVIC_EnableIRQ(TC5_IRQn);
  
   // Enable the TC5 interrupt request
   TC5->COUNT16.INTENSET.bit.MC0 = 1;
   while (tc5IsSyncing()); //wait until TC5 is done syncing 
} 

//Function that is used to check if TC5 is done syncing
//returns true when it is done syncing
bool tc5IsSyncing() {
  return TC5->COUNT16.STATUS.reg & TC_STATUS_SYNCBUSY;
}

//This function enables TC5 and waits for it to be ready
void tc5StartCounter() {
  TC5->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE; //set the CTRLA register
  while (tc5IsSyncing()); //wait until snyc'd
}

//Reset TC5 
void tc5Reset() {
  TC5->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
  while (tc5IsSyncing());
  while (TC5->COUNT16.CTRLA.bit.SWRST);
}

//disable TC5
void tc5Disable() {
  TC5->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
  while (tc5IsSyncing());
}


