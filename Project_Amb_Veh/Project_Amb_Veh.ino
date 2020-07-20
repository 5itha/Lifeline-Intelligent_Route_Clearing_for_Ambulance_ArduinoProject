//***********Lifeline-Intelligent Route Clearance for Ambulance ***********
// 1. Uses Painless Mesh To connect nearby vehicles
// 2. If Ambulance Mode switch is ON Ambulance will Transmit Message
// 3. Json Is used for Message transfer
// 4. If ambulance Mode is active Traffic light will continuously GLOW Green signal on same side
//***********Ambulance and Vehicle communication in mesh***********
#include <painlessMesh.h>
#include <Ticker.h>
#include <Wire.h>
#include <MechaQMC5883.h>
#include <ShiftRegister74HC595.h>
#include <ArduinoJson.h>

// some gpio pin that is connected to an LED... 
#define   LED           16    // GPIO number of connected LED, ON ESP-12 IS GPI16   
#define   SW1           D6
#define   BLINK_PERIOD  3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for
#define   MESH_SSID       "Ambulance"
#define   MESH_PASSWORD   "password"
#define   MESH_PORT     5555

// Prototypes
void sendMessage();
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);
void delayReceivedCallback(uint32_t from, int32_t delay);
 
Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
MechaQMC5883  qmc;
ShiftRegister74HC595<1> sr(D3, D5, D4);
 
int x,y,z;                    // Axis data from Compass
int heading;                  // Direction From Compass
uint8_t LED_Compass[] = {0};
uint8_t AMB_Compass[] = {0xFF};
uint8_t Max_delay     = 5;
String dir = "";
int sec;
 
bool calc_delay = false;
SimpleList<uint32_t> nodes;
 
void sendMessage() ; // Prototype
Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval
 
// Task to blink the number of nodes
Task blinkNoNodes;
 
// Ticker to read Compass in every 100ms
Ticker ms_100;
 
bool onFlag = false;
 
void indication() {
  sec++;
  if(sec>59) sec = 0;
  //digitalWrite(Test,!digitalRead(Test));
  qmc.read(&x,&y,&z,&heading);
 
  if (heading > 354 || heading < 4)
  {
  dir = "NORTH";
  LED_Compass[0] = 0xEF;
  } else if (heading > 4 && heading < 41)
  {
  dir = "NORTH-EAST";
  LED_Compass[0] = 0xDF;
  } else if (heading > 41 && heading < 62)
  {
  dir = "EAST";
  LED_Compass[0] = 0xBF;
  } else if (heading > 62 && heading < 167)
  {
  dir = "SOUTH-EAST";
  LED_Compass[0] = 0x7F;
  } else if (heading > 167 && heading < 235)
  {
  dir = "SOUTH";
  LED_Compass[0] = 0xFE;
  } else if (heading > 235 && heading < 288)
  {
  dir = "SOTUH-WEST";
  LED_Compass[0] = 0xFD;
  } else if (heading > 288 && heading < 309)
  {
  dir = "WEST";
  LED_Compass[0] = 0xFB;
  } else if (heading > 309 && heading < 354)
  {
  dir = "NORTH-WEST";
  LED_Compass[0] = 0xF7;
  }
  //ph Serial.println(dir);
  if(AMB_Compass[0] != 0xFF){
  if((sec % 2) ){
      if(LED_Compass[0] != AMB_Compass[0])  LED_Compass[0] = LED_Compass[0] & AMB_Compass[0];
    else                                  LED_Compass[0] = 0xFF;
  }
  }
 
  sr.setAll(LED_Compass);
}
void setup() {
  Serial.begin(115200);
 
  pinMode(LED, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
 
  Wire.begin(D1, D2);
  qmc.init();
  qmc.setMode(Mode_Continuous,ODR_200Hz,RNG_2G,OSR_256);
 
mesh.setDebugMsgTypes(ERROR | DEBUG);  // set before init() so that you can see error messages
 
  mesh.init(MESH_SSID, MESH_PASSWORD, &userScheduler, MESH_PORT);
  mesh.onReceive(&receivedCallback);
  mesh.onNewConnection(&newConnectionCallback);
  mesh.onChangedConnections(&changedConnectionCallback);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);
  mesh.onNodeDelayReceived(&delayReceivedCallback);
 
  userScheduler.addTask( taskSendMessage );
  taskSendMessage.enable();
 
  blinkNoNodes.set(BLINK_PERIOD, (mesh.getNodeList().size() + 1) * 2, []() {
    // If on, switch off, else switch on
    if (onFlag)
      onFlag = false;
    else
      onFlag = true;
      blinkNoNodes.delay(BLINK_DURATION);
 
    if (blinkNoNodes.isLastIteration()) {
      // Finished blinking. Reset task for next run
      // blink number of nodes (including this node) times
        blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
      // Calculate delay based on current mesh time and BLINK_PERIOD
      // This results in blinks between nodes being synced
        blinkNoNodes.enableDelayed(BLINK_PERIOD -
            (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
    }
  });
  userScheduler.addTask(blinkNoNodes);
  blinkNoNodes.enable();
 
  ms_100.attach_ms(100, indication);
  randomSeed(analogRead(A0));
}
 
void loop() {
  mesh.update();
  digitalWrite(LED, !onFlag);
}
 
void sendMessage() {
   StaticJsonDocument<500> jsonTx;
  if(!digitalRead(SW1)){  jsonTx["Ambulance"] = "ON";  Max_delay = 1;}
  else              {  jsonTx["Ambulanc"] = "OFF"; Max_delay = 5;}
  jsonTx["Direction"] = dir;
  jsonTx["Compass"] = LED_Compass[0];
  String msg;
  serializeJson(jsonTx,msg);
 
  mesh.sendBroadcast(msg);
 
  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
    node++;
  }
  calc_delay = false;
  }
 
  Serial.printf("Transmit: %s\n", msg.c_str());
}
 
void receivedCallback(uint32_t from, String & msg) {
  StaticJsonDocument<200> jsonRx;
  DeserializationError error = deserializeJson(jsonRx, msg.c_str());
 
  // Test if parsing succeeds.
  if (error) {
    //Serial.print(F("deserializeJson() failed: "));
    //Serial.println(error.c_str());
  return;
  } else {
    if(jsonRx["Ambulance"] == "ON"){
    AMB_Compass[0] = jsonRx["Compass"];
      Serial.println("Ambullance Active"); 
  } else if(jsonRx["Ambulance"] == "OFF"){
      Serial.println("Ambullance Passive");
    AMB_Compass[0] = 0xFF;
  }
  
  }
  //Serial.printf("Received : %u msg=%s\n", from, msg.c_str());
}
 
 
void newConnectionCallback(uint32_t nodeId) {
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
  Serial.printf("--> startHere: New Connection, %s\n", mesh.subConnectionJson(true).c_str());
}
 
void changedConnectionCallback() {
  Serial.printf("Changed connections\n");
  // Reset blink task
  onFlag = false;
  blinkNoNodes.setIterations((mesh.getNodeList().size() + 1) * 2);
  blinkNoNodes.enableDelayed(BLINK_PERIOD - (mesh.getNodeTime() % (BLINK_PERIOD*1000))/1000);
 
  nodes = mesh.getNodeList();
 
  Serial.printf("Num nodes: %d\n", nodes.size());
  Serial.printf("Connection list:");
 
  SimpleList<uint32_t>::iterator node = nodes.begin();
  while (node != nodes.end()) {
    Serial.printf(" %u", *node);
  node++;
  }
  Serial.println();
  calc_delay = true;
}
 
void nodeTimeAdjustedCallback(int32_t offset) {
  Serial.printf("Adjusted time %u. Offset = %d\n", mesh.getNodeTime(), offset);
}
 
void delayReceivedCallback(uint32_t from, int32_t delay) {
  Serial.printf("Delay to node %u is %d us\n", from, delay);
}
