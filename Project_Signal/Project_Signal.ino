//***********TrafficSignal***********
#include <painlessMesh.h>
#include <Ticker.h>
#include <ShiftRegister74HC595.h>
#include <ArduinoJson.h>


// some gpio pin that is connected to an LED...
#define   LED             2       // GPIO number of connected LED, ON ESP-12 IS GPIO2
#define   Test           16
#define   BLINK_PERIOD    3000 // milliseconds until cycle repeat
#define   BLINK_DURATION  100  // milliseconds LED is on for
#define   MESH_SSID       "Ambulance"
#define   MESH_PASSWORD   "password"
#define   MESH_PORT       5555

// Prototypes
void sendMessage(); 
void receivedCallback(uint32_t from, String & msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback(); 
void nodeTimeAdjustedCallback(int32_t offset); 
void delayReceivedCallback(uint32_t from, int32_t delay);

Scheduler     userScheduler; // to control your personal task
painlessMesh  mesh;
ShiftRegister74HC595<1> sr(D3, D1, D2);

uint8_t N_E[] = {0xFF};
uint8_t S_W[] = {0xFF};
uint8_t Light[8][2] = {0xDD,0x7D,
                       0XDD,0xBB,
                       0xDD,0xD7,
                       0xDB,0xDB,
                       0XD7,0xDD,
                       0xBB,0xDD,
                       0x7D,0XDD,
                       0XBD,0XBD };
int ms,sec;
bool light_flag = false;
bool amb_flag   = false;
bool calc_delay = false;
SimpleList<uint32_t> nodes;

void sendMessage() ; // Prototype
Task taskSendMessage( TASK_SECOND * 1, TASK_FOREVER, &sendMessage ); // start with a one second interval

// Task to blink the number of nodes
Task blinkNoNodes;
bool onFlag = false;

// Ticker runs in every 100ms
Ticker ms_100;

void indication() {
  digitalWrite(Test,!digitalRead(Test));
  ms++;
  if(ms > 9){
    ms = 0;
    Serial.println(sec);
    switch(sec){
      case 0:                       // North side Green
      case 15:
        N_E[0] = Light[0][1];
        S_W[0] = Light[0][0]; light_flag = true;
        break;
      case 20:                      // N E Yellow
        N_E[0] = Light[1][1];
        S_W[0] = Light[1][0]; light_flag = true;
        break;
      case 22:                      // East side Green
      case 37:
        N_E[0] = Light[2][1];
        S_W[0] = Light[2][0]; light_flag = true;
        break;
      case 42:                      // S E Yellow
        N_E[0] = Light[3][1];
        S_W[0] = Light[3][0]; light_flag = true;
        break;
      case 44:                      // South side Green
      case 59:
        N_E[0] = Light[4][1];
        S_W[0] = Light[4][0]; light_flag = true;
        break;
      case 64:                      // S W Yellow
        N_E[0] = Light[5][1];
        S_W[0] = Light[5][0]; light_flag = true;
        break;
      case 66:                      // West side Green
      case 81:  
        N_E[0] = Light[6][1];
        S_W[0] = Light[6][0]; light_flag = true;
        break;
      case 86:                      // N W Yellow
        N_E[0] = Light[7][1];
        S_W[0] = Light[7][0]; light_flag = true;
        break;  
        
    }

    sec++;
    if(sec>87) sec = 0;
    
    if( light_flag){
      light_flag = false;
      sr.setAll(S_W);
      sr.setAll(N_E);
    }
    
  }
  //N_E[0]++;
  
}
void setup() {
  Serial.begin(115200);

  pinMode(LED, OUTPUT);
  
  N_E[0] = Light[0][1];
  S_W[0] = Light[0][0];
  sr.setAll(S_W);
  sr.setAll(N_E);
          
  //pinMode(Test, OUTPUT);
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
  String msg = "Hello from node ";
  msg += mesh.getNodeId();
  msg += " myFreeMemory: " + String(ESP.getFreeHeap());
  mesh.sendBroadcast(msg);

  if (calc_delay) {
    SimpleList<uint32_t>::iterator node = nodes.begin();
    while (node != nodes.end()) {
      mesh.startDelayMeas(*node);
      node++;
    }
    calc_delay = false;
  }

  //Serial.printf("Transmit: %s\n", msg.c_str());
  
  //taskSendMessage.setInterval( random(TASK_SECOND * 1, TASK_SECOND * 5));  // between 1 and 5 seconds
}


void receivedCallback(uint32_t from, String & msg) {
  //Serial.printf("Received %u msg=%s\n", from, msg.c_str());
  StaticJsonDocument<200> jsonRx;
  DeserializationError error = deserializeJson(jsonRx, msg.c_str());

  if (error) {
    //Serial.print(F("deserializeJson() failed: "));
    //Serial.println(error.c_str());
    return;
  } else {
    if(jsonRx["Ambulance"] == "ON"){
      //AMB_Compass[0] = jsonRx["Compass"];
      
      if(amb_flag == false){                      // Enable Yellow Once for Ambulance arrival
        Serial.println("Ambullance Active");
        amb_flag = true;
        if(jsonRx["Direction"] == "SOUTH")        sec = 85;
        else if(jsonRx["Direction"] == "WEST")    sec = 19;
        else if(jsonRx["Direction"] == "NORTH")   sec = 41;
        else if(jsonRx["Direction"] == "EAST")    sec = 63;
      } else {                                    // Green till Ambulance gone
        if(jsonRx["Direction"] == "SOUTH")        sec = 15;
        else if(jsonRx["Direction"] == "WEST")    sec = 37;
        else if(jsonRx["Direction"] == "NORTH")   sec = 59;
        else if(jsonRx["Direction"] == "EAST")    sec = 81;
      }
      

    } else if(jsonRx["Ambulance"] == "OFF"){
      if(amb_flag == true){
        Serial.println("Ambullance Passive");
        amb_flag = false;
      }
    } 
  }
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
