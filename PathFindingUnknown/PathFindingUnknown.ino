//define an environment of size m by n, free path 0, obstacle -1
//vertice marking
//start point assumed as bottom left vertice, end point given
//dijkstra's algorithm for path finding (least turning)
//retreat when encountering obstacle and update map

#include <stdio.h>
#include <limits.h>

/*
   Choose communication mode define here:
      I2C_MODE    : I2C mode, default pin: MU_SDA <==> ARDUINO_SDA, MU_SCL <==> ARDUINO_SCL
      SERIAL_MODE : Serial mode, default pin: MU_TX <==> ARDUINO_PIN3, MU_RX <==> ARDUINO_PIN2
*/
#define I2C_MODE
#define SERIAL_MODE
#include <I2Cdev.h>
#include "OpenCat.h"


/*
   Choose MU address here: 0x60, 0x61, 0x62, 0x63
          default address: 0x60
*/
#define MU_ADDRESS        0x50 //in later versions we set the I2C device to 0x50, 0x51, 0x52, 0x53
#define ALT_MU_ADDRESS    0x60

#include <Arduino.h>
#include <MuVisionSensor.h>

#ifdef I2C_MODE
#include <Wire.h>
#endif
#ifdef SERIAL_MODE
#include <SoftwareSerial.h>
#define TX_PIN 2
#define RX_PIN 3
SoftwareSerial mySerial(RX_PIN, TX_PIN);
#endif

MuVisionSensor *Mu;
MuVisionSensor Mu0(MU_ADDRESS);
MuVisionSensor Mu1(ALT_MU_ADDRESS);

int xCoord, yCoord; //the x y returned by the sensor
int xDiff, yDiff; //the scaled distance from the center of the frame
int currentX = 0, currentY = 0; //the current x y of the camera's direction in the world coordinate
int range = 10000; //the frame size 0~100 on X and Y direction
int skip = 500, counter; //an efforts to reduce motion frequency without using delay. set skip >1 to take effect
int i2cdelay = 3;

#define SKIP 3
#ifdef SKIP
byte updateFrame = 0;
#endif
byte firstValidJoint;
byte stage = 0;
char choice;
#define MAX_PERIOD 3
#define NORTH 1
#define EAST 2
#define SOUTH 3
#define WEST 4

const int N = 2;
const int M = 2;

int MAP[N*M] = {
0,0,
-1,0,
};

const int V = N*M;
int TARGET = 3;
int SRC = 0;
int GRAPH[V][V] = {0};
bool sptSet[V];
int INST[V] = {-1};
int parent[V] = {-1};
int PATH[V] = {-1};
int LEN;
int orientation = NORTH;
int PERIOD = 0;
int CURR_INST = 0;
int RETREAT = 0;
int prev_orientation[V];

void setup()
{
     // put your setup code here, to run once:
    Serial.begin(115200);
    uint8_t err = 0;
    #ifdef I2C_MODE
      Wire.begin();
      // initialized MU on the I2C port
      err = Mu0.begin(&Wire);
    #elif defined SERIAL_MODE
      mySerial.begin(9600);
      // initialized MU on the soft serial port
      err = Mu0.begin(&mySerial);
    #endif
      if (err == MU_OK) {
        Serial.println("MU initialized");
        Mu = &Mu0;
      } else {
        Serial.println("fail to initialize");
        err = Mu1.begin(&Wire);
        if (err == MU_OK) {
          Serial.println("MU initialized");
          Mu = &Mu1;
        }
        delay(1000);
      }
      strcpy(lastCmd, "");
      // enable vision: color
      (*Mu).VisionBegin(VISION_COLOR_DETECT);
      (*Mu).write(VISION_COLOR_DETECT, kLabel, MU_COLOR_WHITE);            // set detect color type: black
      (*Mu).CameraSetAwb(kLockWhiteBalance); 

      (*Mu).LsBegin(LS_PROXIMITY_ENABLE);
      // light set sensitivity, default value is kSensitivity2
      (*Mu).LsSetSensitivity(kSensitivity2);
      // enable white balance for color detection
      (*Mu).LsWhiteBalanceEnable();

      pwm.begin();
      pwm.setPWMFreq(60 * PWM_FACTOR); // Analog servos run at ~60 Hz updates

      for (byte i = 0; i < DOF; i++) {
        pulsePerDegree[i] = float(PWM_RANGE) / servoAngleRange(i);
        servoCalibs[i] = servoCalib(i);
        calibratedDuty0[i] =  SERVOMIN + PWM_RANGE / 2 + float(middleShift(i) + servoCalibs[i]) * pulsePerDegree[i]  * rotationDirection(i) ;
      }
     counter = 0;
     //  Let us create the example
    // graph discussed above
    delay(10);
    /*int graph[V][V] = {{0, 4, 0, 0, 0, 0, 0, 8, 0},
                       {4, 0, 8, 0, 0, 0, 0, 11, 0},
                        {0, 8, 0, 7, 0, 4, 0, 0, 2},
                        {0, 0, 7, 0, 9, 14, 0, 0, 0},
                        {0, 0, 0, 9, 0, 10, 0, 0, 0},
                        {0, 0, 4, 0, 10, 0, 2, 0, 0},
                        {0, 0, 0, 14, 0, 2, 0, 1, 6},
                        {8, 11, 0, 0, 0, 0, 1, 0, 7},
                        {0, 0, 2, 0, 0, 0, 6, 7, 0}
                    };*/
    Serial.println("huh");
    loadGraph();
    Serial.println("hey");
    dijkstra(GRAPH, 0);
    Serial.println("hi");
    path2instruction();
}

void loop(){
  if (!(counter++ % skip)) {
      //Serial.println("entered");
      uint8_t proximity = (*Mu).LsReadProximity();
      if (proximity >= 175 || proximity == 0){
        RETREAT = 1;
      }
  }
  if (RETREAT){
    orientation = prev_orientation[CURR_INST];
    if (PERIOD == 0){
      RETREAT = 1;
      CURR_INST--;
      SRC = PATH[CURR_INST];
      MAP[SRC] = -1;
      loadGraph();
      dijkstra(GRAPH, SRC);
      path2instruction();
    }
    else{
      int b = getRetreat(INST[CURR_INST-1]);
      startInstruction(b);
    }
  }
  else{
    if (INST[CURR_INST] != -1){
      if (PERIOD == MAX_PERIOD){
        PERIOD = 0;
        CURR_INST++;
      }
      startInstruction(INST[CURR_INST]);
    }
    else{
      startInstruction(6);
    }
  }
  motionBlock();
}

int getRetreat(int b){
  int a;
  switch (b){
    case 0: {a = 3;break;}
    case 1: {a = 4;break;}
    case 2: {a = 5;break;}
    case 3: {a = 0;break;}
    case 4: {a = 1;break;}
    case 5: {a = 2;break;}
  }
  return a;
}
// A utility function to find the vertex with minimum distance value, from
// the set of vertices not yet included in shortest path tree
int minDistance(int dist[], bool sptSet[])
{
   
    // Initialize min value
    int min = INT_MAX, min_index;
 
    for (int v = 0; v < V; v++)
        if (sptSet[v] == false && dist[v] <= min)
            min = dist[v], min_index = v;
 
    return min_index;
}

void checkLeft(int i, int j){
  if (MAP[j] == -1){
    GRAPH[i][j] = 0;
  }
  else{
    GRAPH[i][j] = 1;
  }
}

void checkRight(int i, int j){
  if (MAP[j] == -1){
    GRAPH[i][j] = 0;
  }
  else{
    GRAPH[i][j] = 1;
  }
}

void checkUp(int i, int j){
  if (MAP[j] == -1){
    GRAPH[i][j] = 0;
  }
  else{
    GRAPH[i][j] = 1;
  }
}

void checkDown(int i, int j){
  if (MAP[j] == -1){
    GRAPH[i][j] = 0;
  }
  else{
    GRAPH[i][j] = 1;
  }
}

   
// Function to print shortest
// path from source to j
// using parent array
int getPath(int parent[], int j, int i)
{
       
    // Base Case : If j is source
    if (parent[j] == - 1)
        return -1;
   
    PATH[i] = getPath(parent, parent[j], i+1);

    return j;
}
   
// A utility function to print 
// the constructed distance
// array
/*
int printSolution(int dist[], int n, 
                      int parent[])
{
    int src = 0;
    //Serial.println("Vertex\t Distance\tPath");
    for (int i = 1; i < V; i++)
    {
        //Serial.println(src, i, dist[i], src);
        printPath(parent, i);
        Serial.println();
    }
}
*/
   
// Funtion that implements Dijkstra's
// single source shortest path
// algorithm for a graph represented
// using adjacency matrix representation
void dijkstra(int graph[V][V], int src)
{
       
    // The output array. dist[i]
    // will hold the shortest
    // distance from src to i
    int dist[V]; 
   
    // sptSet[i] will true if vertex
    // i is included / in shortest
    // path tree or shortest distance 
    // from src to i is finalized
    bool sptSet[V];
   
    // Parent array to store
    // shortest path tree
   
    // Initialize all distances as 
    // INFINITE and stpSet[] as false
    for (int i = 0; i < V; i++)
    {
        parent[0] = -1;
        dist[i] = INT_MAX;
        sptSet[i] = false;
    }
   
    // Distance of source vertex 
    // from itself is always 0
    dist[src] = 0;
   
    // Find shortest path
    // for all vertices
    for (int count = 0; count < V - 1; count++)
    {
        // Pick the minimum distance
        // vertex from the set of
        // vertices not yet processed. 
        // u is always equal to src
        // in first iteration.
        int u = minDistance(dist, sptSet);
   
        // Mark the picked vertex 
        // as processed
        sptSet[u] = true;
   
        // Update dist value of the 
        // adjacent vertices of the
        // picked vertex.
        for (int v = 0; v < V; v++)
   
            // Update dist[v] only if is
            // not in sptSet, there is
            // an edge from u to v, and 
            // total weight of path from
            // src to v through u is smaller
            // than current value of
            // dist[v]
            if (!sptSet[v] && graph[u][v] &&
                dist[u] + graph[u][v] < dist[v])
            {
                parent[v] = u;
                dist[v] = dist[u] + graph[u][v];
            } 
    }
   
    // print the constructed
    // distance array
    //printSolution(dist, V, parent);
    PATH[0] = getPath(parent, TARGET, 1);

    LEN = 0;

    while (PATH[LEN] != -1){
      LEN++;
    }
    int tmp;
    for (int i = 0; i<LEN/2; i++){
      tmp = PATH[i];
      PATH[i] = PATH[LEN-i-1];
      PATH[LEN-i-1] = tmp; 
    }
}

//["wkF","wkL","wkR", "bk", "bkL", "bkR"]
void path2instruction(){
  int idx;
  int curr = SRC;
  for (int i=0; i < LEN; i++){
    idx = PATH[i];
    prev_orientation[i] = orientation;
    if (idx == curr + 1){
      if (orientation == NORTH){
        INST[i] = 2;
        orientation = EAST;
        
      }
      else if (orientation == SOUTH){
        INST[i] = 1;
        orientation = EAST;
      }
      else if (orientation == EAST){
        INST[i] = 0;
      }
      else if (orientation == WEST){
        INST[i] = 3;
      }
    }
    if (idx == curr - 1){
      if (orientation == NORTH){
        INST[i] = 1;
        orientation = WEST;
      }
      else if (orientation == SOUTH){
        INST[i] = 2;
        orientation = WEST;
      }
      else if (orientation == EAST){
        INST[i] = 3;
      }
      else if (orientation == WEST){
        INST[i] = 0;
      }
    }
    if (idx == curr + M){
      if (orientation == NORTH){
        INST[i] = 3;
      }
      else if (orientation == SOUTH){
        INST[i] = 0;
      }
      else if (orientation == EAST){
        INST[i] = 2;
        orientation = SOUTH;
      }
      else if (orientation == WEST){
        INST[i] = 1;
        orientation = SOUTH;
      }
    }
    if (idx == curr - M){
      if (orientation == NORTH){
        INST[i] = 0;
      }
      else if (orientation == SOUTH){
        INST[i] = 3;
      }
      else if (orientation == EAST){
        INST[i] = 1;
        orientation = NORTH;
      }
      else if (orientation == WEST){
        INST[i] = 2;
        orientation = NORTH;
      }
    }
    curr = idx;
  }
}

void loadGraph(){
  for (int i = 0; i < N*M; i++){
    for (int j = 0; j < N*M; j++){
      if (MAP[i] == -1){
        GRAPH[i][j] = 0;
      }
      else if (i-1 == j && i % N != 0){
        checkLeft(i,j);
      }
      else if (i+1 == j && i % N != 1){
        checkRight(i,j);
      }
      else if (i-M == j && i >= N){
        checkUp(i,j);
      }
      else if (i+M == j && i < (N-1)*M){
        checkDown(i,j);
      }
      else{
        GRAPH[i][j] = 0;
      }
    }
  }
}

void motionBlock(){
#ifndef HEAD  //skip head
      if (jointIdx == 0)
        jointIdx = 2;
#endif
#ifndef TAIL  //skip tail
      if (jointIdx == 2)
        jointIdx = 4;
#endif
      if (motion.period != 1) {//skip non-walking DOF
        if (jointIdx < 4)
          jointIdx = 4;

      }
#if WALKING_DOF==8 //skip shoulder roll 
      if (jointIdx == 4)
        jointIdx = 8;
#endif
      int dutyIdx = timer * WALKING_DOF + jointIdx - firstValidJoint;
      calibratedPWM(jointIdx, motion.dutyAngles[dutyIdx] );
      //Serial.println("cali");
      jointIdx++;

      if (jointIdx == DOF) {
        jointIdx = 0;
#ifdef SKIP
      if (updateFrame++ == SKIP) {
          updateFrame = 0;
#endif
          timer = (timer + 1) % motion.period;
          if (timer == 0){
            if  (RETREAT){
              PERIOD--;
            }
            else{
              PERIOD++;
            }
          }
          Serial.println(timer);
#ifdef SKIP
        }
#endif
      }
}

void startInstruction(int a){
  char cmd[CMD_LEN] = {};
  switch (a){
    case 0: {strcpy(cmd, "wkF");break;}
    case 1: {strcpy(cmd, "wkL");break;}
    case 2: {strcpy(cmd, "wkR");break;}
    case 3: {strcpy(cmd, "bk");break;}
    case 4: {strcpy(cmd, "bkL");break;}
    case 5: {strcpy(cmd, "bkR");break;}
    case 6: {strcpy(cmd, "balance");break;}
  }
  if (strcmp(lastCmd, cmd)){
    motion.loadBySkillName(cmd);
    //Serial.println(motion.period);
    #ifdef DEVELOPER
        PTF("free memory: ");
        PTL(freeMemory());
#endif
        timer = 0;
        counter = 0;
    if (strcmp(cmd, "balance") && strcmp(cmd, "lifted") && strcmp(cmd, "dropped") )
         strcpy(lastCmd, cmd);
        
        // if posture, start jointIdx from 0
        // if gait, walking DOF = 8, start jointIdx from 8
        //          walking DOF = 12, start jointIdx from 4
        firstValidJoint = (motion.period == 1) ? 0 : DOF - WALKING_DOF;
        jointIdx = firstValidJoint;
        transform(motion.dutyAngles, 1, 1, firstValidJoint);
        //Serial.println("transformed");
        if (!strcmp(cmd, "rest")) {
          shutServos();
          token = T_REST;
        }
  }
}
