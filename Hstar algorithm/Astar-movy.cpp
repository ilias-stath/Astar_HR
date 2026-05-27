#include <iostream>
#include "Astar_HR.h"
#include <set>
#include <vector>
#include <cmath>
#include <cstdint> 
#include <cstring> 
#include <cstdlib>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <string>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <csignal>
#include <errno.h>
#include <fcntl.h>

using namespace std;
using steady_clk  = std::chrono::steady_clock;

#define USHRT_MAX 65535

std::mutex dataMutex;
std::condition_variable pathRequestCv;  // Wakes main thread when message 422 arrives

// os.system('sudo ip link set can0 type can bitrate 250000')
// os.system('sudo ifconfig can0 up') 
// os.system('sudo ifconfig can0 down')


//------------------Astar related functions------------------//
short find_path(float** x, float** y);
unsigned short Obstacles(unsigned short** obstacles, short x, short y, unsigned short xMin, unsigned short yMin, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl); // True horizontal, false vertical
unsigned short stageOb(unsigned short** obstacles, bool team, unsigned short xMin, unsigned short yMin, unsigned short xMax, unsigned short yMax, uint8_t tol, uint8_t Rs); // True blue, false yellow
unsigned short enemyOb(unsigned short** obstacles, short x, short y, unsigned short xMin, unsigned short yMin, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl);
short constrain(short num, short downLimit, short upLimit);
bool MovePoint(unsigned short& px, unsigned short& py, unsigned short obsLeft, unsigned short obsBottom, unsigned short obsRight, unsigned short obsTop, uint8_t step, uint8_t xMin, uint8_t yMin, unsigned short xMax, uint8_t yMax);

// struct Astar_approach {
//     int dx;
//     int dy;
// };

// // Αντιστοίχιση βάσει των οδηγιών σου:
// const Astar_approach offsets[] = {
//     {0, 0},   // 0: τίποτα
//     {0, 20},   // 1: y += 5
//     {20, 0},  // 2: x += 5
//     {0, -20},  // 3: y -= 5
//     {-20, 0}    // 4: x -= 5
// };

//------------------Astar related variables------------------//
const uint8_t pickUpsDefault[8] = {2, 1, 1, 2, 2, 1, 1, 2};
uint8_t pickUps[8] = {2, 1, 1, 2, 2, 1, 1, 2};
// const uint8_t pickUpsDefault[8] = {2, 1, 0, 0, 2, 1, 0, 0};
// uint8_t pickUps[8] = {2, 1, 0, 0, 2, 1, 0, 0};
bool placeAreas[10] = {false, false, false, false, false, false, false, false, false, false};
short pickUpLocation[16] = {17,40, 110,17, 190,17, 282,40, 17,120, 115,80, 185,80, 282,120};
short placeAreasLocation[20] = {70,10, 150,10, 230,10, 10,80, 80,80, 150,80, 220,80, 290,80, 125,145, 175,145};

//------------------CAN related functions------------------//
int setupCAN(const char* iface);
void sendCANmsg(int socket, int msgID, int num1 = -1, int num2 = -1, int num3 = -1, int num4 = -1);
void readCAN(int socket);

//------------------CAN related variables------------------//
volatile sig_atomic_t running = 1;
int bus = -1;

//------------------OS related functions------------------//
void signalHandler(int signum) {
    std::cout << "\nInterrupt received (Ctrl+C). Shutting down..." << std::endl;
    running = 0;
}

//------------------Shared variables------------------//
// Thread-safe shared data
int enemyPos[2] = { -6900, -6900}; // x,y - protected by dataMutex
float robotPos[2] = {385, 1800}; // x,y - protected by dataMutex
float goal[2] = {190, 125}; // x,y - protected by dataMutex
bool side = false; // protected by dataMutex
bool pathRequest = false; // protected by dataMutex, triggered by CAN message 422
bool withEnemy = true;
// uint8_t headingChoice = 0;


int main(){
    float* xPoints = nullptr;
    float* yPoints = nullptr;
    short len = 0;
    // auto next = steady_clk::now();
    // auto start = steady_clk::now();
    bool find_local = false;

    bus = setupCAN("can0");
    if (bus < 0) return 1;

    signal(SIGINT, signalHandler);

    std::thread canThread(readCAN, bus);


    while(running){
        // Wait for CAN message 422 (event-driven, no polling)
        {
            std::unique_lock<std::mutex> lock(dataMutex);
            // Block until pathRequest is true, with 100ms timeout to recover from glitches
            pathRequestCv.wait_for(lock, std::chrono::milliseconds(100), []{ return pathRequest; });
            find_local = pathRequest;
            if (find_local) {
                pathRequest = false;  // Reset flag for next request
            }
        }

        if(find_local){
            // sendCANmsg(bus, 422, 0);
            len = find_path(&xPoints, &yPoints);


            if(len > 0){
                cout << "len->" << len << endl;
                cout << "---Astar points---" << endl;
                for (unsigned short i = 0; i < len; i++) {
                    cout << xPoints[i] << ", " << yPoints[i] << ", ";
                }
                cout << endl;
                sendCANmsg(bus, 421, len);
                cout << len << endl;
                for (unsigned short i = 0; i + 1< len; i+=2) {
                    sendCANmsg(bus, 420, (int)(xPoints[i] * 100), (int)(yPoints[i] * 100), (int)(xPoints[i+1] * 100), (int)(yPoints[i+1] * 100));
                }
                if (len % 2 != 0) {
                    sendCANmsg(bus, 420, (int)(xPoints[len - 1] * 100), (int)(yPoints[len - 1] * 100), 0);
                }else{
                    sendCANmsg(bus, 420, 0);
                }

                free(xPoints);
                free(yPoints);
                xPoints = nullptr;
                yPoints = nullptr;
                
                cout << "Astar sent with CAN" << endl;
            }else{
                cout << "len->" << len << endl;
                cout << "Astar failure but CAN sent" << endl;
                sendCANmsg(bus, 421, len);
            }

            find_local = false;
        }

    }
    

    canThread.join(); 
    close(bus);
    cout << "CAN closed cleanly." << endl;
}

//------------------Astar related functions------------------//
short find_path(float** x, float** y){
    float* xPoints = nullptr;
    float* yPoints = nullptr;
    short len = 0;
    bool NoStar = false;
    // short  xgMil, ygMil;
    short  xen, yen;
    unsigned short xs, ys, xg, yg, xtmp, ytmp, setTmp = USHRT_MAX, tempSet;
    int tempx, tempy, tempstep;
    double enemyDist = 0;
    unsigned short k = 0;
    unsigned short lenObs = 0;
    unsigned short TotLenObs = 0;
    unsigned short xMax = 278; // 300 - 22
    unsigned short* obstacles = nullptr;
    unsigned short* TotObs = nullptr;
    unsigned short* tempObs = nullptr;
    uint8_t yMax = 178, xl, yl, step = 1, tol = 0, Rsize = 17, xMin = 22, yMin = 22; 
    // 1 -> every 1 cm, 5 -> every 5 cm, 10 -> every 10cm 

    // Mutex needed variables
    bool withEnemyCopy = true;
    bool side_copy;
    uint8_t pickUpsSnapshot[8];
    bool placeAreasSnapshot[10];

    // Point reorienting variables
    uint8_t goalAdjustmentCount = 0;
    uint8_t startAdjustmentCount = 0;
    const uint8_t maxAdjustments = 10; // Safeguard against infinite loops

    // // Path end offset
    // unsigned short xgOffset=0, ygOffset=0;
    // uint8_t headingChoiceCopy = 0;

    

    // Snapshot shared arrays once per path request to avoid races with CAN updates.
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        memcpy(pickUpsSnapshot, pickUps, sizeof(pickUpsSnapshot));
        memcpy(placeAreasSnapshot, placeAreas, sizeof(placeAreasSnapshot));
    }

    // Placement Areas
    // When you place the sizes, you place from center to edge, not edge to edge
    xl = 13 + Rsize + tol;
    yl = 13 + Rsize + tol;
    for (unsigned short i = 0; i < 20; i += 2) {
        lenObs = 0;  // Reset lenObs for each iteration
        if (placeAreasSnapshot[k]) {
            lenObs = Obstacles(&obstacles, placeAreasLocation[i], placeAreasLocation[i + 1], xMin, yMin, xMax, yMax, xl, yl);
            cout << "Place area " << k << " is occupied" << endl;
        }
        else {
            cout << "Place area " << k << " is free" << endl;
            obstacles = nullptr;  // Ensure obstacles is null when not used
        }
        k++;


        tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (tempObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
        }

        if (TotLenObs > 0) {
            memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
            free(TotObs);
        }

        TotLenObs += lenObs;

        TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (TotObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
        }

        if (TotLenObs - lenObs > 0) {
            memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
        }

        memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

        free(tempObs);
        free(obstacles);
        tempObs = nullptr;
        obstacles = nullptr;

    }
    k = 0;


    // Collectable Areas
    // When you place the sizes, you place from center to edge, not edge to edge
    xl = 11 + Rsize + tol;
    yl = 8 + Rsize + tol;
    for (unsigned short i = 0; i < 16; i += 2) {
        cout << "i = " << i << " ->  ";
        lenObs = 0;  // Reset lenObs for each iteration
        if (pickUpsSnapshot[k] == 2) {
            cout << "Pick-up " << k << " is vertical"  << endl;
            lenObs = Obstacles(&obstacles, pickUpLocation[i], pickUpLocation[i + 1], xMin, yMin, xMax, yMax, yl, xl); // vertical
        }
        else if (pickUpsSnapshot[k] == 1) {
            cout << "Pick-up " << k << " is horizontal"  << endl;
            lenObs = Obstacles(&obstacles, pickUpLocation[i], pickUpLocation[i + 1], xMin, yMin, xMax, yMax, xl, yl); // horizontal
        }
        else {
            cout << "Pick-up " << k << " is taken" << endl;
            obstacles = nullptr;  // Ensure obstacles is null when not used
        }
        k++;


        tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (tempObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
        }

        if (TotLenObs > 0) {
            memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
            free(TotObs);
        }

        TotLenObs += lenObs;

        TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (TotObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
        }

        if (TotLenObs - lenObs > 0) {
            memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
        }

        memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

        free(tempObs);
        free(obstacles);
        tempObs = nullptr;
        obstacles = nullptr;

    }

    // Enemy Obstacle
    // When you place the sizes, you place from center to edge, not edge to edge
    // enemyPos[0] = 0;
    // enemyPos[1] = 0;

    cout << endl;


    // Enemy Obstacle
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        enemyDist = sqrt(pow(enemyPos[0] - robotPos[0], 2) + pow(enemyPos[1] - robotPos[1], 2));
        enemyDist = enemyDist/10;
        
        xen = enemyPos[0]/10;
        yen = enemyPos[1]/10;
        withEnemyCopy = withEnemy;
    }

    cout << "withEnemy->" << withEnemyCopy << endl;

    if(withEnemyCopy && !(xen < 0 || yen < 0)){
        cout << "Using enemy!!" << endl;
        cout << "Xen -> " << xen << " ,  Yen -> " << yen << endl;

        yl = 30 + Rsize + tol;
        xl = 30 + Rsize + tol;

        // Dyanmically shrink enemy size if we are too close, avoid the safety size because it will return Astar failure
        if(int(enemyDist) < xl){
            yl = xl = int(enemyDist) - 5;
        }

        lenObs = enemyOb(&obstacles, xen, yen, xMin, yMin, xMax, yMax, xl, yl);

        tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (tempObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
        }

        if (TotLenObs > 0) {
            memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
            free(TotObs);
        }

        TotLenObs += lenObs;

        TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (TotObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
        }

        if (TotLenObs - lenObs > 0) {
            memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
        }

        memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

        free(tempObs);
    }else if(xen < 0 || yen < 0){
        cout << endl << "NOT USING ENEMY, because enemy pos not initialized!!" << endl;
    }else{
        cout << endl << "NOT USING ENEMY, because of request to not use!!" << endl;
    }
    


    // Stage Obstacle
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        side_copy = side;
    }
    lenObs = stageOb(&obstacles, side_copy, xMin, yMin, xMax, yMax, tol, Rsize);


    tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
    if (tempObs == nullptr) {
        cout << "Memory allocation failed!" << endl;
    }

    if (TotLenObs > 0) {
        memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
        free(TotObs);
    }

    TotLenObs += lenObs;

    TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
    if (TotObs == nullptr) {
        cout << "Memory allocation failed!" << endl;
    }

    if (TotLenObs - lenObs > 0) {
        memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
    }

    memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

    free(tempObs);
    free(obstacles);
    tempObs = nullptr;
    obstacles = nullptr;


    cout << endl << endl << "END OBSTACLES CREATION" << endl << endl;

    for (int i = 0; i < TotLenObs; i += 2) {
        cout << TotObs[i] << ", " << TotObs[i + 1] << ", ";
    }

    cout << endl;

    cout << "Starting" << endl;
    cout << "Number of Obstacles -> " << TotLenObs << endl;
    cout << "Memory of Obstacles with int -> " << TotLenObs * sizeof(int) << endl;
    cout << "Memory of Obstacles witn short -> " << TotLenObs * sizeof(unsigned short) << endl;

    // Read shared variables with mutex protection
    {
        std::lock_guard<std::mutex> lock(dataMutex);
        xg = goal[0];
        yg = goal[1];
        xs = robotPos[0];
        ys = robotPos[1];
    }

    // cout << "\n--------------------------\nxs:" << xs << " ,ys:" << ys << " ,xg:" << xg << " ,yg:" << yg << "\n--------------------------" << endl;

    // {
    //     std::lock_guard<std::mutex> lock(dataMutex);
    //     headingChoiceCopy = headingChoice;
    // }

    // xgOffset = xg;
    // ygOffset = yg;
    // xg += offsets[headingChoiceCopy].dx;
    // yg += offsets[headingChoiceCopy].dy;
    // cout << "Heading choice -> " << headingChoiceCopy <<  "  ,  xg -> " << xg << " ,  yg ->  " << yg << " ,  xgOffset -> " << xgOffset << " ,  ygOffset ->  " << ygOffset << endl;


    //xg = 2400;
    //yg = 1400;
    // xgMil = xg*10;
    // ygMil = yg*10;
    tempstep = step;
    tempx = xg;
    tempy = yg;
    if(tempx%tempstep == 0){
        xg = tempx;
    }else{
        if ((tempx % tempstep) < (tempstep / 2.0)) {
            xg = tempx - tempx%tempstep;
        } else {
            xg = tempx + (tempstep - tempx%tempstep);
        }
    }

    if(tempy%tempstep == 0){
        yg = tempy;
    }else{
        if ((tempy % tempstep) < (tempstep / 2.0)) {
            yg = tempy - tempy%tempstep;
        } else {
            yg = tempy + (tempstep - tempy%tempstep);
        }
    }

    // tempx = xgOffset;
    // tempy = ygOffset;
    // if(tempx%tempstep == 0){
    //     xgOffset = tempx;
    // }else{
    //     if ((tempx % tempstep) < (tempstep / 2.0)) {
    //         xgOffset = tempx - tempx%tempstep;
    //     } else {
    //         xgOffset = tempx + (tempstep - tempx%tempstep);
    //     }
    // }

    // if(tempy%tempstep == 0){
    //     ygOffset = tempy;
    // }else{
    //     if ((tempy % tempstep) < (tempstep / 2.0)) {
    //         ygOffset = tempy - tempy%tempstep;
    //     } else {
    //         ygOffset = tempy + (tempstep - tempy%tempstep);
    //     }
    // }


    tempx = static_cast<int>(std::round(robotPos[0] / 10.0));
    tempy = static_cast<int>(std::round(robotPos[1] / 10.0));
    xtmp = 0;
    ytmp = 0;
    tempSet = 0;

    cout << "x/10 = " << tempx << "  ,  y/10 = " << tempy << endl;

    //--- Left Down ---//
    if(tempx%tempstep == 0){
        xtmp = tempx;
    }else{
        if ((tempx % tempstep) < (tempstep / 2.0)) {
            xtmp = tempx - tempx%tempstep;
        } else {
            xtmp = tempx + (tempstep - tempx%tempstep);
        }
    }
    
    if(tempy%tempstep == 0){
        ytmp = tempy;
    }else{
        if ((tempy % tempstep) < (tempstep / 2.0)) {
            ytmp = tempy - tempy%tempstep;
        } else {
            ytmp = tempy + (tempstep - tempy%tempstep);
        }
    }

    tempSet = sqrt(pow(xg - xtmp, 2) + pow(yg - ytmp, 2));
    if(setTmp > tempSet){
        setTmp = tempSet;
        xs = xtmp;
        ys = ytmp;
    }

    cout << "Left_Down ->  x = " << xtmp << "  ,  y = " << ytmp << endl;

    //-- Left Up ---//
    if(tempx%tempstep == 0){
        xtmp = tempx;
    }else{
        if ((tempx % tempstep) < (tempstep / 2.0)) {
            xtmp = tempx - tempx%tempstep;
        } else {
            xtmp = tempx + (tempstep - tempx%tempstep);
        }
    }
    
    if(tempy%tempstep == 0){
        ytmp = tempy;
    }else{
        if ((tempy % tempstep) < (tempstep / 2.0)) {
            ytmp = tempy - tempy%tempstep;
        } else {
            ytmp = tempy + (tempstep - tempy%tempstep);
        }
    }

    tempSet = sqrt(pow(xg - xtmp, 2) + pow(yg - ytmp, 2));
    if(setTmp > tempSet){
        setTmp = tempSet;
        xs = xtmp;
        ys = ytmp;
    }

    cout << "Left_Up ->  x = " << xtmp << "  ,  y = " << ytmp << endl;

    //-- Right Down ---//
    if(tempx%tempstep == 0){
        xtmp = tempx;
    }else{
        if ((tempx % tempstep) < (tempstep / 2.0)) {
            xtmp = tempx - tempx%tempstep;
        } else {
            xtmp = tempx + (tempstep - tempx%tempstep);
        }
    }
    
    if(tempy%tempstep == 0){
        ytmp = tempy;
    }else{
        if ((tempy % tempstep) < (tempstep / 2.0)) {
            ytmp = tempy - tempy%tempstep;
        } else {
            ytmp = tempy + (tempstep - tempy%tempstep);
        }
    }

    tempSet = sqrt(pow(xg - xtmp, 2) + pow(yg - ytmp, 2));
    if(setTmp > tempSet){
        setTmp = tempSet;
        xs = xtmp;
        ys = ytmp;
    }

    cout << "Right_Up ->  x = " << xtmp << "  ,  y = " << ytmp << endl;

    //-- Right Up ---//
    if(tempx%tempstep == 0){
        xtmp = tempx;
    }else{
        if ((tempx % tempstep) < (tempstep / 2.0)) {
            xtmp = tempx - tempx%tempstep;
        } else {
            xtmp = tempx + (tempstep - tempx%tempstep);
        }
    }
    
    if(tempy%tempstep == 0){
        ytmp = tempy;
    }else{
        if ((tempy % tempstep) < (tempstep / 2.0)) {
            ytmp = tempy - tempy%tempstep;
        } else {
            ytmp = tempy + (tempstep - tempy%tempstep);
        }
    }

    tempSet = sqrt(pow(xg - xtmp, 2) + pow(yg - ytmp, 2));
    if(setTmp > tempSet){
        setTmp = tempSet;
        xs = xtmp;
        ys = ytmp;
    }

    cout << "Right_Down ->  x = " << xtmp << "  ,  y = " << ytmp << endl;

    //------------------------------------------------------------------------------------------------//

    cout << "xs:" << xs << " ,ys:" << ys << " ,xg:" << xg << " ,yg:" << yg << endl;


    if ((xg == xs && yg == ys) || (xg == robotPos[0] / 10 && yg == robotPos[1] / 10)) {
        cout << "\n----------\nSAME GOAL AS START.... NO ASTAR\n----------" << endl;
        NoStar = true;

    } else if ((xg < xMin || yg < yMin) || (xg > xMax || yg > yMax)) {
        cout << "\n----------\nGOAL OUTSIDE GRID\n----------" << endl;
        NoStar = true;

    } else {
        for (unsigned short k = 0; k < TotLenObs; k+=4) {
            cout << "Index (k) -> " << k << endl;
            

            if ((xg >= TotObs[k] && xg <= TotObs[k + 2]) && (yg >= TotObs[k + 1] && yg <= TotObs[k + 3])) {
                cout << "\n----------\nGOAL IN OBSTACLE....RECOMPUTING GOAL... k -> " << k << " , obstacle is (k/4) -> " << (k/4) << "     ,    TotObs[k] -> " << TotObs[k] << "  ,  TotObs[k+1] -> " << TotObs[k+1] << "  ,  TotObs[k+2] -> " << TotObs[k+2] << "  ,  TotObs[k+3] -> " << TotObs[k+3] << "\n----------" << endl;
                
                if(withEnemyCopy && k == TotLenObs - 8){
                    cout << "Goal is on enemy, don't recompute" << endl;
                    NoStar = true;
                    break;
                }
                
                if (!MovePoint(xg, yg, TotObs[k], TotObs[k + 1], TotObs[k + 2], TotObs[k + 3], step, xMin, yMin, xMax, yMax)) {
                    cout << "New goal position outside grid bounds after adjustment. New goal moved to (" << xg << ", " << yg << ")" << endl;
                    NoStar = true;
                    break;
                }
                
                cout << "Original goal in obstacle at k -> " << (k/4) << ". New goal moved to (" << xg << ", " << yg << ")" << endl;

                goalAdjustmentCount++;
                if (goalAdjustmentCount >= maxAdjustments) {
                    cout << "Max goal adjustments reached, exiting..." << endl;
                    NoStar = true;
                    break;
                }

                if (xg == xs && yg == ys) {
                    cout << "\n----------\nSAME GOAL AS START AFTER ADJUSTMENT.... NO ASTAR\n----------" << endl;
                    NoStar = true;
                    break;
                }
                k = 0;

            } else if ((xs >= TotObs[k] && xs <= TotObs[k + 2]) && (ys >= TotObs[k + 1] && ys <= TotObs[k + 3])) {
                cout << "\n----------\nSTART IN OBSTACLE....RECOMPUTING START... k -> " << k << "     ,    TotObs[k] -> " << TotObs[k] << "  ,  TotObs[k+1] -> " << TotObs[k+1] << "  ,  TotObs[k+2] -> " << TotObs[k+2] << "  ,  TotObs[k+3] -> " << TotObs[k+3] << "\n----------" << endl;
                
                if (!MovePoint(xs, ys, TotObs[k], TotObs[k + 1], TotObs[k + 2], TotObs[k + 3], step, xMin, yMin, xMax, yMax)) {
                    cout << "New start position outside grid bounds after adjustment. New goal moved to (" << xg << ", " << yg << ")" << endl;
                    NoStar = true;
                    break;
                }

                cout << "Original start in obstacle at k -> " << (k/4) << ". New start moved to (" << xs << ", " << ys << ")" << endl;

                startAdjustmentCount++;
                if (startAdjustmentCount >= maxAdjustments) {
                    cout << "Max start adjustments reached, exiting" << endl;
                    NoStar = true;
                    break;
                }

                if (xg == xs && yg == ys) {
                    cout << "\n----------\nSAME GOAL AS START AFTER ADJUSTMENT.... NO ASTAR\n----------" << endl;
                    NoStar = true;
                    break;
                }

                k = 0;

            } else {
                cout << "\n----------\nClear from obstacle k -> " << k << "\n----------" << endl;
            }
        }
    }

    //free(TotObs);
    //TotLenObs = 0;
    if(!NoStar){
        // Astar_HR astar(40, 172, 190, 130, xMin, yMin, xMax, yMax, TotObs, TotLenObs, true, step);
        // Astar_HR path(xs, ys, xg, yg, xgOffset, ygOffset, xMin, yMin, xMax, yMax, TotObs, TotLenObs, true, step); // true for bezier-resample-RDP, false for not
        Astar_HR path(xs, ys, xg, yg, xMin, yMin, xMax, yMax, TotObs, TotLenObs, true, step); // true for bezier-resample-RDP, false for not
        
        // Record start time
        auto start = std::chrono::high_resolution_clock::now();

        len = path.pathGeneration();

        free(TotObs);
        TotObs = nullptr;
        TotLenObs = 0;
        lenObs = 0;

        if (len == -2) {
            cout << "Cannot run Astar....No path to goal" << endl;
            NoStar = true;
        }

        if (!NoStar) {
            // float* xPTemp;
            // float* yPTemp;

            xPoints = (float*)malloc(sizeof(float) * len);
            yPoints = (float*)malloc(sizeof(float) * len);

            path.getPath(xPoints, yPoints);

            // Record end time
            auto end = std::chrono::high_resolution_clock::now();
            // Calculate duration in milliseconds
            std::chrono::duration<double, std::milli> duration = end - start;

            cout << "Path found in " << duration.count() << " ms" << endl;

            *x = (float*)malloc(len * sizeof(float));
            *y = (float*)malloc(len * sizeof(float));

            memcpy(*x, xPoints, len * sizeof(float));
            memcpy(*y, yPoints, len * sizeof(float));

            free(xPoints);
            free(yPoints);
            xPoints = nullptr;
            yPoints = nullptr;


            // cout << "---Astar points---" << endl;
            // for (unsigned short i = 0; i < len; i++) {
            //     cout << xPoints[i] << ", " << yPoints[i] << ", ";
            // }

            // cout << endl << endl << "---Astar points mm---" << endl;
            // for (unsigned short i = 0; i < len; i++) {
            //     cout << xPoints[i]*10 << ", " << yPoints[i]*10 << ", ";
            // }

            // cout << endl << "---X---" << endl;

            // for (unsigned short i = 0; i < len; i++) {
            //     cout << xPoints[i] * 10 << ", ";
            // }

            // cout << endl << "---Y---" << endl;

            // for (unsigned short i = 0; i < len; i++) {
            //     cout << yPoints[i] * 10 << ", ";
            // }

            // cout << endl << endl;

            // len--;
            // xPTemp = (float*)malloc(sizeof(float) * len);
            // yPTemp = (float*)malloc(sizeof(float) * len);

            // memcpy(xPTemp, xPoints + 1, len * sizeof(float));
            // memcpy(yPTemp, yPoints + 1, len * sizeof(float));

            // cout << endl << "---X---" << endl;

            // for (unsigned short i = 0; i < len; i++) {
            //     cout << xPTemp[i] << ", ";
            // }

            // cout << endl << "---Y---" << endl;

            // for (unsigned short i = 0; i < len; i++) {
            //     cout << yPTemp[i] << ", ";
            // }

            // cout << endl << "---Xmm---" << endl;

            // for (unsigned short i = 0; i < len; i++) {
            //     cout << xPTemp[i] * 10 << ", ";
            // }

            // cout << endl << "---Ymm---" << endl;

            // for (unsigned short i = 0; i < len; i++) {
            //     cout << yPTemp[i] * 10 << ", ";
            // }
        }
        else {
            cout << "Astar Failure" << endl;
        }
    }else{
        len = -3;
    }

    return len;
}


unsigned short Obstacles(unsigned short** obstacles, short x, short y, unsigned short xMin, unsigned short yMin, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl) {
    short obLen = 4, xbl, ybl, xtr, ytr;
    unsigned short obs[4];

    xbl = constrain(x - xl, xMin, xMax);
    xtr = constrain(x + xl, xMin, xMax);
    ybl = constrain(y - yl, yMin, yMax);
    ytr = constrain(y + yl, yMin, yMax);

    obs[0] = xbl;
    obs[1] = ybl;
    obs[2] = xtr;
    obs[3] = ytr;

    // cout << "xbl:" << xbl << " ,  ybl:" << ybl << " ,  xtr:" << xtr << " ,  ytr:" << ytr << endl;


    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        cout << "Memory allocation failed!" << endl;
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));
    return obLen;
}


unsigned short stageOb(unsigned short** obstacles, bool team, unsigned short xMin, unsigned short yMin, unsigned short xMax, unsigned short yMax, uint8_t tol, uint8_t Rs) {
    unsigned short obLen = 4, xbl, ybl, xtr, ytr;
    uint8_t Rsize = Rs + tol;
    unsigned short obs[4];

    // True blue, false yellow
    if (team) {
        xbl = xMin;
        ybl = 155 - Rsize - 5; // Adding a tolerance to force robot to avoid more the edge
        xtr = 240 + Rsize - 2;
        ytr = yMax;
    }
    else {
        xbl = 60 - Rsize + 2;
        ybl = 155 - Rsize - 5; // Adding a tolerance to force robot to avoid more the edge
        xtr = xMax;
        ytr = yMax;
    }

    obs[0] = xbl;
    obs[1] = ybl;
    obs[2] = xtr;
    obs[3] = ytr;

    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        cout << "Memory allocation failed!" << endl;
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));

    return obLen;
}

unsigned short enemyOb(unsigned short** obstacles, short x, short y, unsigned short xMin, unsigned short yMin, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl) {
    short obLen = 4, xbl, ybl, xtr, ytr;
    unsigned short obs[4];

    xbl = constrain(x - xl, xMin, xMax);
    xtr = constrain(x + xl, xMin, xMax);
    ybl = constrain(y - yl, yMin, yMax);
    ytr = constrain(y + yl, yMin, yMax);


    obs[0] = xbl;
    obs[1] = ybl;
    obs[2] = xtr;
    obs[3] = ytr;

    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        cout << "Memory allocation failed!" << endl;
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));

    return obLen;
}


short constrain(short num, short downLimit, short upLimit) {
    if (num > upLimit) {
        return upLimit;
    }
    else if (num < downLimit) {
        return downLimit;
    }
    else {
        return num;
    }


}

bool MovePoint(unsigned short& px, unsigned short& py, unsigned short obsLeft, unsigned short obsBottom, unsigned short obsRight, unsigned short obsTop, uint8_t step, uint8_t xMin, uint8_t yMin, unsigned short xMax, uint8_t yMax) {
    if (step == 0) {
        return false;
    }

    unsigned short dist_left = px - obsLeft;
    unsigned short dist_right = obsRight - px;
    unsigned short dist_bottom = py - obsBottom;
    unsigned short dist_top = obsTop - py;

    unsigned short min_dist = dist_left;
    uint8_t closest_side = 0; // 0=left, 1=right, 2=bottom, 3=top
    unsigned short newX = px;
    unsigned short newY = py;

    if (dist_right < min_dist) {
        min_dist = dist_right;
        closest_side = 1;
    }
    if (dist_bottom < min_dist) {
        min_dist = dist_bottom;
        closest_side = 2;
    }
    if (dist_top < min_dist) {
        closest_side = 3;
    }


    if (closest_side == 0) {
        newX = static_cast<int>(obsLeft) - 1;
        newX = (newX / step) * step;
    } else if (closest_side == 1) {
        newX = static_cast<int>(obsRight) + 1;
        newX = ((newX + step - 1) / step) * step;
    } else if (closest_side == 2) {
        newY = static_cast<int>(obsBottom) - 1;
        newY = (newY / step) * step;
    } else {
        newY = static_cast<int>(obsTop) + 1;
        newY = ((newY + step - 1) / step) * step;
    }

    if ((newX < xMin || newY < yMin) || (newX > xMax || newY > yMax)) {
        return false;
    }

    px = newX;
    py = newY;
    return true;
}
//-----------------------------------------------------------//



//------------------CAN related functions------------------//

// Setup function to initialize the CAN interface
int setupCAN(const char* iface) {
    int s;
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket creation failed");
        return -1;
    }

    strcpy(ifr.ifr_name, iface);
    ioctl(s, SIOCGIFINDEX, &ifr);
    fcntl(s, F_SETFL, O_NONBLOCK);

    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind failed");
        return -1;
    }

    return s;
}

// Equivalent to your sendCANmsg python function
void sendCANmsg(int socket, int msgID, int num1, int num2, int num3, int num4) {
    struct can_frame frame;
    frame.can_id = msgID;
    
    vector<int> nums;
    if (num1 != -1) nums.push_back(num1);
    if (num2 != -1) nums.push_back(num2);
    if (num3 != -1) nums.push_back(num3);
    if (num4 != -1) nums.push_back(num4);

    frame.can_dlc = nums.size() * 2; // Each int is 2 bytes (High/Low)

    for (size_t i = 0; i < nums.size(); ++i) {
        frame.data[i * 2] = (nums[i] >> 8) & 0xFF;     // High Byte
        frame.data[i * 2 + 1] = nums[i] & 0xFF;        // Low Byte
    }

    if (write(socket, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
        perror("CAN Write Error");
    } else {
        // cout << "CAN message sent ID: " << msgID << endl;
    }

}

void readCAN(int socket) {
    struct can_frame frame;
    int nbytes, message_id;
    static auto lastEnemyPosTime = steady_clk::now();
    static bool enemyPosSeen = false;
    const auto enemyPosTimeout = std::chrono::seconds(1);

    while (running) {
        if (enemyPosSeen && (steady_clk::now() - lastEnemyPosTime) >= enemyPosTimeout) {
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                enemyPos[0] = -6900;
                enemyPos[1] = -6900;
            }
            enemyPosSeen = false;
        }

        nbytes = read(socket, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                std::this_thread::sleep_for(std::chrono::milliseconds(5));
                continue;
            }
            perror("CAN read error");
            continue;
        }

        if (nbytes != sizeof(struct can_frame)) {
            continue;
        }

        message_id = frame.can_id;

        // Hard shutdown command (message ID 699)
        if (message_id == 699) {
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                running = 0;
                pathRequest = false;
            }
            pathRequestCv.notify_all();
            cout << "RX Shutdown Command (699): exiting program" << endl;
            break;
        }

        // Handle side selection (message ID 401) || or 423 (mine)
        if ((message_id == 401 || message_id == 423) && frame.can_dlc >= 2) {
            uint8_t side_value = frame.data[1];
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                side = (side_value == 1) ? true : false;
                // Side selection is also a "start" event: reset dynamic obstacle state.
                memcpy(pickUps, pickUpsDefault, sizeof(pickUpsDefault));
                for (int i = 0; i < 10; ++i) {
                    placeAreas[i] = false;
                }
            }
            cout << "RX Side Selection: " << (side ? "Blue (true)" : "Yellow (false)") << endl;
            cout << "Obstacle state reset: pickUps=default pattern, placeAreas=false" << endl;
        }


        if (message_id == 741) {
            if (frame.can_dlc >= 2) {
                int pickup_idx = (frame.data[0] << 8) | frame.data[1];
                if (pickup_idx >= 0 && pickup_idx < 8) {
                    {
                        std::lock_guard<std::mutex> lock(dataMutex);
                        pickUps[pickup_idx] = 0;
                    }
                    cout << "Deactivated pickup number: " << pickup_idx << endl;
                } else {
                    cout << "RX 741 ignored: invalid pickup index " << pickup_idx << endl;
                }
            } else {
                cout << "RX 741 ignored: DLC too small (" << (int)frame.can_dlc << ")" << endl;
            }
        }

        if (message_id == 742) {
            if (frame.can_dlc >= 2) {
                int pantry_idx = (frame.data[0] << 8) | frame.data[1];
                if (pantry_idx >= 0 && pantry_idx < 10) {
                    {
                        std::lock_guard<std::mutex> lock(dataMutex);
                        placeAreas[pantry_idx] = true;
                    }
                    cout << "Activated pantry number: " << pantry_idx << endl;
                } else {
                    cout << "RX 742 ignored: invalid pantry index " << pantry_idx << endl;
                }
            } else {
                cout << "RX 742 ignored: DLC too small (" << (int)frame.can_dlc << ")" << endl;
            }
        }
        
        // Handle path request with robot and goal positions (message ID 422)
        if (message_id == 422 && frame.can_dlc >= 8) {
            int x_pos = (frame.data[0] << 8) | frame.data[1];
            int y_pos = (frame.data[2] << 8) | frame.data[3];
            int xg_pos = (frame.data[4] << 8) | frame.data[5];
            int yg_pos = (frame.data[6] << 8) | frame.data[7];
            
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                robotPos[0] = x_pos;
                robotPos[1] = y_pos;
                goal[0] = xg_pos;
                goal[1] = yg_pos;
            }
            // Wake main thread immediately when path request arrives
            pathRequestCv.notify_one();
            cout << "RX Path Request (422): Robot(" << x_pos << "," << y_pos << ") Goal(" << xg_pos << "," << yg_pos << ")" << endl;
        }

        // Selecting if Astar will happend with or without taking enemy into account
        // if (message_id == 424) {
        //     if (frame.can_dlc >= 4) {
        //         bool enemyValue = (frame.data[0] << 8) | frame.data[1];
        //         // uint8_t choice = (frame.data[0] << 8) | frame.data[3];
        //         {
        //             std::lock_guard<std::mutex> lock(dataMutex);
        //             withEnemy = enemyValue;
        //             headingChoice = choice;
        //             pathRequest = true;  // Trigger pathfinding immediately
        //         }
        //         cout << "Astar withEnemy = " << withEnemy << endl;

        //     } else {
        //         cout << "RX 742 ignored: DLC too small (" << (int)frame.can_dlc << ")" << endl;
        //     }
        // }

        if (message_id == 424) {
            if (frame.can_dlc >= 2) {
                bool enemyValue = (frame.data[0] << 8) | frame.data[1];
                {
                    std::lock_guard<std::mutex> lock(dataMutex);
                    withEnemy = enemyValue;
                    pathRequest = true;  // Trigger pathfinding immediately
                }
                cout << "Astar withEnemy = " << withEnemy << endl;

            } else {
                cout << "RX 742 ignored: DLC too small (" << (int)frame.can_dlc << ")" << endl;
            }
        }
        
        // Handle enemy position updates (message ID 225 example)
        if (message_id == 225 && frame.can_dlc >= 4) {
            int enemy_x = (frame.data[0] << 8) | frame.data[1];
            int enemy_y = (frame.data[2] << 8) | frame.data[3];
            
            {
                std::lock_guard<std::mutex> lock(dataMutex);
                enemyPos[0] = enemy_x;
                enemyPos[1] = enemy_y;
            }
            lastEnemyPosTime = steady_clk::now();
            enemyPosSeen = true;
            // cout << "RX Enemy Position: X=" << enemy_x << " Y=" << enemy_y << endl;
        }
        
        // // Print all received CAN messages for debugging
        // if (message_id == 225 || message_id == 401 || message_id == 420 || message_id == 421 || message_id == 423) {

        //     cout << "RX ID=" << message_id << " DLC=" << (int)frame.can_dlc << " DATA: ";

        //     // Parse and print data bytes
        //     for (int i = 0; i < frame.can_dlc; ++i) {
        //         printf("%02X ", frame.data[i]);
        //     }

        //     // Parse 16-bit integers if applicable
        //     if (frame.can_dlc >= 2) {
        //         cout << "| VALUES: ";
        //         for (int i = 0; i < frame.can_dlc; i += 2) {
        //             if (i + 1 < frame.can_dlc) {
        //                 int number = (frame.data[i] << 8) | frame.data[i + 1];
        //                 cout << number << " ";
        //             }
        //         }
        //     }
        //     cout << endl;
        // }
    }
}
//---------------------------------------------------------//
