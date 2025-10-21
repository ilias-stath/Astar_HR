
#include <iostream>
#include "AStar_HR.h"
#include <set>
#include <vector>
#include <cmath>
#include <windows.h>

using namespace std;

unsigned short tribObstacles(unsigned short** obstacles, short x, short y, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl); // True horizontal, false vertical
unsigned short stageOb(unsigned short** obstacles, bool team, unsigned short xMax, unsigned short yMax, uint8_t tol, uint8_t Rs); // True blue, false yellow
unsigned short enemyOb(unsigned short** obstacles, short x, short y, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl);
short constrain(short num, short downLimit, short upLimit);




int main()
{
    float* xPoints = nullptr;
    float* yPoints = nullptr;
    short len = 0;
    unsigned short lenObs = 0;
    unsigned short TotLenObs = 0;
    unsigned short xMax = 300;
    uint8_t yMax = 200;
    unsigned short* obstacles = nullptr;
    unsigned short* TotObs = nullptr;
    unsigned short* tempObs = nullptr;
    uint8_t xl, yl, step = 2, tol = 1, Rsize = 20;
    // 1 -> every 1 cm, 5 -> every 5 cm, 10 -> every 10cm 


    short obsStart[20] = { 7, 40, 77, 25, 223, 25, 293, 40, 7, 133, 110, 95, 190, 95, 293, 133, 81, 173, 219, 173 };
    short bigAreasStart[12] = { 22, 88, 178, 22, 262, 178, 38, 178, 122, 22, 278, 88 };
    short smallAreasStart[8] = { 22, 7, 77, 7, 223, 7, 278, 7 };
    bool bigAreas[6] = {true, true, true, true, true, true};
    bool smallAreas[4] = {true, true, true, true};
    uint8_t stocks[10] = { 2,1,1,2,2,1,1,2,1,1 };
    unsigned short k = 0;


    // Small Areas
    xl = 22 + Rsize + tol;
    yl = 7 + Rsize + tol;
    for (unsigned short i = 0; i < 8; i += 2) {
        if (smallAreas[k]) {
            lenObs = tribObstacles(&obstacles, smallAreasStart[i], smallAreasStart[i + 1], xMax, yMax, xl, yl);
        }
        else {
            cout << "Small area " << k << " is free" << endl;
        }
        k++;


        tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (tempObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
            exit(1);
        }

        if (TotLenObs > 0) {
            memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
            free(TotObs);
        }

        TotLenObs += lenObs;

        TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (TotObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
            exit(1);
        }

        if (TotLenObs - lenObs > 0) {
            memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
        }

        memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

        free(tempObs);

    }
    k = 0;

    cout << endl << "Small areas" << endl;
    for (int i = 0; i < TotLenObs; i += 2) {
        if (TotLenObs - i == 12) {
        }
        cout << TotObs[i] << ", " << TotObs[i + 1] << ", ";
    }

    cout << endl << endl;
    

    // Big Areas
    xl = 22 + Rsize + tol;
    yl = 22 + Rsize + tol;
    for (unsigned short i = 0; i < 12; i += 2) {
        if (bigAreas[k]) {
            lenObs = tribObstacles(&obstacles, bigAreasStart[i], bigAreasStart[i + 1], xMax, yMax, xl, yl);
        }
        else {
            cout << "Small area " << k << " is free" << endl;
        }
        k++;


        tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (tempObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
            exit(1);
        }

        if (TotLenObs > 0) {
            memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
            free(TotObs);
        }

        TotLenObs += lenObs;

        TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (TotObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
            exit(1);
        }

        if (TotLenObs - lenObs > 0) {
            memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
        }

        memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

        free(tempObs);

    }
    k = 0;


    // Tribunes
    xl = 20 + Rsize + tol;
    yl = 5 + Rsize + tol;
    for (unsigned short i = 0; i < 20; i += 2) {
        if (stocks[k] == 2) {
            lenObs = tribObstacles(&obstacles, obsStart[i], obsStart[i + 1], xMax, yMax, yl, xl); // vertical
        }
        else if(stocks[k] == 1){
            lenObs = tribObstacles(&obstacles, obsStart[i], obsStart[i + 1], xMax, yMax, xl, yl); // horizontal
        }
        else {
            cout << "Stock " << k << " is taken" << endl;
        }
        k++;


        tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (tempObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
            exit(1);
        }

        if (TotLenObs > 0) {
            memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
            free(TotObs);
        }

        TotLenObs += lenObs;

        TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
        if (TotObs == nullptr) {
            cout << "Memory allocation failed!" << endl;
            exit(1);
        }

        if (TotLenObs - lenObs > 0) {
            memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
        }

        memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

        free(tempObs);

    }


    // Enemy Obstacle
    yl = xl = 16 + Rsize + tol;
    lenObs = enemyOb(&obstacles, 250, 50, xMax, yMax, xl, yl);


    tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
    if (tempObs == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    if (TotLenObs > 0) {
        memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
        free(TotObs);
    }

    TotLenObs += lenObs;

    TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
    if (TotObs == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    if (TotLenObs - lenObs > 0) {
        memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
    }

    memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

    free(tempObs);


    // Stage Obstacle
    lenObs = stageOb(&obstacles, true, xMax, yMax, tol, Rsize);


    tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
    if (tempObs == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    if (TotLenObs > 0) {
        memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
        free(TotObs);
    }

    TotLenObs += lenObs;

    TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
    if (TotObs == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    if (TotLenObs - lenObs > 0) {
        memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
    }

    memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

    free(tempObs);



    cout << endl << endl << "END OBSTACLES CREATION" << endl << endl;

    for (int i = 0; i < TotLenObs; i += 2) {
        if (TotLenObs - i == 12) {
        }
        cout << TotObs[i] << ", " << TotObs[i + 1] << ", ";
    }

    cout << endl;

    cout << "Starting" << endl;
    cout << "Number of Obstacles -> " << TotLenObs << endl;
    cout << "Memory of Obstacles with int -> " << TotLenObs * sizeof(int) << endl;
    cout << "Memory of Obstacles witn short -> " << TotLenObs * sizeof(unsigned short) << endl;

    bool NoStar = false;
    Astar_HR astar(0, 0, 150, 200, xMax, yMax, TotObs, TotLenObs, true, step);

    len = astar.pathGeneration();

    if (len == -1) {
        cout << "Cannot run Astar....No path to goal" << endl;
        NoStar = true;
    }

    if (!NoStar) {
        float* xPTemp;
        float* yPTemp;

        xPoints = (float*)malloc(sizeof(float) * len);
        yPoints = (float*)malloc(sizeof(float) * len);

        astar.getPath(xPoints, yPoints);


        cout << "---Astar points---" << endl;
        for (unsigned short i = 0; i < len; i++) {
            cout << xPoints[i] << ", " << yPoints[i] << ", ";
        }

        cout << endl << "---X---" << endl;

        for (unsigned short i = 0; i < len; i++) {
            cout << xPoints[i] * 10 << ", ";
        }

        cout << endl << "---Y---" << endl;

        for (unsigned short i = 0; i < len; i++) {
            cout << yPoints[i] * 10 << ", ";
        }

        cout << endl << endl;

        len--;
        xPTemp = (float*)malloc(sizeof(float) * len);
        yPTemp = (float*)malloc(sizeof(float) * len);

        memcpy(xPTemp, xPoints + 1, len * sizeof(float));
        memcpy(yPTemp, yPoints + 1, len * sizeof(float));

        for (unsigned short i = 0; i < len; i++) {
            cout << xPTemp[i] * 10 << ", ";
        }

        cout << endl << "---Y---" << endl;

        for (unsigned short i = 0; i < len; i++) {
            cout << yPTemp[i] * 10 << ", ";
        }
    }
    else {
        cout << "Astar Failure" << endl;
    }
    

}




unsigned short tribObstacles(unsigned short** obstacles, short x, short y, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl) {
    short obLen = 4, xtl, ytl, xtr, ybr;
    unsigned short obs[4];

    xtl = constrain(x - xl, 0, xMax);
    xtr = constrain(x + xl, 0, xMax);
    ytl = constrain(y + yl, 0, yMax);
    ybr = constrain(y - yl, 0, yMax);

    obs[0] = xtl;
    obs[1] = ytl;
    obs[2] = xtr;
    obs[3] = ybr;


    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));
    return obLen;
}


unsigned short stageOb(unsigned short** obstacles, bool team, unsigned short xMax, unsigned short yMax, uint8_t tol, uint8_t Rs) {
    unsigned short obLen = 6, xtl, ytl, ybl, xtm, ytm, xtr;
    uint8_t Rsize = Rs + tol;
    unsigned short obs[6];

    if (team) {
        xtl = 0;
        ytl = yMax;
        ybl = 155 - Rsize;
        xtm = 195 + Rsize;
        ytm = 180 - Rsize;
        xtr = 235 + Rsize;
    }
    else {
        xtr = xMax;
        xtm = xMax - 195 - Rsize;
        ytm = 180 - Rsize;
        ybl = 180 - Rsize;
        xtl = xMax - 235 - Rsize;
        ytl = yMax;
    }

    obs[0] = xtl;
    obs[1] = ytl;
    obs[2] = xtr;
    obs[3] = ybl;
    obs[4] = xtm;
    obs[5] = ytm;

    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));

    return obLen;
}



unsigned short enemyOb(unsigned short** obstacles, short x, short y, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl) {
    short obLen = 4, xtl, ytl, xtr, ybr;
    unsigned short obs[4];
    
    xtl = constrain(x - xl, 0, xMax);
    xtr = constrain(x + xl, 0, xMax);
    ytl = constrain(y + yl, 0, yMax);
    ybr = constrain(y - yl, 0, yMax);


    obs[0] = xtl;
    obs[1] = ytl;
    obs[2] = xtr;
    obs[3] = ybr;

    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        cout << "Memory allocation failed!" << endl;
        exit(1);
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
