#include <Arduino.h>
#include <FreeRTOS.h>
#include <Astar_HR.h>
#include <task.h>



float* xPoints = nullptr;
float* yPoints = nullptr;
unsigned short len = 0;
/*int lenObstaclePoints = 51;
int lenObs = 2 * lenObstaclePoints;*/
unsigned short lenObs = 0;
unsigned short TotLenObs = 0;
unsigned short xMax = 300;
uint8_t yMax = 200;
//bool grid[150][100];
unsigned short* obstacles = nullptr;
unsigned short* TotObs = nullptr;
unsigned short* tempObs = nullptr;
// 1 -> every 1 cm, 5 -> every 5 cm, 10 -> every 10cm 
uint8_t xl, yl, step = 2, tol = 2, Rsize = 15;
double time1,time2;


void PathFind(void* pvParameters) {
  (void)pvParameters;
  uint8_t count=0;

  while (1) {

    if(count == 150){
      count = 0;
    }

    Serial.print("Count=");
    Serial.println(count);
    if(count == 2){
      // TotObs = (unsigned short*)malloc(sizeof(unsigned short) * 2);
      // TotLenObs = 2;
      // TotObs[0] = 10;
      // TotObs[1] = 10; 
      Serial.println("---Astar__Start---");
      time1 = millis();
      Astar_HR shit(122, 22, 280, 180, xMax, yMax, TotObs, TotLenObs, true, step);
      len = shit.pathGeneration();
      xPoints = (float*)malloc(sizeof(float)*len);
      yPoints = (float*)malloc(sizeof(float)*len);
      shit.getPath(xPoints, yPoints);
      time2 = millis();

      for(int i=0; i<len; i++){
        // Serial.print("X:");
        // Serial.print(xPoints[i]);
        // Serial.print("  , Y:");
        // Serial.println(yPoints[i]);
        Serial.print(xPoints[i]);
        Serial.print(", ");
        Serial.print(yPoints[i]);
        Serial.print(", ");
      }
      Serial.print("Lenght of points:");
      Serial.println(len);
      Serial.print("Time:");
      Serial.println(time2-time1);
      Serial.println("---Astar__End---");
    }
    count++;
    vTaskDelay(pdMS_TO_TICKS(12000)); // Adjust the delay as needed
  }
}




void setup() {

  delay(50);
  Serial.begin(115200);
  
  Serial.println("------Start Of Obstacles------");
  
  short obsStart[20] = {7, 40, 77, 25, 223, 25, 293, 40, 7, 133, 110, 95, 190, 95, 293, 133, 81, 173, 219, 173};
  unsigned short k = 0;

  // Tribunes
  xl = 20 + Rsize + tol;
  yl = 5 + Rsize + tol;
  for (unsigned short i = 0; i < 20; i += 2) {
      if (k == 0 || k == 3 || k == 4 || k == 7) {
          lenObs = tribObstacles(&obstacles, obsStart[i], obsStart[i + 1], xMax, yMax, yl, xl, false);
      }
      else {
          lenObs = tribObstacles(&obstacles, obsStart[i], obsStart[i + 1], xMax, yMax, xl, yl, true);
      }
      k++;


      tempObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
      if (tempObs == nullptr) {
          // cout << "Memory allocation failed!" << endl;
          exit(1);
      }

      if (TotLenObs > 0) {
          memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
          free(TotObs);
      }

      TotLenObs += lenObs;

      TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
      if (TotObs == nullptr) {
          // cout << "Memory allocation failed!" << endl;
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
      // cout << "Memory allocation failed!" << endl;
      exit(1);
  }

  if (TotLenObs > 0) {
      memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
      free(TotObs);
  }

  TotLenObs += lenObs;

  TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
  if (TotObs == nullptr) {
      // cout << "Memory allocation failed!" << endl;
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
      // cout << "Memory allocation failed!" << endl;
      exit(1);
  }

  if (TotLenObs > 0) {
      memcpy(tempObs, TotObs, TotLenObs * sizeof(unsigned short));
      free(TotObs);
  }

  TotLenObs += lenObs;

  TotObs = (unsigned short*)malloc(TotLenObs * sizeof(unsigned short));
  if (TotObs == nullptr) {
      // cout << "Memory allocation failed!" << endl;
      exit(1);
  }

  if (TotLenObs - lenObs > 0) {
      memcpy(TotObs, tempObs, (TotLenObs - lenObs) * sizeof(unsigned short));
  }

  memcpy(TotObs + (TotLenObs - lenObs), obstacles, lenObs * sizeof(unsigned short));

  free(tempObs);

  for (int i = 0; i < TotLenObs; i += 2) {
      // cout << TotObs[i] << ", " << TotObs[i + 1] << ", ";
      Serial.print(TotObs[i]);
      Serial.print(", ");
      Serial.print(TotObs[i+1]);
      Serial.println(", ");
  }


  Serial.println("------End Of Obstacles------");


  xTaskCreate(PathFind, "PathFind", 2048, NULL, 1, NULL);
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS will manage task execution, so you don't need the loop function.
}


unsigned short tribObstacles(unsigned short** obstacles, short x, short y, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl, bool horizontal) {
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
        // cout << "Memory allocation failed!" << endl;
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
        // cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));

    return obLen;
}


unsigned short enemyOb(unsigned short** obstacles, short x, short y, unsigned short xMax, unsigned short yMax, uint8_t xl, uint8_t yl) {
    short obLen = 4, xtl, ytl, xtr, ybr;
    unsigned short obs[4];

    xtl =  constrain(x - xl, 0, xMax);
    xtr = constrain(x + xl, 0, xMax);
    ytl = constrain(y + yl, 0, yMax);
    ybr = constrain(y - yl, 0, yMax);


    obs[0] = xtl;
    obs[1] = ytl;
    obs[2] = xtr;
    obs[3] = ybr;

    *obstacles = (unsigned short*)malloc(obLen * sizeof(unsigned short));
    if (*obstacles == nullptr) {
        // cout << "Memory allocation failed!" << endl;
        exit(1);
    }

    memcpy(*obstacles, obs, obLen * sizeof(unsigned short));

    return obLen;
}


// short constrain(short num, short downLimit, short upLimit) {
//     if (num > upLimit) {
//         return upLimit;
//     }
//     else if (num < downLimit) {
//         return downLimit;
//     }
//     else {
//         return num;
//     }

// }

