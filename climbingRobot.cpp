#include <hFramework.h>
#include <DistanceSensor.h>
#include <hMotor.h>
#include <hSerial.h>
#include <Lego_Touch.h>
#include <stddef.h>
#include <stdio.h>

using namespace hFramework;
using namespace hModules;
using namespace hSensors;

#define MANUAL_MODE 0x01
#define AUTO_MODE 0x02
#define DEATH_ZONE 10
#define _DEBUG_EXTRA

#ifdef _DEBUG_HARDWARE
#define PRINTF_HW(...) printf(__VA_ARGS__)
#else
#define PRINTF_HW(...) 
#endif

#ifdef _DEBUG_MOTOR
#define PRINTF_M(...) printf(__VA_ARGS__)
#else
#define PRINTF_M(...) 
#endif

#ifdef _DEBUG_EXTRA
#define PRINTF_E(...) printf(__VA_ARGS__)
#else
#define PRINTF_E(...) 
#endif

DistanceSensor sens1(hSens1.getBaseSens());
DistanceSensor sens2(hSens2.getBaseSens());
Lego_Touch sens3(hSens3);

void manual(void);
void automatic(void);

bool isManualNow;
bool isAutomaticNow;
bool manualModeIsOver;
bool autoModeIsOver;
bool isFirstTime;

static int32_t setPosition;
static int16_t motorPower;

void TaskMotorControl(void);
void TaskReporter(void);
int CentsToTicks(int cents);

void InitializeTask(void){
    isManualNow = false;
    isAutomaticNow = false;
    isFirstTime = true;
    PRINTF_HW("InitializeTask\n");
}

void EndTasks(void){
    isManualNow = false;
    isAutomaticNow = false;
    if (isFirstTime){
        PRINTF_HW("IT IS FIRST TIME\n");
        autoModeIsOver = true;
        manualModeIsOver = true;
        isFirstTime = false;
    }else{
        autoModeIsOver = false;
        manualModeIsOver = false;
    }
}

void SwichMode(uint8_t mode){
    EndTasks();
    isFirstTime = false;
    if (mode == AUTO_MODE){
        isAutomaticNow = true;
        PRINTF_HW("WE SHOULD SWITCH TO AUTO NOW\n");
        while (!manualModeIsOver){
            PRINTF_HW("WAITING TO END MANUAL MODE\n");
            sys.delay(50);
        }
        manualModeIsOver = false;
        PRINTF_HW("automatic");
        sys.taskCreate(automatic);
    }
    else if (mode == MANUAL_MODE){
        isManualNow = true;
        PRINTF_HW("WE SHOULD SWITCH TO MANUL NOW\n");
        while (!autoModeIsOver){
            PRINTF_HW("WAITING TO END AUTO MODE\n");
            sys.delay(50);
        }
        autoModeIsOver = false;
        PRINTF_HW("manual");
        sys.taskCreate(manual);
        sys.taskCreate(TaskMotorControl);
        sys.taskCreate(TaskReporter);
    }
}

void automatic(void){
    sys.setLogDev(&Serial);
    DistanceSensor sens1(hSens1.getBaseSens());
    DistanceSensor sens2(hSens2.getBaseSens());
    hExt1.pin3.setIn_pd();
    hExt1.pin4.setIn_pu();

    while (isAutomaticNow){
        int dist1 = sens1.getDistance();
        int dist2 = sens2.getDistance();
        bool check = false;
        bool pressed = false;

        check = hExt1.pin3.read();
        pressed = hExt1.pin4.read();
        sys.delay(1000);

        if (pressed == 0){
            hMot1.stop();
            hMot2.stop();
            hMot3.setPower(-200);
            hMot4.setPower(200);
            PRINTF_M("ACTUATOR WORKING ");
        }else{
            hMot3.stop();
            hMot4.stop();
            PRINTF_M("ACTUATOR DISABLED\r\n");

            if (check == 1){
                PRINTF_M("DRIVING\r\n");
                hMot1.setPower(1000);
                hMot2.setPower(1000);
            }

            while (pressed == 1 && isAutomaticNow){
                dist1 = sens1.getDistance();
                dist2 = sens2.getDistance();
                check = hExt1.pin3.read();
                pressed = hExt1.pin4.read();

                PRINTF_M("top: %d;  bottom: %d\r\n", dist1, dist2);

                if (dist1 == -1 || dist2 == -1){
                    PRINTF_M("%d; %d - WRONG DATA\r\n", dist1, dist2);
                    hLED1.on();
                    hMot1.stop();
                    hMot2.stop();
                    continue;
                }else if (dist1 <= 10 && check == 1){
                    hMot1.setPower(-250);
                    hMot2.setPower(-250);
                }else if (dist2 <= 30 && check == 1){
                    hMot1.setPower(1000);
                    hMot2.setPower(1000);
                }else if (check == 0){
                    hMot1.setPower(-250);
                    hMot2.setPower(-250);
                }else{
                    PRINTF_M("WORKING \r\n");
                }
            }
        }
        PRINTF_HW("AUTO MODE\r\n");
        sys.delay(100);
    }
    autoModeIsOver = true;
}

void manual(void){
    sys.setLogDev(&Serial);
    DistanceSensor sens1(hSens1.getBaseSens());
    DistanceSensor sens2(hSens2.getBaseSens());
    hExt1.pin3.setIn_pd();
    hExt1.pin4.setIn_pu();

    float distance;
    int tick;
    int dist1 = sens1.getDistance();
    int dist2 = sens2.getDistance();
    bool check = false;
    bool pressed = false;
    char c;
    char incomingString[40];

    hMot1.stop();
    hMot2.stop();

    for (int i = 0; i < 40; i++){
        incomingString[i] = '0';
    }

    uint8_t incomingDataCounter = 0;
    char buff[20];

    while (isManualNow){
        check = hExt1.pin3.read();
        pressed = hExt1.pin4.read();
		dist1 = sens1.getDistance();
		dist2 = sens2.getDistance();

        PRINTF_HW("SENS 1 : %d\n",dist1);
        PRINTF_HW("SENS 2 : %d\n",dist2);

        if (pressed == 0){
            hMot1.stop();
            hMot2.stop();
            hMot3.setPower(-200);
            hMot4.setPower(200);
        }else if (pressed == 1){
            hMot3.stop();
            hMot4.stop();
        }
        if ((check != 1 || dist1 < 10) && pressed == 1){
            hMot1.stop();
            hMot2.stop();
            setPosition = 0;
            hMot2.resetEncoderCnt();

            setPosition = CentsToTicks(-5);
            if (setPosition > hMot2.getEncoderCnt()){
                if (pressed == 1){
                    hMot1.setPower(1000);
                    hMot2.setPower(1000);
                }
            }else if (setPosition < hMot2.getEncoderCnt()){
                if (pressed == 1){
                    hMot1.setPower(-250);
                    hMot2.setPower(-250);
                }
            }
        }
        if (dist2 < 30 && pressed == 1 && check == 1){
            hMot1.stop();
            hMot2.stop();
            setPosition = 0;
            hMot2.resetEncoderCnt();

            setPosition = CentsToTicks(5);
            if (setPosition > hMot2.getEncoderCnt()){
                if (pressed == 1){
                    hMot1.setPower(1000);
                    hMot2.setPower(1000);
                }
            }else if (setPosition < hMot2.getEncoderCnt()){
                if (pressed == 1){
                    hMot1.setPower(-250);
                    hMot2.setPower(-250);
                }
            }
        }

        if (hExt2.serial.available() > 0){
            while (hExt2.serial.available() > 0){
                c = hExt2.serial.getch();
                incomingString[incomingDataCounter] = c;
                incomingDataCounter++;
            }
            for (int i = 0; i < incomingDataCounter; i++){
                PRINTF_E("%c", incomingString[i]);
            }
            int32_t result = 0;
            uint32_t multiplyPoiter = 1;
            for (int i = incomingDataCounter - 1; i >= 1; i--){
                int newInt = incomingString[i] - '0';
                result += multiplyPoiter * newInt;
                multiplyPoiter = multiplyPoiter * 10;
            }
            if (incomingString[0] == '-'){
                result = -result;
            }else{
                int newInt = incomingString[0] - '0';
                result += newInt * multiplyPoiter;
            }
            incomingDataCounter = 0;
            PRINTF_HW("DATA: %d\n", result);
            setPosition += CentsToTicks(result);
            if (setPosition > hMot2.getEncoderCnt()){
                if (pressed == 1 && check == 1){
                    hMot1.setPower(1000);
                    hMot2.setPower(1000);
                }
            }else if (setPosition < hMot2.getEncoderCnt()){
                if (pressed == 1 && check == 1){
                    hMot1.setPower(-250);
                    hMot2.setPower(-250);
                }
            }
            PRINTF_HW("TICKS: %d\n", CentsToTicks(result));
            PRINTF_HW("NEW POS: %d\n", setPosition);
        }

        PRINTF_HW("MANUAL WORK\r\n");
        sys.delay(100);
    }
    manualModeIsOver = true;
}

int CentsToTicks(int cents){
    int res;
    res = (int)(cents * 98.6);
    return res;
}

void TaskReporter(void){
    int32_t actPos = hMot2.getEncoderCnt();
    int32_t diff;

    while (isManualNow){
        actPos = hMot2.getEncoderCnt();
        diff = (int32_t)abs(actPos - setPosition);
        PRINTF_M("ACT POS: %d\t SET POS: %d\t DIFF:%d\n", (int)actPos, (int)setPosition, (int)diff);
        sys.delay(100);
    }
}

void TaskMotorControl(void){
    int32_t actPos = hMot2.getEncoderCnt();
    int32_t diff;

    while (isManualNow){
        actPos = hMot2.getEncoderCnt();
        diff = (int32_t)abs(actPos - setPosition);
        if (diff < DEATH_ZONE){
            hMot1.stop();
            hMot2.stop();
        }else if (motorPower > 0 && actPos > setPosition){
            hMot1.stop();
            hMot2.stop();
        }else if (motorPower < 0 && actPos < setPosition){
            hMot1.stop();
            hMot2.stop();
        }
    }
}

void hMain(){
    sys.setLogDev(&Serial);
    bool previous = false;
    bool current = false;
    int licznik = 0;
    InitializeTask();
    hExt2.serial.init(115200, Parity::None, StopBits::One);

    while (true){
        current = sens3.isPressed();
        if (current && previous != current){
            licznik++;
            if (licznik % 2 == 0){
                SwichMode(AUTO_MODE);
            }else{
                SwichMode(MANUAL_MODE);
            }
        }
        sys.delay(50);
        previous = current;
    }
}