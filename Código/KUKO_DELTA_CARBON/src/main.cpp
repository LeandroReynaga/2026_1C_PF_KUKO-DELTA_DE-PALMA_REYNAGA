#include <Arduino.h>
#include "Pinout.h"
#include "hardware/Conveyor.h"
#include "hardware/Encoders.h"
#include "hardware/Pneumatics.h"
#include "hardware/Endstops.h"
#include "hardware/Motors.h"
#include "robot/Robot.h"

Conveyor conveyor(CINTAPWM); 
Encoders encoders;           
Pneumatics pneumatics;       
Motors motors;           
Robot robot;    

void setup()
{
    Serial.begin(115200); // Inicializa la comunicación con el monitor serie

    //conveyor.begin();
    //encoders.begin();
    //pneumatics.begin();
    //endstops.begin();
    //motors.begin();
    robot.begin();
    robot.startHoming();
}

void loop()
{

    robot.update();

 if(robot.homingFinished())
    {
        Serial.println("HOME OK");
    }
    
Serial.println(digitalRead(FC1));


}