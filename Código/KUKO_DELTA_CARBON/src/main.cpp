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
Endstops endstops;

void setup()
{
    Serial.begin(115200); // Inicializa la comunicación con el monitor serie

    //conveyor.begin();
    //encoders.begin();
    //pneumatics.begin();
    endstops.begin();
    //motors.begin();
    //robot.begin();
    //robot.startHoming();
    //robot.testMotor1();
}

void loop()
{

    //robot.update();

Serial.println(endstops.readMotor1());
delay(10);

    /*
    Serial.print("E1: ");
    Serial.print(encoders.leerGrados(4));

    Serial.print(" | E2: ");
    Serial.print(encoders.leerGrados(3));

    Serial.print(" | E3: ");
    Serial.println(encoders.leerGrados(6));
*/
   



}