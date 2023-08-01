// File:          linefollower2.cpp
// Date:
// Description:
// Author:
// Modifications:

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <dos.h>
#include <string>
#include <vector>
#include <math.h> 

#define TIME_STEP 32
#define NORMAL_SPEED 5
#define MAX_SPEED 7
#define KP 1.2
#define KI 0.0
#define KD 0.04


char motorNames[2][15] = { "left_motor", "right_motor" };
char irNames[8][5]={"ir0","ir1","ir2","ir3","ir4","ir5","ir6","ir7"};

double sensorValues[8];
int IR_THRESHOLD = 400;
//PID variables
double prevError = 0;
double integral = 0;



// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

//----function to read the values of the sensors and convert to binary-------
string getIRValues(DistanceSensor **irPanel, int IR_THRESHOLD)
    {
        string panelVals = "";
        string panelVals_act = "";

        for (int i = 0; i < 8; i++)
        {
            panelVals_act += std::to_string((double)irPanel[i]->getValue()) + " ";
            if (irPanel[i] -> getValue() > IR_THRESHOLD) // For Balck
                panelVals += "0";
            else  //For White
                panelVals += "1"; 
        }
        // std::cout << panelVals_act << std::endl;

        return panelVals;
     }
//----------------------end of the read() function----------------------------

 void PID(Motor **motors, string IRVal, double *integral, double *prevError)
    {
        int panelWeights[8] = {-400,-300,-200,-100,100,200,300,400};
        double position = 0;
        double error = 0;
        double correction = 0;
        int count = 0;

        for (int i = 0; i < 8; i++)
        {
            if (IRVal[i] == '1') // For White
            {
                position += panelWeights[i];
                count++;
            } 
        }
        
        //checking for zero division
        if (count == 0) 
        count = 1;
        
        error = position/(count * 10);   
        *integral += error;
        // XV_print(error)
        
        correction = KP * error + KI * (*integral) + KD*(error - *prevError); //calculating the correction
        
        // XV_print(correction)
        double leftSpeed = (NORMAL_SPEED + correction);
        double rightSpeed = NORMAL_SPEED - correction; 
        
        //Limiting the max speed
        if (leftSpeed > MAX_SPEED)
            leftSpeed = MAX_SPEED;
        if (rightSpeed > MAX_SPEED)
            rightSpeed = MAX_SPEED;
        if (leftSpeed < -MAX_SPEED)
            leftSpeed = -MAX_SPEED;
        if (rightSpeed < -MAX_SPEED)
            rightSpeed = -MAX_SPEED;
        
        motors[0]->setVelocity(leftSpeed);
        motors[1]->setVelocity(rightSpeed);
        
        *prevError = error;
    }



// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  //objects
Motor *motors[2];
DistanceSensor* irPanel[8];
  
  // init Motors
  
	for (int i = 0; i < 2; i++)
	{
		motors[i] = robot->getMotor(motorNames[i]);
		motors[i]->setPosition(INFINITY);
		motors[i]->setVelocity(0.0);
	}

    //------------------------------------------------------------------------
   

	// init IR Panel
	for (int i = 0; i < 8; i++)
	{
		irPanel[i] = robot->getDistanceSensor(irNames[i]);
		irPanel[i]->enable(TIME_STEP);
	}
    //------------------------------------------------------------------------

 
    
    //-------------------set position sensors------------------------
    
   /* PositionSensor *leftPs = robot->getPositionSensor("left wheel sensor");
    leftPs->enable(TIME_STEP);
    PositionSensor *rightPs = robot->getPositionSensor("right wheel sensor");
    rightPs->enable(TIME_STEP);*/
    
    //----------------------------------------------------------------

  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1) {
    string IR_Values = getIRValues(irPanel, IR_THRESHOLD);
    std::cout<<IR_Values;
    PID(motors, IR_Values, &integral, &prevError);
    }


  // Enter here exit cleanup code.

  delete robot;
  return 0;
}