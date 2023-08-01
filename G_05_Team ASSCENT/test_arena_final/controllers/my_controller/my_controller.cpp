#include <webots/Robot.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/Camera.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
//#include <webots/Keyboard.hpp>
#include <iostream>
#include <stdio.h>
#include <string>
#include<cmath>

#include <fstream>

#define TIME_STEP 64
#define MAX_SPEED 6.28
#define baseSpeed 6

using namespace webots;
using namespace std;

bool break_main_loop;

int colour_var =0; //red-0
                   //blue-1

int IR_THRESHOLD;
double set = 3500;
double sensorValues[10];
string IR_panel;
double line_le=0;
double kp=0.8;
double kd=0.04;

//wall following
double wall_le=0;
int target=1560;
double wall_kp=0.04;
double wall_kd=0.006;



int timeStep;
//state variable
int state = 0;
int substate =0;

char motorNames[2][15] = { "left motor", "right motor" };
double UltraSonics[3];



bool error_found = 0;
int error_count = 0;
//char IR_panel[10];

////////////////////////////Objects



Motor *motors[5];
DistanceSensor *ds[3];



Robot *robot;
PositionSensor *ps_right;
PositionSensor *ps_left;
double mleft;
double mright;

//control motor speed
double Mdriver(double speed){

 if (speed > 0){
   if (speed > baseSpeed){
     speed = baseSpeed;
   }
 }else{
   if (speed < -baseSpeed){
     speed = -baseSpeed;
   }
 }
 
 return speed;
}

void forward()
    {motors[0]->setVelocity(MAX_SPEED);
     motors[1]->setVelocity(MAX_SPEED);
     } 

void stop()
{
  motors[0]->setVelocity(0);
  motors[1]->setVelocity(0);
}

//----function to read the values of the sensors and convert to binary-------
string read(int ir_threshold=500){
 string IR_panel_val="";
 for (int i = 0; i < 8; i++){
   std::cout<<sensorValues[i]<<" ";
   if (sensorValues[i] > ir_threshold){
   
     sensorValues[i] = 1;
     IR_panel_val +="1";
   }else{
     sensorValues[i] = 0;
     IR_panel_val +="0";
   }
 }
 std::cout<<std::endl;
 return IR_panel_val; 
}
//----------------------end of the read() function----------------------------


//------------------function for PID calculation-------------------------------
//only PD needed

double PID_calc_linefollow()
 
  {
        int panelWeights[8] = {-400,-300,-200,-100,100,200,300,400};
        double position = 0;
        
        int count = 0;

        for (int i = 0; i < 8; i++)
        {
            if (sensorValues[i]  == 1) // For White
            {
                position += panelWeights[i];
                count++;
            } 
        }
        
        //checking for zero division
        if (count == 0) 
        count = 1;
        
          
       double e = position /(count * 10); //error
       double p = kp * e;
       double d = kd * (e - line_le);
       //double i = ki * (e + le);
       double offset = p + d;
       line_le = e;  // le= last error
       return offset;
}
//--------------------end of the PID_calc() function---------------------------

double PID_calc_wallfollowing(double UV_read ){
            
              double e= (target - UV_read);
              double p = wall_kp * e;
              double d = wall_kd * (e - wall_le);
              //double i = ki * (e + le);
              double offset = (p + d);
              
              wall_le = e;  // le= last error
    	   
             return offset;         

}

int main(int argc, char **argv) {
            Robot *robot = new Robot();
            // init Motors
  
	for (int i = 0; i < 2; i++)
	{
		motors[i] = robot->getMotor(motorNames[i]);
		motors[i]->setPosition(INFINITY);
		motors[i]->setVelocity(0.0);
	}
    
    
            // initialize sensors
    
            //object
            DistanceSensor *ir[10];
            
            //name
            char sensorNames[10][5] = {
            "ir0", "ir1", "ir2", "ir3", "ir4",
            "ir5", "ir6", "ir7", "lm", "rm"
            };
            
            //enable the sensors to get measurements
            for (int i = 0; i < 10; i++) {
            ir[i] = robot->getDistanceSensor(sensorNames[i]);
            ir[i]->enable(TIME_STEP);
            }
            
            
           DistanceSensor* us[3];
          char dsNames[3][20] = { "left_ultrasonic", "right_ultrasonic","front_ultrasonic"};
          
          //enable the sensors to get measurements
          for (int i = 0; i < 3; i++) {
          us[i] = robot->getDistanceSensor(dsNames[i]);
          us[i]->enable(TIME_STEP);
          
          }
          
          
          
          
	while (robot->step(TIME_STEP) != -1)
	{
		if (break_main_loop)
		{
			motors[0]->setVelocity(0.0);
			motors[1]->setVelocity(0.0);
			break;
		}
	

		switch (state)
		{
              		case 0:
              		std::cout<<"success"<<std::endl;
                        		forward();
                        		for (int i = 0; i < 10 ; i++){
                                              sensorValues[i] = ir[i]->getValue();
                                              }
                                            IR_THRESHOLD=500; 
                        		IR_panel=read(IR_THRESHOLD); // call a function to get out put as binary values from the IR array
                        		
                        		if (IR_panel != "00000000")//white
                        		{
                          		     stop();
                          		     if (substate==0)
                            		          //state=1;
                              		          IR_THRESHOLD=500;
                              		     else
                              		     /*{ if (colour_var ==0)
                                                        {IR_THRESHOLD=600;
                                                        std::cout<<"im following red line"<<std::endl;}           
                                                    else*/
                                                        IR_THRESHOLD=950; }
                                                  state=1;
                              		     break;}
                              	break;
                        	case 1:
                                      	
                                      	for (int i = 0; i < 10 ; i++){
                                              sensorValues[i] = ir[i]->getValue();
                                              }
                                              
                                             IR_panel=read(IR_THRESHOLD); 
                                              
                        		if (IR_panel == "11111111" and substate ==0)//black
                        		{
                          		     stop();
                          		     state=2;
                                  	     break;}
                                  	     
                                  	else if (IR_panel == "00000000" and substate==1)
                                  	{  stop();
                                      	
                          		     state=3;
                                  	     break;}
                                  	     
                                  	else{
                                      	double offset = PID_calc_linefollow(); //get the offset by calling pre defined function
                          
                                                      //---------------------set motor speed values to minimize the error------------------------
                                                      
                                              //std::cout<<"offset"<<offset<<std::endl;
                                              
                                              double left = baseSpeed - offset;
                                              double right = baseSpeed + offset;
                                                      
                                                      //---call a function to map the above speds within its maximum & minimum speed---
                                                      
                                               double leftSpeed = Mdriver(left);
                                               double rightSpeed = Mdriver(right);
                                                      
                                                      
                                               mleft = leftSpeed;
                                               mright = rightSpeed;
                                                      
                                                      //----------------------pass the speeds to the motor for run------------------------------
                                                      
                                               motors[0]->setVelocity(mleft);
                                               motors[1]->setVelocity(mright);
                                                    
                                                    //-------------print the sensor outputs from the IR array & current offset-----------------
                                             /*std::cout<<"ir0 = "<<sensorValues[0]<<"  ";
                                             std::cout<<"ir1 = "<<sensorValues[1]<<"  ";
                                             std::cout<<"ir2 = "<<sensorValues[2]<<"  ";
                                             std::cout<<"ir3 = "<<sensorValues[3]<<"  ";
                                             std::cout<<"ir4 = "<<sensorValues[4]<<"  ";
                                             std::cout<<"ir5 = "<<sensorValues[5]<<"  ";
                                             std::cout<<"ir6 = "<<sensorValues[6]<<"  ";
                                             std::cout<<"ir7 = "<<sensorValues[7]<<std::endl;
                                                        
                                             std::cout<<" offset : "<<offset<<std::endl;*/
                                             }
                                             
                                             break;
                                           
                               case 2:
                                             for (int i = 0; i < 10 ; i++){
                                              sensorValues[i] = ir[i]->getValue();
                                              }
                                              IR_THRESHOLD=500; 
                              		  IR_panel=read(IR_THRESHOLD); // call a function to get out put as binary values from the IR array
                        		
                        		if (IR_panel == "00000000")//white
                        		{
                          		     stop();
                            		     state=0;
                            		     substate=1;
                              		     break;}   
                              		 
                              		 else{
                                                  for (int i = 0; i < 3 ; i++){
                                                  UltraSonics[i] = us[i]->getValue();
                                                  }
                                            
                                              
                                                  std::cout<<"left = "<<UltraSonics[0]<<left<<std::endl; 
                                                  std::cout<<"right = "<<UltraSonics[1]<<right<<std::endl; 
                                                        
                                                
                                                 
                                                 for (int i=0;i<2;i++){
                                                  
                                                  if (UltraSonics[i]<1800)
                                                  {
                                                  double UV_read= UltraSonics[i];
                                                  double offset = PID_calc_wallfollowing(UV_read);
                                                  
                                                
                                        	   
                                                  double left = baseSpeed + pow(-1,i)*offset;
                                                  
                                                  double right = baseSpeed - pow(-1,i)*offset;
                                                  double leftSpeed = Mdriver(left);
                                                  double rightSpeed = Mdriver(right);
                                                
                                                 mleft = leftSpeed;
                                                 mright = rightSpeed;
                                               
                                                            //----------------------pass the speeds to the motor for run------------------------------
                                     
                                                 motors[0]->setVelocity(mleft);
                                                 motors[1]->setVelocity(mright);
                                                 //std::cout<<offset<<" left spped="<<mleft<<"right speed ="<<mright<<std::endl;
                                                 }
                                                 else{
                                                 motors[0]->setVelocity(MAX_SPEED);
                                                 motors[1]->setVelocity(MAX_SPEED); 
                                                 }
                                                 }
                                             }break;
                                             
                                 case 3:
                                 
                                               break;
                              		  
                                            
                                                        
                            }
                            } 
	// cleanup
	delete robot;
	return 0;	
}
                             
                                            		                                                 
                  
              