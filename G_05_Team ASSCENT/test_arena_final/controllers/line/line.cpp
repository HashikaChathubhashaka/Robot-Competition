
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <dos.h>

#define TIME_STEP 16
#define MAX_SPEED 6.28

using namespace webots;
using namespace std;




//defining variables and constant
double baseSpeed = 6;
double le = 0;
double set = 3500;
double sensorValues[10];
double mleft;
double mright;

int stage = 1;
int state = 2; // 1==left, 2==right
bool detect = false;
double lpos;
double rpos;
int count = 0;
int c;


//.....only for testing ...............
int given_colour=1 ;// blue
//...........................



// All the webots classes are defined in the "webots" namespace



//----function to read the values of the sensors and convert to binary-------
void read(){
 for (int i = 0; i < 8; i++){
   if (sensorValues[i] > 950){
     sensorValues[i] = 1;
   }else{
     sensorValues[i] = 0;
   }
 }
}
//----------------------end of the read() function----------------------------


//------------------function for PID calculation-------------------------------
//only PD needed

double PID_calc_linefollow(){
 double average = 0;
 double sum = 0;
 for (int i = 0; i < 8 ; i++){ 
   average += sensorValues[i] * i * 1000;
   sum += sensorValues[i];
   
 }
 
 double position = average / sum;  //---------weighted mean---------------------
 
 double kp = 0.01;
 double kd = 0.0006;
 double ki = 0.0001;
 double e = position - set; //error
 double p = kp * e;
 double d = kd * (e - le);
 double i = ki * (e + le);
 double offset = p + d;
 le = e;  // le= last error
 return offset;
}
//--------------------end of the PID_calc() function---------------------------

//---------------------function for motor driving------------------------------
// to not exceed maximum and not discrese minimum value
double Mdriver(double speed){

 if (speed > 0){
   if (speed > 6){
     speed = 6;
   }
 }else{
   if (speed < -6){
     speed = -6;
   }
 }
 
 return speed;
}


int main(int argc, char **argv) {
    Robot *robot = new Robot();
    
    // get a handler to the motors and set target position to infinity (speed control)
    Motor *leftMotor = robot->getMotor("left motor");
    Motor *rightMotor = robot->getMotor("right motor");
    leftMotor->setPosition(INFINITY);
    rightMotor->setPosition(INFINITY);
    

    //------------------------------------------------------------------------
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
 
    leftMotor->setVelocity(0);
    rightMotor->setVelocity(0);
    

  
  while (robot->step(TIME_STEP) != -1) {
  
  
    for (int i = 0; i < 10 ; i++){
            sensorValues[i] = ir[i]->getValue();
            }
            
            
            //--------------read junction detection sensor values----------------
     read(); // call a function to get out put as binary values from the IR array
     
     
     //---------detecting colour---------//
     double  leftMostValue = sensorValues[8];
     double  rightMostValue = sensorValues[9];
     
            
     if (leftMostValue > 800){
                leftMostValue = 1; // blue in left
     }else{
                leftMostValue = 0;
            }
            
            //double  rightMostValue = rightMost->getValue();
      if (rightMostValue > 800){
                rightMostValue = 1;   // blue in right
      }else{
            rightMostValue = 0;
            }
      
              
    std::cout<<" lm "<<leftMostValue<<std::endl; 
    std::cout<<" rm "<<rightMostValue<<std::endl;   
  
  
 
    
    
    double offset = PID_calc_linefollow(); //get the offset by calling pre defined function
            
            //---------------------set motor speed values to minimize the error------------------------
            
    std::cout<<"offset"<<offset<<std::endl;
    
    double left = baseSpeed - offset;
    double right = baseSpeed + offset;
            
            //---call a function to map the above speds within its maximum & minimum speed---
            
     double leftSpeed = Mdriver(left);
     double rightSpeed = Mdriver(right);
            
            
     mleft = leftSpeed;
     mright = rightSpeed;
            
            //----------------------pass the speeds to the motor for run------------------------------
            
     leftMotor->setVelocity(leftSpeed);
     rightMotor->setVelocity(rightSpeed);
            
            
            //-------------print the sensor outputs from the IR array & current offset-----------------
     std::cout<<"ir0 = "<<sensorValues[0]<<"  ";
     std::cout<<"ir1 = "<<sensorValues[1]<<"  ";
     std::cout<<"ir2 = "<<sensorValues[2]<<"  ";
     std::cout<<"ir3 = "<<sensorValues[3]<<"  ";
     std::cout<<"ir4 = "<<sensorValues[4]<<"  ";
     std::cout<<"ir5 = "<<sensorValues[5]<<"  ";
     std::cout<<"ir6 = "<<sensorValues[6]<<"  ";
     std::cout<<"ir7 = "<<sensorValues[7]<<std::endl;
                
     std::cout<<" offset : "<<offset<<std::endl;  
  
  };

 

  delete robot;
  return 0;
}
