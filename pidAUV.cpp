#include <iostream>
using namespace std;


class pidControl

{
public :
        float kp,ki,kd,error,previousValue,previous_error,finalValue,presentValue,derivative,integral,dt,propOut,intOut,derOut,output,maxVal,minVal;
        pidControl();
        void calculate_pid();

};

pidControl::pidControl()
{
	
    finalValue = 0;
    presentValue = 0;
    previous_error=0;
    dt = 0.002;
    minVal=-50;
    maxVal=50;

}

void pidControl::calculate_pid()
{
   error = finalValue - presentValue;   //......proportional term
   propOut = kp * error;

   integral = integral + (error*dt);    //......integral term
   intOut = ki*integral;

   derivative = (error - previous_error) / dt;   //.....derivative term
   derOut = kd * derivative;

   output = propOut + intOut + derOut;

   if(output >= maxVal) 
        output = maxVal;              //......limiting values to possible range

   else if (output <= minVal) 
        output = minVal;        
    

    previous_error  = error;

}



