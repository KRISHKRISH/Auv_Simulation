# include </home/krish/simulate/Auv_sumilate.cpp>
# include </home/krish/simulate/pidAUV.cpp>
pidControl pitch;
pidControl yaw;
pidControl roll;
pidControl z;
pidControl x_y;
pidControl x;
Point3f Auv_pos;
Point3f E_x_y;
Auv a(0,0,0,0.52,0.40,0.40,0.05,Point3f(0,-0.133,-0.9));

float calculate_pitch()
{
return(a.theta.x);
}
void controll_pitch()
{
	float err; 
err=calculate_pitch();
pitch.presentValue=err;
pitch.calculate_pid();
a.F[0]+=pitch.output;
a.F[1]+=pitch.output;
a.F[2]+=-pitch.output;
a.F[3]+=-pitch.output;
}

void controll_yaw()
{
  float err; 
yaw.calculate_pid();
a.F[5]+=-yaw.output;
a.F[4]+=yaw.output;
}
float calculate_roll()
{
  
return(a.theta.y);
}
void controll_roll()
{
  float err; 
err=calculate_roll();
roll.presentValue=err;
roll.calculate_pid();
a.F[0]+=-roll.output;
a.F[3]+=-roll.output;
a.F[1]+=roll.output;
a.F[2]+=roll.output;
}
float get_z()
{
return(a.pos.z);
}
void controll_z()
{
  float err; 
err=get_z();
z.presentValue=err;
z.finalValue=Auv_pos.z;
z.calculate_pid();
a.F[0]+=-z.output;
a.F[1]+=-z.output;
a.F[2]+=-z.output;
a.F[3]+=-z.output;
}

void controll_x_y()
{
  float err; 
  E_x_y.z=0;
  err=norm(E_x_y );
  if(E_x_y.y<0)err=-err;
x_y.presentValue=err;
x_y.calculate_pid();
yaw.presentValue=atan2(-E_x_y.x,abs(E_x_y.y));
 cout<<"err "<<x_y.output <<endl;
a.F[5]+=x_y.output;
a.F[4]+=x_y.output;
}
void c_x()
{
float err; 
  err=E_x_y.x;
x.presentValue=err;
x.calculate_pid();
roll.finalValue=0.2;
cout<<x.output<<endl;
a.F[1]+=5*x.output;
a.F[2]+=5*x.output;
a.F[0]+=5*x.output;
a.F[3]+=5*x.output;

}
int main()
{ 
	int err;
  Auv_pos=Point3f(0,-1,1); // required final position of the Auv
  a.pos=Point3f(0,0,0);
   pitch.kp=100;pitch.ki=150;pitch.kd=15,pitch.finalValue=0;
roll.kp=125;roll.ki=75;roll.kd=6,roll.finalValue=0;
z.kp=200;z.ki=105;z.kd=10,z.finalValue=0;
x_y.kp=150;x_y.ki=0;x_y.kd=10,x_y.finalValue=0;
yaw.kp=200;yaw.ki=100;yaw.kd=20,yaw.finalValue=0;
x.kp=200;x.ki=100;x.kd=20,x.finalValue=0;
init();
 a.set_Environment();
while(1)
{   
	
	//AUV = cv::Scalar(255,70,70);
    a.Draw_Auv();
   // a.drawTranslate();
    a.Calculate_Force();
    a.updatedParams();  
   //imshow("image", xy_trans_img);
     imshow("AUV", AUV);
    if(cv::waitKey(10)==27)
   {
    break;
   }
   for(int i=0;i<6;i++)
  {
    a.F[i]=0;
  }
  E_x_y=Auv_pos-a.pos;
  E_x_y=Translate(E_x_y,invrotation);
controll_x_y();
 //c_x();
 controll_pitch();
 controll_yaw();
 controll_roll();
 controll_z();
 cout<<a.pos<<endl;
cout<<a.F[0]<<endl;
}


}

