#include "opencv2/opencv.hpp"
#include "math.h"
using namespace std;
using namespace cv;
cv::Point3f operator*(cv::Mat M, const cv::Point3f& p)
{ 
    cv::Mat_<double> src(3/*rows*/,1 /* cols */); 

    src(0,0)=p.x; 
    src(1,0)=p.y; 
    src(2,0)=p.z; 

    cv::Mat_<double> dst = M*src; //USE MATRIX ALGEBRA 
    return cv::Point3f(dst(0,0),dst(1,0),dst(2,0)); 
} 
  Mat AUV(600,600, CV_8UC3, Scalar(0,0,0));  // the image which displays the orientation of AUV
  Mat Env(600,600, CV_8UC3, Scalar(0,0,0));  // the image which displays the enviroment
  Mat xy_trans_img(600,600, CV_8UC3, Scalar(0,0,0));  // the image which displays the orientation of AUV
	Mat cameraMatrix=Mat::zeros(3, 3, CV_32F); // The matrix which stores the parameters of the camera
	Mat distCoeffs(4,1,cv::DataType<double>::type); // the distortion coefficients of the camera
  Mat tvec(3,1,cv::DataType<double>::type);   // the traslation vector of the auv with respect to frame of the 
  Mat rotation(3,3,cv::DataType<float>::type); // identity matrix
  Mat invrotation(3,3,cv::DataType<float>::type); // identity matrix
  Mat R;
  Mat momentOfInertia =(Mat_<double>(3,3) <<                 // the moment of inertia metrix (Depends on the vehicle)
               3.3670,       0,              0,
               0,       13.4012,           0,
               0,       0 ,                3.3670
               );
  vector<Point3f> Translate(vector<Point3f> objPoints)     // rotate object points with respect to origin of the object
  {
    
    vector<Point3f> transPoints;
     Mat mat;
     for(int i = 0; i<objPoints.size(); i++) 
     { 
      mat=rotation*(Mat(objPoints[i]));
       Point3f points(mat);
       transPoints.push_back(points);
     }
    return transPoints; 
  }
  Point3f Translate(Point3f objPoints)     // rotate object points with respect to origin of the object
  {
    
       Mat mat;
      mat=rotation*(Mat(objPoints));
       Point3f points(mat);
     
    return points; 
  }
  Point3f Translate(Point3f objPoints,Mat m)     // rotate object points with respect to origin of the object
  {
    
       Mat mat;
      mat=m*(Mat(objPoints));
       Point3f points(mat);
     
    return points; 
  }
Mat eulerAnglesToRotationMatrix(Point3f theta) // euler Angles to rotation
{
    // Calculate rotation about x axis
    Mat R_x = (Mat_<float>(3,3) <<
               1,       0,              0,
               0,       cos(theta.x),   -sin(theta.x),
               0,       sin(theta.x),   cos(theta.x)
               );
     
    // Calculate rotation about y axis
    Mat R_y = (Mat_<float>(3,3) <<
               cos(theta.y),    0,      sin(theta.y),
               0,               1,      0,
               -sin(theta.y),   0,      cos(theta.y)
               );
     
    // Calculate rotation about z axis
    Mat R_z = (Mat_<float>(3,3) <<
               cos(theta.z),    -sin(theta.z),      0,
               sin(theta.z),    cos(theta.z),       0,
               0,               0,                  1);
     
     
    // Combined rotation matrix
    Mat R = R_z * R_y * R_x;
    return R;
 
}

class Auv 
{
public:
  float length,breadth,width,depth,dt,mass;
  Point3f theta,alpha,omega,velocity,accleration,pos,previousPos;
  Point3f Center_of_Mass,boyance,Torque,Force; 
  vector<Point3f> objPoints; // points to draw axis
  vector<Point3f> cordinatePoints; // points to draw axis
  vector<Point2f> imgPoints; 
  vector<Point3f> transPoints; 
//Force defenitions 
  float F[6],F_boynt;
  //Auv();
  Auv(float r,float p,float y,float l,float b,float w,float d,Point3f boyn);
  void Orientation();
  void Draw_Auv();
  Point3f Calculate_Force();
  void updatedParams() ;
  void drawTranslate();
  void draw_cordinates();
  void set_Environment();
};
 Auv::Auv(float r,float p,float y,float l,float b,float w,float d,Point3f boyn)
  {
    dt=0.001;
theta=Point3f(r,p,y);
alpha=Point3f(0,0,0);
omega=Point3f(0,0,0);
pos=Point3f(0,0,0);
velocity=Point3f(0,0,0);
previousPos=Point3f(0,0,0);
accleration=Point3f(0,0,0);
length=l;
breadth=b;
width=w;
depth=-d;
mass=18;
boyance=boyn;
  Center_of_Mass=Point3f(0,-0.133,-0.13);
  /* objectpoint convention 
                                                            
       1 _____________2            (into)z o -------------> x
        |             |                    |              
        |             |                    |
 _______|             |________            |                        // width 5-6 length 1-4 breadth 1-2 
 5      |             |       6            |        
        |             |                    | y
       4|_____________|3                   v

    */
   objPoints.push_back(Point3f(-breadth/2,-length/2,0)-Center_of_Mass);
  objPoints.push_back(Point3f(breadth/2,-length/2,0)-Center_of_Mass);
  objPoints.push_back(Point3f(breadth/2,length/2,0)-Center_of_Mass);
  objPoints.push_back(Point3f(-breadth/2,length/2,0)-Center_of_Mass);
  objPoints.push_back(Point3f(-width/2,0,depth)-Center_of_Mass);
  objPoints.push_back(Point3f(width/2,0,depth)-Center_of_Mass);
  objPoints.push_back(Point3f(0,0,0));
  objPoints.push_back(Point3f(.1,0,0));
  objPoints.push_back(Point3f(0,.1,0));
  objPoints.push_back(Point3f(0,0,.1));
  objPoints.push_back(boyance-Center_of_Mass);
  cordinatePoints.push_back(Point3f(-4,-8,6));// 00
  cordinatePoints.push_back(Point3f(6,-8,6));// x
  cordinatePoints.push_back(Point3f(-4,2,6));// y
  cordinatePoints.push_back(Point3f(-4,-8,-2));//z
  cordinatePoints.push_back(Point3f(-4,-8,6)+Point3f(0,-10,0));// 00
  cordinatePoints.push_back(Point3f(6,-8,6)+Point3f(0,-10,0));// x
  cordinatePoints.push_back(Point3f(-4,2,6)+Point3f(0,-10,0));// y
  cordinatePoints.push_back(Point3f(-4,-8,-2)+Point3f(0,-10,0));//z
  cordinatePoints.push_back(Point3f(6,-8,-2)+Point3f(0,-10,0));//xz
  

  F_boynt=200;
  //F_boynt=0;
  for(int i=0;i<6;i++)
  {
    F[i]=0;
  }
  }
  void Auv::Orientation()
{
rotation=eulerAnglesToRotationMatrix(theta);
}
void Auv::Draw_Auv()
{ Orientation();
  
	/*projectPoints(cordinatePoints,R,tvec,cameraMatrix,distCoeffs,imgPoints);
line(AUV,imgPoints[0],imgPoints[1], Scalar(0,0,255), 5, LINE_AA); 
line(AUV,imgPoints[0],imgPoints[2], Scalar(0,255,0), 5, LINE_AA); 
line(AUV,imgPoints[0],imgPoints[3], Scalar(255,0,0), 5, LINE_AA);
*/ 
	Env.copyTo(AUV);
  transPoints=Translate(objPoints);
  Mat B;
  Mat(pos).convertTo(B,cv::DataType<double>::type);
projectPoints(transPoints,R,tvec+B,cameraMatrix,distCoeffs,imgPoints);
//projectPoints(transPoints,R,tvec,cameraMatrix,distCoeffs,imgPoints);
line(AUV,imgPoints[0],imgPoints[1], Scalar(0,0,100), 5, LINE_AA); 
line(AUV,imgPoints[1],imgPoints[2], Scalar(0,0,100), 5, LINE_AA); 
line(AUV,imgPoints[2],imgPoints[3], Scalar(0,0,100), 5, LINE_AA); 
line(AUV,imgPoints[0],imgPoints[3], Scalar(0,0,100), 5, LINE_AA); 
line(AUV,imgPoints[4],imgPoints[5], Scalar(0,0,100), 5, LINE_AA);  
line(AUV,imgPoints[6],imgPoints[7], Scalar(0,0,255), 5, LINE_AA); 
line(AUV,imgPoints[6],imgPoints[8], Scalar(0,255,0), 5, LINE_AA); 
line(AUV,imgPoints[6],imgPoints[9], Scalar(255,0,0), 5, LINE_AA); 
}
Point3f Auv::Calculate_Force()
{
  Torque=Point3f(0,0,0);
  Force = Point3f(0,0,0);
  Point3f vect;
for(int i=0;i<4;i++)
{
  vect=transPoints[i];
 Torque=Torque+F[i]*(vect.cross(-(transPoints[9])))/(norm(transPoints[9]));
 Force=Force+F[i]*(-(transPoints[9]))/(norm(transPoints[9]));
}
for(int i=4;i<6;i++)
{
  vect=transPoints[i];
 Torque=Torque+F[i]*(vect.cross(-(transPoints[8])))/(norm(transPoints[8]));
 Force=Force+F[i]*(-(transPoints[8]))/(norm(transPoints[8]));
}
vect=transPoints[10];

Torque=Torque+F_boynt*(vect.cross(-(Point3f(0,0,1))));
Force=Force+((mass*10)-F_boynt)*objPoints[9];
vector<Point3f> o;
/*o.push_back(-(Point3f(0,0,1)));
o.push_back(vect);
o.push_back(vect.cross(-(Point3f(0,0,1))));
projectPoints(o,R,tvec,cameraMatrix,distCoeffs,imgPoints);
line(AUV,imgPoints[0],imgPoints[6], Scalar(0,100,100), 5, LINE_AA); 
line(AUV,imgPoints[1],imgPoints[6], Scalar(0,100,100), 5, LINE_AA); 
line(AUV,imgPoints[3],imgPoints[6], Scalar(0,200,0), 5, LINE_AA); 
*/
}
void Auv::updatedParams()    
{
	 invert(rotation,invrotation);
  Torque=Translate(Torque,invrotation);
   alpha = momentOfInertia* Torque;
      omega = 0.99*omega+(alpha*dt);
       theta = theta+(omega*dt);
      accleration = Force/mass;
    velocity  = 0.99*velocity + (accleration*dt);
      pos = pos + (velocity*dt);
   
   }
 void Auv::drawTranslate()
{
	projectPoints(cordinatePoints,R,tvec,cameraMatrix,distCoeffs,imgPoints);
line(xy_trans_img,imgPoints[0],imgPoints[1], Scalar(0,0,255), 5, LINE_AA); 
line(xy_trans_img,imgPoints[0],imgPoints[2], Scalar(0,255,0), 5, LINE_AA); 
line(xy_trans_img,imgPoints[0],imgPoints[3], Scalar(255,0,0), 5, LINE_AA); 
  vector<Point3f> o;
o.push_back(pos);
o.push_back(previousPos);
projectPoints(o,R,tvec,cameraMatrix,distCoeffs,imgPoints);
line(xy_trans_img,imgPoints[0],imgPoints[1], Scalar(0,200,0), 5, LINE_AA); 

  previousPos=pos;

}
void Auv::set_Environment()
{
	
projectPoints(cordinatePoints,R,tvec,cameraMatrix,distCoeffs,imgPoints);
Env= cv::Scalar(255,70,70);
line(Env,imgPoints[0],imgPoints[1], Scalar(0,0,255), 5, LINE_AA); 
line(Env,imgPoints[0],imgPoints[2], Scalar(0,255,0), 5, LINE_AA); 
line(Env,imgPoints[0],imgPoints[3], Scalar(255,0,0), 5, LINE_AA);

}
 void init()
 {
 	R=eulerAnglesToRotationMatrix(Point3f(-1,0,0));
 	R.convertTo(R, cv::DataType<double>::type);
  cameraMatrix.at<float>(0, 0) =600.0;
  cameraMatrix.at<float>(0, 1) =0;
  cameraMatrix.at<float>(0, 2) =300;
  cameraMatrix.at<float>(1, 0) =0;
  cameraMatrix.at<float>(1, 1) =600;
  cameraMatrix.at<float>(1, 2) =300;
  cameraMatrix.at<float>(2, 0) =0;
  cameraMatrix.at<float>(2, 1) =0;
  cameraMatrix.at<float>(2, 2) =1;
  distCoeffs.at<double>(0) = 0;
  distCoeffs.at<double>(1) = 0;
  distCoeffs.at<double>(2) = 0;
  distCoeffs.at<double>(3) = 0;
  tvec.at<double>(0,0)=0;
  tvec.at<double>(1,0)=0;
  tvec.at<double>(2,0)=3;



 }
