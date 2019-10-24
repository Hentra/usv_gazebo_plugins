// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
//#include <gazebo_plugins/gazebo_ros_utils.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Scalar.h>

using namespace gazebo;
using namespace ignition;

void printrot(tf2::Matrix3x3 m){
  printf("\t%f \t%f \t%f \n \t%f \t%f \t%f \n \t%f \t%f \t%f \n",
	 m[0][0],m[0][1],m[0][2],
	 m[1][0],m[1][1],m[1][2],
	 m[2][0],m[2][1],m[2][2]);
}

void printeuler(math::Vector3d e){
  printf(" %f, %f, %f [rpy]\n",e.X(),e.Y(),e.Z());
}
void printq(tf2::Quaternion q){
  tf2::Vector3 v = q.getAxis();
  tf2Scalar w = q.getW();
  printf("%f, %f, %f, %f \n",v[0],v[1],v[2],w);
}

math::Vector3d q2rpy(tf2::Quaternion q){
    tf2::Matrix3x3 m(q);
    math::Vector3d rpy;
    //m.getEulerYPR(rpy.Z(),rpy.Y(),rpy.X());
    tf2::Matrix3x3(q).getEulerYPR(rpy.Z(),rpy.Y(),rpy.X());
    return rpy;
}

tf2::Quaternion rpy2q(math::Vector3d rpy){
  tf2::Matrix3x3 m;
  m.setEulerYPR(rpy.Z(),rpy.Y(),rpy.X());
  printrot(m);
  tf2::Quaternion q;
  m.getRotation(q);
  return q;
}

int main(){

  //math::Vector3d T = math::Vector3d(1.0,0.0,0.0);
  float a = 0.1;
  math::Vector3d T = math::Vector3d(3*cos(a),0.0,-3*sin(a)).Normalize();
  //T = T/T.GetLength();
  math::Vector3d B = math::Vector3d(0.0,10.0,0.0).Normalize();
  math::Vector3d N = math::Vector3d(sin(a),0.0,cos(a)).Normalize();
  /*
  tf2::Matrix3x3 btn = tf2::Matrix3x3(T[0],B[0],N[0],
				      T[1],B[1],N[1],
				      T[2],B[2],N[2]);
  */
  tf2::Matrix3x3 btn = tf2::Matrix3x3(B[1],B[0],B[2],
				      T[1],T[0],T[2],
				      N[1],N[0],N[2]);
  printrot(btn);
  //btn.setIdentity();
  printf("%f,%f,%f\n",btn[0][0],btn[0][1],btn[0][2]);

  // Wave rotation
  tf2::Quaternion wq = tf2::Quaternion();
  btn.getRotation(wq);
  printf("wq: ");
  printq(wq);
  printeuler(q2rpy(wq));
  
  // Express vehicle frame as quat
  math::Vector3d ve = math::Vector3d(0.0,0.0,3.14159/2.0);
  printf("ve: ");
  printeuler(ve);
  tf2::Quaternion vq = rpy2q(ve);
  printf("vq: ");
  printq(vq);
  printeuler(q2rpy(vq));

  // Transfor quaternions
  tf2::Quaternion vwq = wq*vq;
  printf("vwq: ");
  printq(vwq);
  printeuler(q2rpy(vwq));

  /*
  // Veh attitude in wave frame
  // Using trasform
  tf2::Transform tg2btn = tf2::Transform(btn);
  tf2::Quaternion newatt2 = tg2btn*gatt;
  printq(gatt);
  printq(newatt2);

  
  math::Vector3d ne = q2rpy(newatt2);
  printf("ne: \t");
  printeuler(ne);

  math::Vector3d neweuler2;
  tf2::Matrix3x3 n(newatt2);
  n.getRPY(neweuler2.X(),neweuler2.Y(),neweuler2.Z());
  n.getEulerYPR(neweuler2.Z(),neweuler2.Y(),neweuler2.X());
  printf("Veh in Wave: %f : %f : %f \n",neweuler2.X(),
	 neweuler2.Y(),neweuler2.Z());
  // Using quat
  tf2::Quaternion newatt = g2btn*gatt;
  math::Vector3d neweuler;
  tf2::Matrix3x3 m(newatt);
  m.getEulerYPR(neweuler.Z(),neweuler.Y(),neweuler.X());
  printf("Veh in Wave: %f : %f : %f \n",neweuler.X(),
	 neweuler.Y(),neweuler.Z());
  */
}

