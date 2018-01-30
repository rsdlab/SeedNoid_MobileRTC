#ifndef _MECANUM_WHEEL_H_
#define _MECANUM_WHEEL_H_

//Include
#include <iostream>
#include <stdio.h>
#include <cmath>

struct Wheel_int
{
  //Right front wheel parameter 
  int rightFront;
  //Right back wheel parameter
  int rightBack;
  //Left front wheel parameter
  int leftFront;
  //Left Back wheel parameter
  int leftBack;
};

struct Wheel_double
{
  //Right front wheel parameter 
  double rightFront;
  //Right back wheel parameter
  double rightBack;
  //Left front wheel parameter
  double leftFront;
  //Left Back wheel parameter
  double leftBack;
};

struct Velocity
{
  //Velocity along the x axis in metres per second
  double vx;
  //Velocity along the y axis in metres per second
  double vy;
  //Yaw velocity in radians per second
  double va;
};

struct Pose
{
  //X coordinate in metres
  double x;
  //Y coordinate in metres
  double y;
  //Z coordinate in metres
  double z;
  //Heading in dadians
  double heading;
};


class MecanumWheel
{
 public:

  //----------Constructor----------
  MecanumWheel()
    {
      //Seednoid Param
      wheelRadiusParam = 0.076;
      treadParam = 0.39;
      wheelBaseParam = 0.453;

      currentPose = {0,0,0,0};
      oldPose = {0,0,0,0};

      positionLog = {0,0,0,0};

      movementValue = {0,0,0,0};
    }
  
  //----------Function----------
  //----------Set Wheel Radius Function----------
  void setWheelRadius(double wheelRadius)
  {
    wheelRadiusParam = wheelRadius;
  }

  //----------Set Tread Function----------
  void setTread(double tread)
  {
    treadParam = tread;
  }

  //----------Set Wheel Base Function----------
  void setWheelBase(double wheelBase)
  {
    wheelBaseParam = wheelBase;
  }

  //----------MecanumWheel Kinematics Function----------
  struct Wheel_double calcMecanumWheelKinematics(struct Velocity truckSpeed)
  {
    struct Wheel_double angularVelocity;

    angularVelocity.rightFront = 60 * (truckSpeed.vx + truckSpeed.vy + ((treadParam / 2) + (wheelBaseParam / 2)) * truckSpeed.va) / wheelRadiusParam;     //Right front wheel angular velocity [rpm]
    angularVelocity.rightBack = 60 * (truckSpeed.vx - truckSpeed.vy + ((treadParam / 2) + (wheelBaseParam / 2)) * truckSpeed.va) / wheelRadiusParam;     //Right Back wheel angular velocity [rpm]
    angularVelocity.leftFront = 60 * (truckSpeed.vx - truckSpeed.vy - ((treadParam / 2) + (wheelBaseParam / 2)) * truckSpeed.va) / wheelRadiusParam;     //Left right wheel angular velocity [rpm]
    angularVelocity.leftBack = 60 * (truckSpeed.vx + truckSpeed.vy - ((treadParam / 2) + (wheelBaseParam / 2)) * truckSpeed.va) / wheelRadiusParam;     //Left Back wheel angular velocity [rpm]

    return angularVelocity;
  }

  //----------MecanumWheel Odometry Function----------
  struct Pose calcMecanumWheelOdometry(struct Wheel_int encoderData)
  {
    struct Pose odometryData;

    movementValue.rightFront  = 2 * M_PI * wheelRadiusParam * encoderData.rightFront / 360;
    movementValue.rightBack = 2 * M_PI * wheelRadiusParam * encoderData.rightBack / 360;
    movementValue.leftFront = 2 * M_PI * wheelRadiusParam * encoderData.leftFront / 360;
    movementValue.leftBack = 2 * M_PI * wheelRadiusParam * encoderData.leftBack / 360;

    currentPose.x = (movementValue.rightFront + movementValue.rightBack + movementValue.leftFront + movementValue.leftBack) / 4;
    currentPose.y = (movementValue.rightFront - movementValue.rightBack - movementValue.leftFront + movementValue.leftBack) / 4;
    currentPose.heading = (movementValue.rightFront + movementValue.rightBack - movementValue.leftFront - movementValue.leftBack) / (4 * ((treadParam / 2) + (wheelBaseParam / 2)));

    odometryData.x = positionLog.x + (currentPose.x - oldPose.x) * std::cos(currentPose.heading) - (currentPose.y - oldPose.y) * sin(currentPose.heading);
    odometryData.y = positionLog.y + (currentPose.x - oldPose.x) * std::sin(currentPose.heading) + (currentPose.y - oldPose.y) * cos(currentPose.heading);
    odometryData.heading = currentPose.heading;

    if(std::abs(currentPose.x - oldPose.x) < 2.0)
      {
	oldPose.x = currentPose.x;
      }
    if(std::abs(currentPose.y - oldPose.y) < 2.0)
      {
	oldPose.y = currentPose.y;
      }

    if(std::abs(odometryData.x - positionLog.x) < 2.0)
      {
        positionLog.x = odometryData.x;
      }
    if(std::abs(odometryData.y - positionLog.y) < 2.0)
      {
        positionLog.y = odometryData.y;
      }

    return odometryData;
  }

  //----------Param Reset Function----------
  void resetParam()
  {
    currentPose = {0,0,0,0};
    oldPose = {0,0,0,0};

    positionLog = {0,0,0,0};

    movementValue = {0,0,0,0};
  }
  
 private:
  
  double wheelRadiusParam,treadParam,wheelBaseParam;
  struct Pose currentPose;
  struct Pose oldPose;
  struct Pose positionLog;
  struct Wheel_double movementValue;
};

#endif     //_MECANUM_WHEEL_H_
