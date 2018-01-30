#ifndef _SEED_COMMAND_H_
#define _SEED_COMMAND_H_

#include <chrono>
#include <thread>
//SerialCom include
#include "SerialCom.h"
//MecanumWheel include
#include "MecanumWheel.h"

struct Lifter
{
  //Angle of Top Lifter in degrees
  double top;
  //Angle of Bottom Lifter in degrees
  double bottom;
};

class SeedCommand
{
 public:

  static const int ENCODER_THRESHOLD = 10000;

  SeedCommand();
  void seedSetParam(const double wheelRadius,const double tread,const double wheelBase);
  void seedSerialOpen(const std::string portName,int baudRate);
  void seedSerialClose();
  void seedSendServo(unsigned char sendNo,unsigned short sendParam,unsigned char allMode = 1);
  void seedMecanumWheelKinematics(struct Velocity truckSpeed);
  void seedSendTruckSpeed();
  void seedGetWheelEncoder();
  struct Pose seedCalcOdometry();
  bool CheckTargetAngle(Lifter targetLifterAngle);
  bool CheckTargetPose(struct Pose targetLifterPose);
  void seedMoveLifterForwardKinematics(struct Lifter targetLifterAngle,int time = 3);
  void seedMoveLifterReverseKinematics(struct Pose targetLifterPose,int time = 3);
  struct Lifter seedGetLifterAngle();
  struct Pose seedGetLifterPose();
  void resetParam();

  double lifterLength;

 private:

  unsigned int checkSum,checkCount,dataLength;
  double lifterAngleTop,lifterAngleUnder,calcLifterAngleTop,calcLifterAngleUnder;
  int lifterAngleLimit,lifterTop_16,lifterUnder_16;
  struct Wheel_double angularVelocity;
  struct Wheel_int currentEncoder,oldEncoder,diffEncoder,countEncoder,encoderData;

  std::vector<unsigned char> sendData;
  std::vector<unsigned char> receiveData;

};

#endif     //_SEED_COMMAND_H_
