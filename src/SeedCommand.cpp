#include "SeedCommand.h"

SerialCom sc;
MecanumWheel meca;

SeedCommand::SeedCommand()
{
  checkSum = 0;
  checkCount = 0;
  dataLength = 0;

  lifterLength = 0.25; /*0.25[m]*/
  lifterAngleLimit = 85;
  lifterAngleTop = lifterAngleUnder = 0.0;
  lifterTop_16 = lifterUnder_16 = calcLifterAngleTop = calcLifterAngleUnder = 0;

  angularVelocity = {0,0,0,0};
  currentEncoder = {0,0,0,0};
  oldEncoder = {0,0,0,0};
  diffEncoder = {0,0,0,0};
  countEncoder = {0,0,0,0};
  encoderData = {0,0,0,0};
}

void SeedCommand::seedSetParam(const double wheelRadius,const double tread,const double wheelBase)
{
  meca.setWheelRadius(wheelRadius);
  meca.setTread(tread);
  meca.setWheelBase(wheelBase);
}

void SeedCommand::seedSerialOpen(const std::string portName, const int baudRate)
{
  if(sc.serialOpen(portName.c_str(),baudRate))
    {
      std::cout << "Connect!" << std::endl;
    }
  else
    {
      std::cout << "Connecr Error!" << std::endl;
    }
}

void SeedCommand::seedSerialClose()
{
  if(sc.serialClose())
    {
      std::cout << "Dis Connect!" << std::endl;
    }
  else
    {
      std::cout << "Dis Connece Error!" << std::endl;
    }
}

void SeedCommand::seedSendServo(unsigned char sendNo,unsigned short sendParam,unsigned char allMode)
{
  checkSum = 0;
  if(allMode)
    {
      dataLength = 68;
      sendData.resize(dataLength);

      sendData[0] = 0xFD;
      sendData[1] = 0xDF;
      sendData[2] = 64;
      sendData[3] = 0x21;
      sendData[4] = 0x00;

      for(int i(5); i < 65; i+=2)
	{
	  sendData[i] = 0x00;
	  sendData[i+1] = sendParam;
	}

      sendData[65] = 0x00;
      sendData[66] = 0x00;
    }
  else
    {
      dataLength = 8;
      sendData.resize(dataLength);

      sendData[0] = 0xFD;
      sendData[1] = 0xDF;
      sendData[2] = 4;
      sendData[3] = 0x21;
      sendData[4] = sendNo;
      sendData[5] = sendParam >> 8;
      sendData[6] = sendParam;
    }
      
  //CheckSum
  for(checkCount = 2;checkCount < dataLength-1;checkCount++)
    {
      checkSum += sendData[checkCount];
    }
  sendData[dataLength-1] = ~checkSum;

  /*
  for(int i(0);i < 68;i++)
    {
      std::cout << std::uppercase << std::setw(2) << std::setfill('0') << static_cast<int>(sendData[i]);
    }
  std::cout << std::endl;
  */  

  sc.serialAsyncWrite(sendData);
}

void SeedCommand::seedMecanumWheelKinematics(struct Velocity truckSpeed)
{
  angularVelocity = meca.calcMecanumWheelKinematics(truckSpeed);
}

void SeedCommand::seedSendTruckSpeed()
{
  dataLength = 68;
  checkSum = 0;
  sendData.resize(dataLength);

  sendData[0] = 0xFD;
  sendData[1] = 0xDF;
  sendData[2] = 64;
  sendData[3] = 0x15;
  sendData[4] = 0x00;

  for(int i(0);i < 4;i++)
    {
      sendData[i+5] = 0x00;
    }

  sendData[9] = static_cast<int>(-angularVelocity.rightFront) >> 8;
  sendData[10] = static_cast<int>(-angularVelocity.rightFront);
  sendData[11] = static_cast<int>(-angularVelocity.rightBack) >> 8;
  sendData[12] = static_cast<int>(-angularVelocity.rightBack);
  sendData[13] = static_cast<int>(angularVelocity.leftFront) >> 8;
  sendData[14] = static_cast<int>(angularVelocity.leftFront);
  sendData[15] = static_cast<int>(angularVelocity.leftBack) >> 8;
  sendData[16] = static_cast<int>(angularVelocity.leftBack);

  for(unsigned int i(0);i < 50;i++)
    {
      sendData[i+17] = 0x00;
    }

  //CheckSum
  for(checkCount = 2;checkCount < dataLength-1;checkCount++)
    {
      checkSum += sendData[checkCount];
    }
  sendData[dataLength-1] = ~checkSum;

  /*
  for(int i(9);i < 17;i+=2)
    {
      printf("%d,%d 0x%02X%02X\n", i,i+1,sendData[i],sendData[i+1]);
    }
  */

  sc.serialAsyncWrite(sendData);
}


void SeedCommand::seedGetWheelEncoder()
{
  checkSum = 0;
  dataLength = 6;
  sendData.resize(dataLength);

  sendData[0] = 0xFD;
  sendData[1] = 0xDF;
  sendData[2] = 2;
  sendData[3] = 0x41;
  sendData[4] = 0x00;

  //CheckSum
  for(checkCount = 2;checkCount < dataLength-1;checkCount++)
    {
      checkSum += sendData[checkCount];
    }
  sendData[dataLength-1] = ~checkSum;


  sc.flushSerialPort();
  sc.serialAsyncWrite(sendData);

  //std::chrono::milliseconds dura(10);
  //std::this_thread::sleep_for(dura);     //6milliseconds sleep

  sc.serialAsyncRead(receiveData,68);
  
  /*
  std::cout << "sendData:";
  for(int i(0);i < 6;++i)
    {
      std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(sendData[i]);
    }
  std::cout << std::endl;
  std::cout << "receiveData:";
  for(int i(0);i < 68;++i)
    {
      std::cout << std::uppercase << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(receiveData[i]);
    }
  std::cout << std::endl;
  */

  if(receiveData[0] == 0xDF && receiveData[3] == 0x41)
    {
      //Get encoder value of each wheels
      currentEncoder.rightFront = (static_cast<int>(receiveData[9] << 8) | (static_cast<int>(receiveData[10])));     //Right front wheel Hexadecimal angle data [deg]
      currentEncoder.rightBack = (static_cast<int>(receiveData[11] << 8) | (static_cast<int>(receiveData[12])));     //Right rear wheel Hexadecimal angle data [deg]
      currentEncoder.leftFront = (static_cast<int>(receiveData[13] << 8) | (static_cast<int>(receiveData[14])));     //Left front wheel Hexadecimal angle data [deg]
      currentEncoder.leftBack = (static_cast<int>(receiveData[15] << 8) | (static_cast<int>(receiveData[16])));     //Left rear wheel Hexadecimal angle data [deg]
    }

  //Encoder difference calculation
  diffEncoder.rightFront = currentEncoder.rightFront - oldEncoder.rightFront;
  diffEncoder.rightBack = currentEncoder.rightBack - oldEncoder.rightBack;
  diffEncoder.leftFront = currentEncoder.leftFront - oldEncoder.leftFront;
  diffEncoder.leftBack = currentEncoder.leftBack - oldEncoder.leftBack;

  //Assignment of encoder value
  oldEncoder.rightFront = currentEncoder.rightFront;
  oldEncoder.rightBack = currentEncoder.rightBack;
  oldEncoder.leftFront = currentEncoder.leftFront;
  oldEncoder.leftBack = currentEncoder.leftBack;

  //Count encoder cycle(27204)
  //Right front wheel count
  if(diffEncoder.rightFront > 10000/*ENCODER_THRESHOLD*/)
    {
      countEncoder.rightFront --;
    }
  else if(diffEncoder.rightFront < -10000/*-ENCODER_THRESHOLD*/)
    {
      countEncoder.rightFront ++;
    }
  //Right rear wheel count
  if(diffEncoder.rightBack > 10000/*ENCODER_THRESHOLD*/)
    {
      countEncoder.rightBack --;
    }
  else if(diffEncoder.rightBack < -10000/*-ENCODER_THRESHOLD*/)
    {
      countEncoder.rightBack ++;
    }
  //Left front wheel count
  if(diffEncoder.leftFront > 10000/*ENCODER_THRESHOLD*/)
    {
      countEncoder.leftFront --; 
    }
  else if(diffEncoder.leftFront < -10000/*-ENCODER_THRESHOLD*/)
    {    
      countEncoder.leftFront ++;
    }
  //Left rear wheel count
  if(diffEncoder.leftBack > 10000/*ENCODER_THRESHOLD*/)
    {
      countEncoder.leftBack --;
    }
  else if(diffEncoder.leftBack < -10000/*-ENCODER_THRESHOLD*/)
    {
      countEncoder.leftBack ++;
    }

  //Calculate degrees of wheel rotation angle
  encoderData.rightFront = currentEncoder.rightFront + 27204 * countEncoder.rightFront;     //Right front wheel rotation angle data [deg]
  encoderData.rightBack = currentEncoder.rightBack + 27204 * countEncoder.rightBack;     //Right rear wheel roration angle data [deg]
  encoderData.leftFront = currentEncoder.leftFront + 27204 * countEncoder.leftFront;     //Left ftonr wheel rotation angle data [deg]
  encoderData.leftBack = currentEncoder.leftBack + 27204 * countEncoder.leftBack;     //Left rear wheel roration angle data [deg]

  //Adjustment of rotation direction
  encoderData.rightFront = - encoderData.rightFront;
  encoderData.rightBack = - encoderData.rightBack;
}


struct Pose SeedCommand::seedCalcOdometry()
{
  struct Pose odometryData;

  odometryData = meca.calcMecanumWheelOdometry(encoderData);

  return odometryData;
}


bool SeedCommand::CheckTargetAngle(Lifter targetLifterAngle)
{
  if(targetLifterAngle.top < 0 || 90 < targetLifterAngle.top || targetLifterAngle.bottom < 0 || 90 < targetLifterAngle.bottom)
    {
      //std::cout << "Out of calculation range" <<std::endl;
      return false;
    }
  else
    {
      //std::cout << "Range of movement" << std::endl;
      return true;
    }
}


bool SeedCommand::CheckTargetPose(struct Pose targetLifterPose)
{
  //Determination of movable range of lifter
  //Lifter top
  if(targetLifterPose.z >= lifterLength)
    {
      if((pow((0.0 - targetLifterPose.x),2) + pow((lifterLength - targetLifterPose.z),2)) <= pow(lifterLength,2))
	{
	  //std::cout << "Range of movement" << std::endl;
	  return true;
	}
      else
	{
	  //std::cout << "Out of calculation range" << std::endl;
	  return false;
	}
    }
  //Lifter under
  else if(0 <= targetLifterPose.z && targetLifterPose.z < lifterLength)
    {
      //Front direction
      if(0 <= targetLifterPose.x && targetLifterPose.x <= lifterLength)
	{
	  if((pow((lifterLength - targetLifterPose.x),2) + pow((0.0 - targetLifterPose.z),2)) >= pow(lifterLength,2))
	    {
	      //std::cout << "Range of movement" << std::endl;
	      return true;
	    }
	  else
	    {
	      //std::cout << "Out of calculation range" << std::endl;
	      return false;
	    }
	}
      //Back direction
      else if(targetLifterPose.x < 0 && -lifterLength <= targetLifterPose.x)
	{
	  if((pow((-lifterLength - targetLifterPose.x),2) + pow((0.0 - targetLifterPose.z),2)) >= pow(lifterLength,2))
	    {
	      //std::cout << "Range of movement" << std::endl;
	      return true;
	    }
	  else
	    {
	      //std::cout << "Out of calculation range" << std::endl;
	      return false;
	    }
	}
      else
	{
	  //std::cout << "Out of calculation range" << std::endl;
	  return false;
	}
    }
  else
    {
      //std::cout << "Out of calculation range" << std::endl;
      return false;
    }
}


void SeedCommand::seedMoveLifterForwardKinematics(struct Lifter targetLifterAngle,int time)
{
  //Time = time * 10[ms]
  if(!CheckTargetAngle(targetLifterAngle))
    {
      std::cout << "Out of calculation range" << std::endl;
    }
  else{
    if(85 < targetLifterAngle.top && targetLifterAngle.top <= 90)
      {
	targetLifterAngle.top = 85;
      }

    if(85 < targetLifterAngle.bottom && targetLifterAngle.bottom <= 90)
      {
	targetLifterAngle.bottom = 85;
      }

    lifterTop_16 = (int)((0.0000000000189750997846207 * std::pow(targetLifterAngle.top,6) - 0.00000000720603482440296 * std::pow(targetLifterAngle.top,5) + 0.00000121514687201341 * std::pow(targetLifterAngle.top,4) - 0.000131421951692801 * std::pow(targetLifterAngle.top,3) + 0.00697190814844362 * std::pow(targetLifterAngle.top,2) + 0.617198185427696 * targetLifterAngle.top - 0.00145630350050144) * 100);
    lifterUnder_16 = (int)((0.0000000000189750997846207 * std::pow(targetLifterAngle.bottom,6) - 0.00000000720603482440296 * std::pow(targetLifterAngle.bottom,5) + 0.00000121514687201341 * std::pow(targetLifterAngle.bottom,4) - 0.000131421951692801 * std::pow(targetLifterAngle.bottom,3) + 0.00697190814844362 * std::pow(targetLifterAngle.bottom,2) + 0.617198185427696 * targetLifterAngle.bottom - 0.00145630350050144) * 100);

    dataLength = 68;
    sendData.resize(dataLength);
    sendData[0] = 0xFD;
    sendData[1] = 0xDF;
    sendData[2] = 64;
    sendData[3] = 0x14;
    sendData[4] = 0x00;
    sendData[5] = static_cast<int>(lifterTop_16) >> 8;
    sendData[6] = static_cast<int>(lifterTop_16);
    sendData[7] = static_cast<int>(lifterUnder_16) >> 8;
    sendData[8] = static_cast<int>(lifterUnder_16);

    for(int i(0);i < 56;i++)
      {
	sendData[i+9] = 0x00;
      }

    sendData[65] = static_cast<int>(time / 10) >> 8;
    sendData[66] = static_cast<int>(time / 10);

    //CheckSum
    for(checkCount = 2;checkCount < dataLength-1;checkCount++)
      {
	checkSum += sendData[checkCount];
      }
    sendData[dataLength-1] = ~checkSum;

    sc.serialAsyncWrite(sendData);
  }
}


void SeedCommand::seedMoveLifterReverseKinematics(struct Pose targetLifterPose,int time)
{
  //Time = time * 10[ms]
  if(!CheckTargetPose(targetLifterPose))
    {
      std::cout << "Out of calculation range" << std::endl;
    }
  else
    {
      calcLifterAngleTop = std::acos((std::pow(targetLifterPose.x,2) + std::pow(targetLifterPose.z,2) - 2 * std::pow(lifterLength,2)) / (2 * std::pow(lifterLength,2)));

      if((calcLifterAngleTop * 180 / M_PI) > 0)
	{
	  calcLifterAngleTop = -calcLifterAngleTop;
	}

      calcLifterAngleUnder = std::asin(-(((targetLifterPose.x * lifterLength * std::sin(calcLifterAngleTop)) - (targetLifterPose.z * (lifterLength + lifterLength * std::cos(calcLifterAngleTop)))) / (std::pow((lifterLength + lifterLength * std::cos(calcLifterAngleTop)),2) + std::pow(lifterLength,2) * std::pow(std::sin(calcLifterAngleTop),2))));

      //Change Radian to Degree
      calcLifterAngleTop = (calcLifterAngleTop * 180 / M_PI);
      calcLifterAngleUnder = (calcLifterAngleUnder * 180 / M_PI);
  
      if(calcLifterAngleUnder < 90 || 180 < calcLifterAngleUnder)
	{
	  calcLifterAngleUnder = 180 - calcLifterAngleUnder;
	}
  
      lifterAngleUnder = calcLifterAngleUnder - 90;
      lifterAngleTop = 0 - (calcLifterAngleTop + (calcLifterAngleUnder - 90));

      if(85 < lifterAngleTop && lifterAngleTop <= 90)
	{
	  lifterAngleTop = 85;
	}

      if(85 < lifterAngleUnder && lifterAngleUnder <= 90)
	{
	  lifterAngleUnder = 85;
	}

      lifterTop_16 = (int)((0.0000000000189750997846207 * std::pow(lifterAngleTop,6) - 0.00000000720603482440296 * std::pow(lifterAngleTop,5) + 0.00000121514687201341 * std::pow(lifterAngleTop,4) - 0.000131421951692801 * std::pow(lifterAngleTop,3) + 0.00697190814844362 * std::pow(lifterAngleTop,2) + 0.617198185427696 * lifterAngleTop - 0.00145630350050144) * 100);
      lifterUnder_16 = (int)((0.0000000000189750997846207 * std::pow(lifterAngleUnder,6) - 0.00000000720603482440296 * std::pow(lifterAngleUnder,5) + 0.00000121514687201341 * std::pow(lifterAngleUnder,4) - 0.000131421951692801 * std::pow(lifterAngleUnder,3) + 0.00697190814844362 * std::pow(lifterAngleUnder,2) + 0.617198185427696 * lifterAngleUnder - 0.00145630350050144) * 100);

      dataLength = 68;
      sendData.resize(dataLength);

      sendData[0] = 0xFD;
      sendData[1] = 0xDF;
      sendData[2] = 64;
      sendData[3] = 0x14;
      sendData[4] = 0x00;
      sendData[5] = static_cast<int>(lifterTop_16) >> 8;
      sendData[6] = static_cast<int>(lifterTop_16);
      sendData[7] = static_cast<int>(lifterUnder_16) >> 8;
      sendData[8] = static_cast<int>(lifterUnder_16);

      for(int i(0);i < 56;i++)
	{
	  sendData[i+9] = 0x00;
	}

      sendData[65] = static_cast<int>(time / 10) >> 8;
      sendData[66] = static_cast<int>(time / 10);

      //CheckSum
      for(checkCount = 2;checkCount < dataLength-1;checkCount++)
	{
	  checkSum += sendData[checkCount];
	}
      sendData[dataLength-1] = ~checkSum;

      sc.serialAsyncWrite(sendData);
    }
}


struct Lifter SeedCommand::seedGetLifterAngle()
{
  struct Lifter currentLifterAngle;

  checkSum = 0;
  dataLength = 6;
  sendData.resize(dataLength);

  sendData[0] = 0xFD;
  sendData[1] = 0xDF;
  sendData[2] = 2;
  sendData[3] = 0x41;
  sendData[4] = 0x00;

  //CheckSum
  for(checkCount = 2;checkCount < dataLength-1;checkCount++)
    {
      checkSum += sendData[checkCount];
    }
  sendData[dataLength-1] = ~checkSum;


  sc.flushSerialPort();
  sc.serialAsyncWrite(sendData);

  sc.serialAsyncRead(receiveData,68);

  sc.serialAsyncRead(receiveData,68);

  if(receiveData[0] == 0xDF && receiveData[3] == 0x41)
    {
      //Get encoder value of each lifters
      //Lifter Top Hexadecimal angle data [deg]
      lifterTop_16 = (static_cast<int>(receiveData[5] << 8) | (static_cast<int>(receiveData[6])));
      //Lifter Under Hexadecimal angle data [deg]
      lifterUnder_16 = (static_cast<int>(receiveData[7] << 8) | (static_cast<int>(receiveData[8])));
    }

  currentLifterAngle.top = -0.000000000320705477365596 * std::pow((lifterTop_16/100),6) + 0.000000101518057737948 * std::pow((lifterTop_16/100),5) - 0.0000109692918783821 * std::pow((lifterTop_16/100),4) + 0.000649565749313297 * std::pow((lifterTop_16/100),3) - 0.0205117592049646 * std::pow((lifterTop_16/100),2) + 1.59004309387819 * (lifterTop_16/100) + 0.025539887312334;
  currentLifterAngle.bottom = -0.000000000320705477365596 * std::pow((lifterUnder_16/100),6) + 0.000000101518057737948 * std::pow((lifterUnder_16/100),5) - 0.0000109692918783821 * std::pow((lifterUnder_16/100),4) + 0.000649565749313297 * std::pow((lifterUnder_16/100),3) - 0.0205117592049646 * std::pow((lifterUnder_16/100),2) + 1.59004309387819 * (lifterUnder_16/100) + 0.025539887312334;

  return currentLifterAngle;
}


struct Pose SeedCommand::seedGetLifterPose()
{
  struct Pose currentLifterPose;

  checkSum = 0;
  dataLength = 6;
  sendData.resize(dataLength);

  sendData[0] = 0xFD;
  sendData[1] = 0xDF;
  sendData[2] = 2;
  sendData[3] = 0x41;
  sendData[4] = 0x00;

  //CheckSum
  for(checkCount = 2;checkCount < dataLength-1;checkCount++)
    {
      checkSum += sendData[checkCount];
    }
  sendData[dataLength-1] = ~checkSum;


  sc.flushSerialPort();
  sc.serialAsyncWrite(sendData);

  sc.serialAsyncRead(receiveData,68);

  if(receiveData[0] == 0xDF && receiveData[3] == 0x41)
    {
      //Get encoder value of each lifters
      //Lifter Top Hexadecimal angle data [deg]
      lifterTop_16 = (static_cast<int>(receiveData[5] << 8) | (static_cast<int>(receiveData[6])));
      //Lifter Under Hexadecimal angle data [deg]
      lifterUnder_16 = (static_cast<int>(receiveData[7] << 8) | (static_cast<int>(receiveData[8])));
    }

  lifterAngleTop = -0.000000000320705477365596 * pow((lifterTop_16/100),6) + 0.000000101518057737948 * pow((lifterTop_16/100),5) - 0.0000109692918783821 * pow((lifterTop_16/100),4) + 0.000649565749313297 * pow((lifterTop_16/100),3) - 0.0205117592049646 * pow((lifterTop_16/100),2) + 1.59004309387819 * (lifterTop_16/100) + 0.025539887312334;
  lifterAngleUnder = -0.000000000320705477365596 * pow((lifterUnder_16/100),6) + 0.000000101518057737948 * pow((lifterUnder_16/100),5) - 0.0000109692918783821 * pow((lifterUnder_16/100),4) + 0.000649565749313297 * pow((lifterUnder_16/100),3) - 0.0205117592049646 * pow((lifterUnder_16/100),2) + 1.59004309387819 * (lifterUnder_16/100) + 0.025539887312334;

  //Change Degree to Radian
  lifterAngleTop = lifterAngleTop * M_PI / 180;
  lifterAngleUnder = lifterAngleUnder * M_PI / 180;

  currentLifterPose.x = lifterLength * sin(lifterAngleTop) - lifterLength * sin(lifterAngleUnder);
  currentLifterPose.z = lifterLength * cos(lifterAngleTop) + lifterLength * cos(lifterAngleUnder);

  return currentLifterPose;
}

void SeedCommand::resetParam()
{
  angularVelocity = {0,0,0,0};
  currentEncoder = {0,0,0,0};
  oldEncoder = {0,0,0,0};
  diffEncoder = {0,0,0,0};
  countEncoder = {0,0,0,0};
  encoderData = {0,0,0,0};

  meca.resetParam();
}
