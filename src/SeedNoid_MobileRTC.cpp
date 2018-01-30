// -*- C++ -*-
/*!
 * @file  SeedNoid_MobileRTC.cpp
 * @brief SeedNoid Mobile component
 * @date $Date$
 *
 * $Id$
 */

#include "SeedNoid_MobileRTC.h"

// Module specification
// <rtc-template block="module_spec">
static const char* seednoid_mobilertc_spec[] =
  {
    "implementation_id", "SeedNoid_MobileRTC",
    "type_name",         "SeedNoid_MobileRTC",
    "description",       "SeedNoid Mobile component",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "SeedNoid",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.debug", "0",
    "conf.default.PortName", "/dev/ttyUSB0",
    "conf.default.WheelRadiusParam", "0.076",
    "conf.default.TreadParam", "0.39",
    "conf.default.WheelBaseParam", "0.453",
    "conf.default.LifterMoveTime", "3000",

    // Widget
    "conf.__widget__.debug", "text",
    "conf.__widget__.PortName", "text",
    "conf.__widget__.WheelRadiusParam", "text",
    "conf.__widget__.TreadParam", "text",
    "conf.__widget__.WheelBaseParam", "text",
    "conf.__widget__.LifterMoveTime", "text",
    // Constraints

    "conf.__type__.debug", "int",
    "conf.__type__.PortName", "string",
    "conf.__type__.WheelRadiusParam", "double",
    "conf.__type__.TreadParam", "double",
    "conf.__type__.WheelBaseParam", "double",
    "conf.__type__.LifterMoveTime", "int",

    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
SeedNoid_MobileRTC::SeedNoid_MobileRTC(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_targetVelocityIn("targetVelocity", m_targetVelocity),
    m_currentPoseOut("currentPose", m_currentPose),
    m_lifterPort("lifter")

    // </rtc-template>
{
}

/*!
 * @brief destructor
 */
SeedNoid_MobileRTC::~SeedNoid_MobileRTC()
{
}



RTC::ReturnCode_t SeedNoid_MobileRTC::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("targetVelocity", m_targetVelocityIn);
  
  // Set OutPort buffer
  addOutPort("currentPose", m_currentPoseOut);
  
  // Set service provider to Ports
  m_lifterPort.registerProvider("lifter", "RTC::LifterPoseInterface", m_lifter);
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  addPort(m_lifterPort);
  m_lifter.setRTC(this);
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("debug", m_debug, "0");
  bindParameter("PortName", m_PortName, "/dev/ttyUSB0");
  bindParameter("WheelRadiusParam", m_WheelRadiusParam, "0.076");
  bindParameter("TreadParam", m_TreadParam, "0.39");
  bindParameter("WheelBaseParam", m_WheelBaseParam, "0.453");
  bindParameter("LifterMoveTime", m_LifterMoveTime, "3000");
  // </rtc-template>
  
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t SeedNoid_MobileRTC::onActivated(RTC::UniqueId ec_id)
{
  //----------Set Seednoid Param----------
  SC.seedSetParam(m_WheelRadiusParam,m_TreadParam,m_WheelBaseParam);

  //----------Set Seednoid Param----------
  SC.resetParam();

  //----------Open Serial Port----------
  SC.seedSerialOpen(m_PortName,1000000);
 
  //----------Servo ON----------
  SC.seedSendServo(0,1,1);

  //----------Move Lifter----------
  LifterPose.x = 0.0;
  LifterPose.y = 0.0;
  LifterPose.z = 0.0;

  //LifterAngle.top = 45;
  //LifterAngle.bottom = 45;

  Time = m_LifterMoveTime;

  //SC.seedMoveLifterForwardKinematics(LifterAngle,Time);
  SC.seedMoveLifterReverseKinematics(LifterPose,Time);

  TruckVelocity = {0,0,0};
  OdometryData = {0,0,0,0};

  LifterAngle = {0,0};
  OldLifterAngle = {0,0};

  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedNoid_MobileRTC::onDeactivated(RTC::UniqueId ec_id)
{
  //----------Servo OFF----------
  SC.seedSendServo(0,0,1);

  //----------Close Serial Port----------
  SC.seedSerialClose();

  return RTC::RTC_OK;
}


RTC::ReturnCode_t SeedNoid_MobileRTC::onExecute(RTC::UniqueId ec_id)
{
  if(m_targetVelocityIn.isNew())
    {
      m_targetVelocityIn.read();
           
      TruckVelocity.vx = m_targetVelocity.data.vx;
      TruckVelocity.vy = m_targetVelocity.data.vy;
      TruckVelocity.va = m_targetVelocity.data.va;

      //----------Calc Kinematics----------
      SC.seedMecanumWheelKinematics(TruckVelocity);

      //----------Send Speed----------
      SC.seedSendTruckSpeed();

      //----------Get Encoder----------
      SC.seedGetWheelEncoder();

      //----------Check Odometry----------
      OdometryData = SC.seedCalcOdometry();

      if(m_debug)
	{
	  std::cout << "PositionX : " << OdometryData.x << " [m] " << std::endl;
	  std::cout << "PositionY : " << OdometryData.y << " [m] " << std::endl;
	  std::cout << "Direction : " << OdometryData.heading * 180 / M_PI << " [rad] " << std::endl;
	}

      m_currentPose.data.position.x = OdometryData.x;
      m_currentPose.data.position.y = OdometryData.y;
      m_currentPose.data.heading = OdometryData.heading;

      m_currentPoseOut.write();
    }

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t SeedNoid_MobileRTC::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void SeedNoid_MobileRTCInit(RTC::Manager* manager)
  {
    coil::Properties profile(seednoid_mobilertc_spec);
    manager->registerFactory(profile,
                             RTC::Create<SeedNoid_MobileRTC>,
                             RTC::Delete<SeedNoid_MobileRTC>);
  }
  
};


