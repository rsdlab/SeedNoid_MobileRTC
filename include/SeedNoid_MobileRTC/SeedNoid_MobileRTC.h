// -*- C++ -*-
/*!
 * @file  SeedNoid_MobileRTC.h
 * @brief SeedNoid Mobile component
 * @date  $Date$
 *
 * $Id$
 */

#ifndef SEEDNOID_MOBILERTC_H
#define SEEDNOID_MOBILERTC_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">
#include "SeedNoid_MobileSVC_impl.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ExtendedDataTypesStub.h"

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="port_stub_h">
// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
//Include SeedCommand
#include "SeedCommand.h"

using namespace RTC;

/*!
 * @class SeedNoid_MobileRTC
 * @brief SeedNoid Mobile component
 *
 */
class SeedNoid_MobileRTC
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  SeedNoid_MobileRTC(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~SeedNoid_MobileRTC();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
  /*!
   * 
   * - Name:  debug
   * - DefaultValue: 0
   */
  int m_debug;
  /*!
   * 
   * - Name:  PortName
   * - DefaultValue: /dev/ttyUSB0
   */
  std::string m_PortName;
  /*!
   * 
   * - Name:  WheelRadiusParam
   * - DefaultValue: 0.076
   */
  double m_WheelRadiusParam;
  /*!
   * 
   * - Name:  TreadParam
   * - DefaultValue: 0.39
   */
  double m_TreadParam;
  /*!
   * 
   * - Name:  WheelBaseParam
   * - DefaultValue: 0.453
   */
  double m_WheelBaseParam;
  /*!
   * 
   * - Name:  LifterMoveTime
   * - DefaultValue: 3000
   */
  int m_LifterMoveTime;

  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedVelocity2D m_targetVelocity;
  /*!
   */
  InPort<RTC::TimedVelocity2D> m_targetVelocityIn;
  
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedPose2D m_currentPose;
  /*!
   */
  OutPort<RTC::TimedPose2D> m_currentPoseOut;
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  /*!
   */
  RTC::CorbaPort m_lifterPort;
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  /*!
   */
  RTC_LifterPoseInterfaceSVC_impl m_lifter;
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

  SeedCommand SC;

  struct Velocity TruckVelocity;
  struct Pose OdometryData;

public:
  int Time;
  struct Pose LifterPose;
  struct Lifter LifterAngle;
  struct Lifter CurrentLifterAngle;
  struct Lifter OldLifterAngle;

  bool CheckAngle(struct Lifter lifterAngle)
  {
    return SC.CheckTargetAngle(lifterAngle);
  }

  bool CheckPose(struct Pose lifterPose)
  {
    return SC.CheckTargetPose(lifterPose);
  }

  void moveLifterForwardKinematics(struct Lifter lifterAngle)
  {
    SC.seedMoveLifterForwardKinematics(lifterAngle,Time);
  }

  void moveLifterReverseKinematics(struct Pose lifterPose)
  {
    SC.seedMoveLifterReverseKinematics(lifterPose,Time);
  }

  struct Lifter currentLifterAngle()
  {
    struct Lifter currentAngle;
    currentAngle = SC.seedGetLifterAngle();
    return currentAngle;
  }

  struct Pose currentLifterPose()
  {
    struct Pose currentPose;
    currentPose = SC.seedGetLifterPose();
    return currentPose;
  }

};


extern "C"
{
  DLL_EXPORT void SeedNoid_MobileRTCInit(RTC::Manager* manager);
};

#endif // SEEDNOID_MOBILERTC_H
