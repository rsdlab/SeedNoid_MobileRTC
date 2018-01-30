// -*-C++-*-
/*!
 * @file  SeedNoid_MobileSVC_impl.cpp
 * @brief Service implementation code of SeedNoid_Mobile.idl
 *
 */

#include "SeedNoid_MobileSVC_impl.h"

//Include SeedNoid_MobileRTC
#include "SeedNoid_MobileRTC.h"

/*
 * Example implementational code for IDL interface RTC::LifterPoseInterface
 */
RTC_LifterPoseInterfaceSVC_impl::RTC_LifterPoseInterfaceSVC_impl()
{
  // Please add extra constructor code here.
}


RTC_LifterPoseInterfaceSVC_impl::~RTC_LifterPoseInterfaceSVC_impl()
{
  // Please add extra destructor code here.
}


/*
 * Methods corresponding to IDL attributes and operations
 */
RTC::LIFTER_STATE RTC_LifterPoseInterfaceSVC_impl::checkState()
{
//set return value
RTC::LIFTER_STATE result;

m_pRTC->CurrentLifterAngle = m_pRTC->currentLifterAngle();

if(m_pRTC->CurrentLifterAngle.top == m_pRTC->OldLifterAngle.top && m_pRTC->CurrentLifterAngle.bottom == m_pRTC->OldLifterAngle.bottom)
  {
std::cout << "Move Stopping" << std::endl;
result = RTC::LIFTER_STOPPED;
}
 else
   {
std::cout << "Moving" << std::endl;
result = RTC::LIFTER_MOVING;
}
  
m_pRTC->OldLifterAngle = m_pRTC->currentLifterAngle();

return result;
}

RTC::LIFTER_STATE RTC_LifterPoseInterfaceSVC_impl::setLifterTime(::CORBA::Double MoveTime)
{
  //set return value
  RTC::LIFTER_STATE result;

  m_pRTC->Time = static_cast<int>(MoveTime);

  result = RTC::LIFTER_SET_PARAM_OK;

  return result;
}

RTC::RETURN_VALUE RTC_LifterPoseInterfaceSVC_impl::sendLifterAngle(const RTC::LifterAngle& targetLifterAngle)
{
  //set return value
  RTC::RETURN_VALUE result;

  m_pRTC->LifterAngle.top = targetLifterAngle.top;
  m_pRTC->LifterAngle.bottom = targetLifterAngle.bottom;

  if(!m_pRTC->CheckAngle(m_pRTC->LifterAngle))
    {
      std::cout << "It is out of the movable range" << std::endl;

      result = RTC::RETURN_NOT_FOUND;
    }
  else
    {
      std::cout << "It is within movable range" << std::endl;
      m_pRTC->moveLifterForwardKinematics(m_pRTC->LifterAngle);

      result = RTC::RETURN_MOVE_OK;
    }

  return result;
}

RTC::RETURN_VALUE RTC_LifterPoseInterfaceSVC_impl::sendLifterPose(const RTC::Point3D& targetLifterPose)
{
  //set return value
  RTC::RETURN_VALUE result;

  m_pRTC->LifterPose.x = targetLifterPose.x;
  m_pRTC->LifterPose.z = targetLifterPose.z;

  if(!m_pRTC->CheckPose(m_pRTC->LifterPose))
    {
      std::cout << "Out of calculation range" << std::endl;

      result = RTC::RETURN_OUT_OF_RANGE;
    }
  else
    {
      std::cout << "Range of movement" << std::endl;
      m_pRTC->moveLifterReverseKinematics(m_pRTC->LifterPose);

      result = RTC::RETURN_MOVE_OK;
    }

  return result;
}

RTC::RETURN_VALUE RTC_LifterPoseInterfaceSVC_impl::getLifterAngle(RTC::LifterAngle& currentLifterAngle)
{
  //set return value
  RTC::RETURN_VALUE result = RTC::RETURN_OK;

  struct Lifter currentAngle;

  currentAngle = m_pRTC->currentLifterAngle();

  currentLifterAngle.top = currentAngle.top;
  currentLifterAngle.bottom = currentAngle.bottom;

  return result;
}

RTC::RETURN_VALUE RTC_LifterPoseInterfaceSVC_impl::getLifterPose(RTC::Point3D& currentLifterPose)
{
  //set return value
  RTC::RETURN_VALUE result = RTC::RETURN_OK;

  struct Pose currentPose;

  currentPose = m_pRTC->currentLifterPose();

  currentLifterPose.x = currentPose.x;
  currentLifterPose.z = currentPose.z;

  return result;
}



// End of example implementational code
