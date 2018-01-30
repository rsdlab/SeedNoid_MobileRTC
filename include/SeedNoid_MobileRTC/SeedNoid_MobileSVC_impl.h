// -*-C++-*-
/*!
 * @file  SeedNoid_MobileSVC_impl.h
 * @brief Service implementation header of SeedNoid_Mobile.idl
 *
 */

#include "BasicDataTypeSkel.h"
#include "ExtendedDataTypesSkel.h"

#include "SeedNoid_MobileSkel.h"

#ifndef SEEDNOID_MOBILESVC_IMPL_H
#define SEEDNOID_MOBILESVC_IMPL_H

class SeedNoid_MobileRTC; 
/*!
 * @class LifterPoseInterfaceSVC_impl
 * Example class implementing IDL interface RTC::LifterPoseInterface
 */
class RTC_LifterPoseInterfaceSVC_impl
  : public virtual POA_RTC::LifterPoseInterface,
    public virtual PortableServer::RefCountServantBase
{
private:
  // Make sure all instances are built on the heap by making the
  // destructor non-public
  //virtual ~LifterPoseInterfaceSVC_impl();
  SeedNoid_MobileRTC* m_pRTC;

public:
  /*!
   * @brief standard constructor
   */
  RTC_LifterPoseInterfaceSVC_impl();
  /*!
   * @brief destructor
   */
  virtual ~RTC_LifterPoseInterfaceSVC_impl();

  // attributes and operations
  RTC::LIFTER_STATE checkState();
  RTC::LIFTER_STATE setLifterTime(::CORBA::Double MoveTime);
  RTC::RETURN_VALUE sendLifterAngle(const RTC::LifterAngle& targetLifterAngle);
  RTC::RETURN_VALUE sendLifterPose(const RTC::Point3D& targetLifterPose);
  RTC::RETURN_VALUE getLifterAngle(RTC::LifterAngle& currentLifterAngle);
  RTC::RETURN_VALUE getLifterPose(RTC::Point3D& currentLifterPose);

  void setRTC(SeedNoid_MobileRTC* pRTC)
  {
    m_pRTC = pRTC;
  }

};

#endif // SEEDNOID_MOBILESVC_IMPL_H

