#pragma once

#if defined(__MINGW32__)
#include <_mingw.h>
#endif

#ifndef ME_DECL_DEPRECATED
  #if defined _MSC_VER
    #define ME_DECL_DEPRECATED(message) __declspec(deprecated(message))
  #elif defined __GNUC__
    #define ME_DECL_DEPRECATED(message) __attribute__((deprecated(message)))
  #else
	#define ME_DECL_DEPRECATED(message)
  #endif
#endif

#include "MEDecl.h"
#include "MEGBase.h"
#include "MEGC.h"
#include "GenicamParameter.h"
#include <string>
#include <vector>
#include <functional>
#include <memory>

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class GenicamDevicePrivate;
  class ReflectControlDevice;
  class SurfaceControlDevice;
  class ScanControlDevice;

  using ReflectControlDevicePtr = std::shared_ptr<ReflectControlDevice>;
  using SurfaceControlDevicePtr = std::shared_ptr<SurfaceControlDevice>;
  using ScanControlDevicePtr = std::shared_ptr<ScanControlDevice>;

  class MEGCPP_VISIBILITY GenicamDevice
  {
  public:
    enum Type
    {
      T_ReflectControl,
      T_SurfaceControl,
      T_ScanControl,
      T_Unknown = 1000
    };

    enum AcquisitionMode
    {
      AM_Continuous,
      AM_SingleFrame
    };

    enum EventWarningCode
    {
      EW_CriticalTemperature = 1
    };

    enum EventErrorCode
    {
      EE_SensorHardware = 1,
      EE_SensorAcquisition = 2,
      EE_3dReconstruction = 3,
      EE_3dResampling = 4,
      EE_3dFilter = 5,
      EE_SensorReferencing = 6,
      EE_ReferenceData = 7,
      EE_ApplySensorReferencing = 8,
      EE_CriticalTemperature = 9
    };

    virtual ~GenicamDevice();

    std::string id() const;
    std::string modelName() const;
    std::string vendorName() const;
    std::string serialNumber() const;
    std::string ipAddress() const;
    std::string macAddress() const;
    std::string subnetMask() const;
    std::string defaultGateway() const;

    meg_bool_t isDHCPAvailable() const;
    meg_bool_t isDHCPEnabled() const;
    meg_bool_t isLLAAvailable() const;
    meg_bool_t isLLAEnabled() const;
    meg_bool_t isPersistentAvailable() const;
    meg_bool_t isPersistentEnabled() const;

    Type type();
    ReflectControlDevicePtr toReflectControlDevice();
    SurfaceControlDevicePtr toSurfaceControlDevice();
    ScanControlDevicePtr toScanControlDevice();

    megc_status_t gevVersion(int* majorpt, int*minorpt) const;
    megc_access_type_t accessType();

    megc_status_t setIpConfiguration(const std::string& ip, const std::string& mask, const std::string& gateway);

    void setDisconnectCallback(void(*callback)(void *), void *userdata = nullptr);
    void setDisconnectCallback(std::function<void()> callback);
    void setImageCallback(void(*callback)(const std::vector<meg_image_t*>&, void *), void *userdata = nullptr);
    void setImageCallback(std::function<void(const std::vector<meg_image_t*>&)> callback);
    void setEventCallback(void(*callback)(GenicamParameterVec, meg_uint64_t, void *), void *userdata = nullptr);
    void setEventCallback(std::function<void(GenicamParameterVec, meg_uint64_t)> callback);

    virtual megc_status_t connect();
    megc_status_t disconnect();
    bool isConnected() const;

    GenicamParameterVec deviceParameters() const;
    GenicamParameterVec streamParameters() const;
    GenicamParameterVec communicationParameters() const;
    GenicamParameterPtr parameterByName(const std::string& name) const;

    megc_status_t acquisitionStart();
    megc_status_t acquisitionStart(AcquisitionMode mode, bool triggered = false);
    megc_status_t acquisitionStop();
    megc_status_t setAcquisitionMode(AcquisitionMode mode);
    megc_status_t setTriggerMode(bool triggered);
    megc_status_t triggerSoftware();

    meg_bool_t isAcquisitionActive();

    megc_status_t makeIpPersistent(const std::string& ip, const std::string& mask, const std::string& gateway, meg_bool_t use_persistent);
    megc_status_t dumpXml(const std::string& filename);

    megc_status_t loadUserSet(int idx);
    megc_status_t saveUserSet(int idx);

    //deprecated functions, will be removed in subsequent releases
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventCallback(void(*callback)(GenicamParameterVec, meg_uint64_t, void *), void *userdata = nullptr)\" instead.") void setEventCallback(void(*callback)(GenicamParameterVec, void *), void *userdata = nullptr);
    ME_DECL_DEPRECATED("This function is deprecated. Please use \"setEventCallback(std::function<void(GenicamParameterVec, meg_uint64_t)> callback)\" instead.") void setEventCallback(std::function<void(GenicamParameterVec)> callback);
    ///////////////////////

  protected:
    GenicamDevice();
    GenicamDevice(GenicamDevicePrivate& d);
    GenicamDevice(const GenicamDevice &) = delete;
    GenicamDevice &operator=(const GenicamDevice &) = delete;

    GenicamDevicePrivate* const d_ptr;

    friend class GenicamSystem;
    friend class GenicamSystemPrivate;
    friend class GenicamInterface;
  };

  using GenicamDevicePtr = std::shared_ptr<GenicamDevice>;
  using GenicamDeviceVec = std::shared_ptr<std::vector<GenicamDevicePtr>>;
}
