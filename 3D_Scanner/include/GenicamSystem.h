#pragma once

#if defined(__MINGW32__)
#include <_mingw.h>
#endif

#include "MEDecl.h"
#include "MEGBase.h"
#include "GenicamInterface.h"
#include "GenicamDevice.h"
#include <vector>

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class GenicamSystemPrivate;

  class MEGCPP_VISIBILITY GenicamSystem 
  {
  public:
    static megc_status_t initialize();
    static megc_status_t destroy();
    static meg_bool_t isInitialized();

    static std::string version();

    static GenicamInterfaceVec interfaces();

    static megc_status_t findDevices();
    static GenicamDeviceVec devices();
    static GenicamDevicePtr deviceByPattern(const std::string& pattern);
    static GenicamDevicePtr deviceByIp(const std::string& ip);
    static GenicamDevicePtr deviceBySerialNumber(const std::string& sn);

    static void setLogFilename(const std::string& filename);
    static void setLogLevel(meg_loglevel_t level);
    static void setLogCategory(meg_logcategory_t category);
    static void setLogActive(bool active = true);

  private:
    static GenicamSystemPrivate* d_ptr;

    friend class GenicamDevice;
    friend class ReflectControlDevice;
    friend class SurfaceControlDevice;
    friend class ScanControlDevice;
    friend class GenicamInterface;
  };
}

