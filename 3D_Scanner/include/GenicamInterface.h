#pragma once

#include "MEDecl.h"
#include <vector>
#include "GenicamDevice.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class GenicamInterfacePrivate;

  class MEGCPP_VISIBILITY GenicamInterface
  {
  public:
    ~GenicamInterface();

    std::vector<std::string> ipAddresses() const;
    std::vector<std::string> subnetMasks() const;
    std::string macAddress() const;
    std::string defaultGateway() const;
    std::string description() const;

    GenicamDeviceVec devices() const;

  private:
    GenicamInterface();
    GenicamInterface(const GenicamInterface &) = delete;
    GenicamInterface &operator=(const GenicamInterface &) = delete;
 
    GenicamInterfacePrivate* const d_ptr;

    friend class GenicamSystem;
  };

  using GenicamInterfacePtr = std::shared_ptr<GenicamInterface>;
  using GenicamInterfaceVec = std::shared_ptr<std::vector<GenicamInterfacePtr>>;
}

