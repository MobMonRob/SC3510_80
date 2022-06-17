#pragma once

#include "MEDecl.h"
#include "GenicamParameter.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterUInt : public GenicamParameter
  {
  public:
    ~GenicamParameterUInt();

    unsigned int minimum(megc_status_t* status = nullptr) const;
    unsigned int maximum(megc_status_t* status = nullptr) const;
    unsigned int increment(megc_status_t* status = nullptr) const;

    unsigned int value(megc_status_t* status = nullptr) const;
    megc_status_t setValue(unsigned int value);

  private:
    GenicamParameterUInt();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

