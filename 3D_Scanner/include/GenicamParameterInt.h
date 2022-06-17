#pragma once

#include "MEDecl.h"
#include "GenicamParameter.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterInt : public GenicamParameter
  {
  public:
    ~GenicamParameterInt();

    int minimum(megc_status_t* status = nullptr) const;
    int maximum(megc_status_t* status = nullptr) const;
    unsigned int increment(megc_status_t* status = nullptr) const;

    int value(megc_status_t* status = nullptr) const;
    megc_status_t setValue(int value);

  private:
    GenicamParameterInt();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

