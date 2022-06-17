#pragma once

#include "MEDecl.h"
#include "GenicamParameter.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterBoolean : public GenicamParameter
  {
  public:
    ~GenicamParameterBoolean();

    int value(megc_status_t* status = nullptr) const;
    megc_status_t setValue(int value);

  private:
    GenicamParameterBoolean();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

