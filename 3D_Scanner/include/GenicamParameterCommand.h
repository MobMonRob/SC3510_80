#pragma once

#include "MEDecl.h"
#include "GenicamParameter.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterCommand : public GenicamParameter
  {
  public:
    ~GenicamParameterCommand();

    megc_status_t sendCommand();

  private:
    GenicamParameterCommand();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

