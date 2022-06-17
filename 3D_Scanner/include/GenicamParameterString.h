#pragma once

#include "MEDecl.h"
#include "GenicamParameter.h"
#include <string>

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterString : public GenicamParameter
  {
  public:
    ~GenicamParameterString();

    std::string value(megc_status_t* status = nullptr) const;
    megc_status_t setValue(const std::string& value);

  private:
    GenicamParameterString();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

