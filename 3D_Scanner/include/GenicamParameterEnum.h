#pragma once

#include "MEDecl.h"
#include "GenicamParameter.h"
#include <string>
#include <vector>

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterEnum : public GenicamParameter
  {
  public:
    ~GenicamParameterEnum();

    std::vector<int> values(megc_status_t* status = nullptr) const;
    std::vector<std::string> entries(megc_status_t* status = nullptr) const;
    std::vector<std::string> displayEntries(megc_status_t* status = nullptr) const;

    int value(megc_status_t* status = nullptr) const;
    std::string entry(megc_status_t* status = nullptr) const;
    std::string displayEntry(megc_status_t* status = nullptr) const;

    megc_status_t setValue(int value);
    megc_status_t setEntry(const std::string& entry);
    megc_status_t setDisplayEntry(const std::string& entry);

  private:
    GenicamParameterEnum();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

