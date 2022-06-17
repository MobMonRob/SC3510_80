#pragma once

#include "MEDecl.h"
#include <string>
#include <vector>
#include <functional>
#include <memory>
#include "MEGBase.h"
#include "MEGC.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class GenicamParameter;
  class GenicamParameterPrivate;
  class GenicamParameterBoolean;
  class GenicamParameterCommand;
  class GenicamParameterEnum;
  class GenicamParameterFloat;
  class GenicamParameterInt;
  class GenicamParameterUInt;
  class GenicamParameterString;

  using GenicamParameterPtr = std::shared_ptr<GenicamParameter>;
  using GenicamParameterVec = std::shared_ptr<std::vector<GenicamParameterPtr>>;
  using GenicamParameterBooleanPtr = std::shared_ptr<GenicamParameterBoolean>;
  using GenicamParameterCommandPtr = std::shared_ptr<GenicamParameterCommand>;
  using GenicamParameterEnumPtr = std::shared_ptr<GenicamParameterEnum>;
  using GenicamParameterFloatPtr = std::shared_ptr<GenicamParameterFloat>;
  using GenicamParameterIntPtr = std::shared_ptr<GenicamParameterInt>;
  using GenicamParameterUIntPtr = std::shared_ptr<GenicamParameterUInt>;
  using GenicamParameterStringPtr = std::shared_ptr<GenicamParameterString>;

  class MEGCPP_VISIBILITY GenicamParameter
  {
  public:
    virtual ~GenicamParameter();

    std::string name() const;
    std::string displayName() const;
    std::string categoryName() const;
    std::string description() const;
    std::string tooltip() const;

    meg_param_type_t type() const;

    meg_bool_t isSelector() const;
    meg_bool_t isSelected() const;
    GenicamParameterVec selectedParameters() const;
    GenicamParameterVec selectingParameters() const;

    meg_bool_t isReadable() const;
    meg_bool_t isWritable() const;
    meg_bool_t isCached() const;
    meg_bool_t isVisible(meg_param_visibility_t visibility) const;
    meg_param_visibility_t visibility() const;

    GenicamParameterBooleanPtr toBoolean() const;
    GenicamParameterCommandPtr toCommand() const;
    GenicamParameterEnumPtr toEnum() const;
    GenicamParameterFloatPtr toFloat() const;
    GenicamParameterIntPtr toInt() const;
    GenicamParameterUIntPtr toUInt() const;
    GenicamParameterStringPtr toString() const;

    void setParameterCallback(void(*callback)(void *), void *userdata = nullptr);
    void setParameterCallback(std::function<void()> callback);

  protected:
    GenicamParameter();
    GenicamParameter(const GenicamParameter &) = delete;
    GenicamParameter &operator=(const GenicamParameter &) = delete;

    GenicamParameterPrivate* const d_ptr;

    friend class GenicamDevice;
    friend class GenicamDevicePrivate;
  };
}

