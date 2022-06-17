#ifndef GENICAMPARAMETERFLOAT_H
#define GENICAMPARAMETERFLOAT_H

#include "MEDecl.h"
#include "GenicamParameter.h"

#ifndef MEGCPP_VISIBILITY
#define MEGCPP_VISIBILITY ME_DECL_IMPORT
#endif

namespace ME
{
  class MEGCPP_VISIBILITY GenicamParameterFloat : public GenicamParameter
  {
  public:
    ~GenicamParameterFloat();

    float minimum(megc_status_t* status = nullptr) const;
    float maximum(megc_status_t* status = nullptr) const;

    float value(megc_status_t* status = nullptr) const;
    megc_status_t setValue(float value);

  private:
    GenicamParameterFloat();

    friend class GenicamDevicePrivate;
    friend class GenicamParameter;
  };
}

#endif //GENICAMPARAMETERFLOAT_H
