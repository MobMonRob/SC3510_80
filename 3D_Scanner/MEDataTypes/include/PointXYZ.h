#pragma once

#include "MEDataTypesCommon.h"

#include <type_traits>

#ifdef _MSC_VER
  #pragma warning( disable : 4201)
#else
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wpedantic"
#endif

namespace ME
{

  template <typename COORD_TYPE>
  struct DLLExportMEDataTypes PointXYZ
  {
    typedef COORD_TYPE CoordType;
    union
    {
      CoordType data[3];
      struct {
        CoordType x;
        CoordType y;
        CoordType z;
      };
    };


    /// @brief Standart constructor.
    /// @param x X coordinate of the point
    /// @param y Y coordinate of the point
    /// @param z Z coordinate of the point
 
    PointXYZ();
    PointXYZ(const COORD_TYPE& x, const COORD_TYPE& y, const COORD_TYPE& z);

    template<typename OTHER_COORD_TYPE>
    PointXYZ(const OTHER_COORD_TYPE & _x,
      const OTHER_COORD_TYPE & _y,
      const OTHER_COORD_TYPE & _z);

    PointXYZ operator+(const PointXYZ & other) const;
    PointXYZ operator-(const PointXYZ & other) const;
    PointXYZ operator-() const;
    PointXYZ & operator+=(const PointXYZ & other);
    PointXYZ & operator-=(const PointXYZ & other);

    PointXYZ operator * (const CoordType & scalar) const;

    void setNull();

    bool operator ==(const PointXYZ & other) const;
    bool operator !=(const PointXYZ & other) const;

    bool isNull() const;
    bool isNullLength() const;
    bool isValid() const;

    //vector operations
    CoordType length() const;
    PointXYZ normalized() const;
    void normalize();

    CoordType dot(const PointXYZ & other) const;
    PointXYZ cross(const PointXYZ & other) const;
  };

  template <typename COORD_TYPE>
  struct DLLExportMEDataTypes PointXYZP
  {
    typedef COORD_TYPE CoordType;
    union
    {
      CoordType data[4];
      struct {
        CoordType x;
        CoordType y;
        CoordType z;
        CoordType p;
      };
    };


    /// @brief Standart constructor.
    PointXYZP();


    /// @brief Standart constructor.
    /// @param x X coordinate of the point
    /// @param y Y coordinate of the point
    /// @param z Z coordinate of the point
    /// @param p Value for padding element
    PointXYZP(const COORD_TYPE & x,
      const COORD_TYPE & y,
      const COORD_TYPE & z,
      const COORD_TYPE & p = static_cast<COORD_TYPE>(1.0));


    template<typename OTHER_COORD_TYPE>
    PointXYZP(const OTHER_COORD_TYPE & _x,
      const OTHER_COORD_TYPE & _y,
      const OTHER_COORD_TYPE & _z,
      const OTHER_COORD_TYPE & _p = static_cast<OTHER_COORD_TYPE>(1.0));


    PointXYZP operator+(const PointXYZP & other) const;
    PointXYZP operator-(const PointXYZP & other) const;
    PointXYZP operator-() const;
    PointXYZP & operator+=(const PointXYZP & other);
    PointXYZP & operator-=(const PointXYZP & other);

    PointXYZP operator * (const CoordType & scalar) const;

    void setNull();

    bool operator ==(const PointXYZP & other) const;
    bool operator !=(const PointXYZP & other) const;

    bool isNull() const;
    bool isNullLength() const;
    bool isValid() const;

    //vector operations
    CoordType length() const;
    PointXYZP normalized() const;
    void normalize();

    CoordType dot(const PointXYZP & other) const;
    PointXYZP cross(const PointXYZP & other) const;
  };


  template<typename COORD_TYPE>
  template<typename OTHER_COORD_TYPE>
  inline PointXYZ<COORD_TYPE>::PointXYZ(const OTHER_COORD_TYPE & _x,
    const OTHER_COORD_TYPE & _y,
    const OTHER_COORD_TYPE & _z)
    : x(static_cast<COORD_TYPE>(_x))
    , y(static_cast<COORD_TYPE>(_y))
    , z(static_cast<COORD_TYPE>(_z)) {}


  template<typename COORD_TYPE>
  template<typename OTHER_COORD_TYPE>
  inline PointXYZP<COORD_TYPE>::PointXYZP(const OTHER_COORD_TYPE & _x,
    const OTHER_COORD_TYPE & _y,
    const OTHER_COORD_TYPE & _z,
    const OTHER_COORD_TYPE & _p)
    : x(static_cast<COORD_TYPE>(_x))
    , y(static_cast<COORD_TYPE>(_y))
    , z(static_cast<COORD_TYPE>(_z))
    , p(static_cast<COORD_TYPE>(_p)) {};


}



#ifdef _MSC_VER 
#pragma warning( default : 4201)
#else
#pragma GCC diagnostic push
#endif
