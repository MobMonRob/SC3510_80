#pragma once

namespace ME
{
  /// Defines supported point cloud types.
  enum PointTypeID : int
  {
    ME_POINT_XYZ_FLOAT,
    ME_POINT_XYZ_DOUBLE,
    ME_POINT_XYZP_FLOAT,
    ME_POINT_XYZP_DOUBLE
  };

  /// The following definitions are required for automatic instantion of classes and function for supported types.
  /// For more information please refer the ME_POINT_CLOUD_INSTANTIATE macro.

#define ME_POINT_XYZ_FLOAT_TYPE ME::PointXYZ<float> 
#define ME_POINT_XYZ_DOUBLE_TYPE ME::PointXYZ<double> 
#define ME_POINT_XYZP_FLOAT_TYPE ME::PointXYZP<float> 
#define ME_POINT_XYZP_DOUBLE_TYPE ME::PointXYZP<double> 

#define ME_POINT_CLOUD_INSTANTIATE(macro) \
  macro(ME_POINT_XYZ_FLOAT_TYPE) \
  macro(ME_POINT_XYZ_DOUBLE_TYPE) \
  macro(ME_POINT_XYZP_FLOAT_TYPE) \
  macro(ME_POINT_XYZP_DOUBLE_TYPE)

#define ME_POINT_CLOUD_SELECTIVE_CALL(funcname, type_id, ...) \
  switch(type_id) \
  { \
  case ME::PointTypeID::ME_POINT_XYZ_FLOAT: \
    funcname<ME_POINT_XYZ_FLOAT_TYPE>(__VA_ARGS__); \
    break; \
  case ME::PointTypeID::ME_POINT_XYZ_DOUBLE: \
    funcname<ME_POINT_XYZ_DOUBLE_TYPE>(__VA_ARGS__); \
    break; \
  case ME::PointTypeID::ME_POINT_XYZP_FLOAT: \
    funcname<ME_POINT_XYZP_FLOAT_TYPE>(__VA_ARGS__); \
    break; \
  case ME::PointTypeID::ME_POINT_XYZP_DOUBLE: \
    funcname<ME_POINT_XYZP_DOUBLE_TYPE>(__VA_ARGS__); \
    break; \
  default: \
    break; \
  }


  /// Defines supported point cloud 2D types.
  enum Point2DTypeID : int
  {
    ME_POINT_XY_FLOAT,
    ME_POINT_XY_DOUBLE
  };

#define ME_POINT_XY_FLOAT_TYPE ME::PointXY<float> 
#define ME_POINT_XY_DOUBLE_TYPE ME::PointXY<double> 

#define ME_POINT_CLOUD_INSTANTIATE_2D(macro) \
  macro(ME_POINT_XY_FLOAT_TYPE) \
  macro(ME_POINT_XY_DOUBLE_TYPE)
  
#define ME_POINT_CLOUD_2D_SELECTIVE_CALL(funcname, type_id, ...) \
  switch(type_id) \
  { \
  case ME::Point2DTypeID::ME_POINT_XY_FLOAT: \
    funcname<ME_POINT_XY_FLOAT_TYPE>(__VA_ARGS__); \
    break; \
  case ME::Point2DTypeID::ME_POINT_XY_DOUBLE: \
    funcname<ME_POINT_XY_DOUBLE_TYPE>(__VA_ARGS__); \
    break; \
  default: \
    break; \
  }  
}
