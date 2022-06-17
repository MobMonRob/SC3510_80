#pragma once

#include "PointCloudCommon.h"


#include "PointCloudBase.h"
#include "Array.h"

namespace ME
{

  template <typename PointType>
  class PointCloud;
  template <typename PointType>
  using PointCloudPtr = std::shared_ptr<PointCloud<PointType>>;
  template <typename PointType>
  using PointCloudCPtr = std::shared_ptr<PointCloud<PointType> const>;

  /// @brief  A template virtual class that represent a pointcloud of specific type.
  /// This class provides read access to the point cloud object data.
  ///
  /// @code
  ///   
  ///   bool hasInvalidPoints(ME::PointCloudBase const & point_cloud)
  ///   {
  ///       if (point_cloud.type() == ME::ME_POINT_XYZ_FLOAT)
  ///       {
  ///         using PointType = ME::PointXYZ<float>;
  ///         ME::PointCloud<PointType> const & src = point_cloud; // implicit conversion
  ///         for (PointType const & point : src.points())
  ///           if (!point.isValid())
  ///             return true;
  ///       }
  ///       else
  ///         . . .
  ///       return false;
  ///   }
  ///
  /// @endcode

  template <typename POINT_TYPE>
  class DLLExportMEPointCloud PointCloud : public PointCloudBase
  {
  public:
    using PointType = POINT_TYPE; /// Points type. Same as the point type
    using NormalType = PointType; /// Normal vectors type. Same as the point type
    using IntensityType = unsigned short; /// Intensity values type.
    using MetaDataType = typename PointType::CoordType; /// Meta data type. Same as point coordinates type.

    using Points = Array<PointType>;  /// Points array type
    using Normals = Array<NormalType>; /// Normals array type
    using Intensities = Array<IntensityType>; /// Intensityes array type
    using MetaData = Array<MetaDataType>; /// Meta data array type

    explicit PointCloud();

    PointCloud(const PointCloud & other) = delete;
    virtual ~PointCloud();

    PointCloud& operator=(const PointCloud & other) = delete;

    static PointTypeID staticTypeID();
    virtual PointTypeID type() const;

    virtual int width()  const = 0;
    virtual int height() const = 0;
    virtual int count()  const = 0;

    virtual bool hasNormals() const = 0;
    virtual bool hasMesh() const = 0;
    virtual bool hasIntensities() const = 0;
    virtual bool hasMetadata() const = 0;

    /// Return const reference to points array
    virtual const Points&      points()      const = 0;
    /// Return const reference to normals array
    virtual const Normals&     normals()     const = 0;
    /// Return const reference to intensities array
    virtual const Intensities& intensities() const = 0;
    /// Return const reference to metadata array
    virtual const MetaData&    metaData()    const = 0;

    /// Return true if the point cloud has regular grid structure
    virtual bool grid() const = 0;
    /// Return true if the regular grid has default position in space
    virtual bool gridHasDefaultPosition() const = 0;
    /// Return true if the regular grid has default orientation in space
    virtual bool gridHasDefaultOrientation() const = 0;
    /// Return vector in 3D that coresponds the X axis of the regular grid structure
    virtual PointType gridX() const = 0;
    /// Return vector in 3D that coresponds the Y axis of the regular grid structure
    virtual PointType gridY() const = 0;
    /// Return vector in 3D that coresponds the direction that is perpendicular to the regular grid structure (depth direction) 
    virtual PointType gridZ() const = 0;
    /// Return offset of the (0,0) point in the regular grid structure
    virtual PointType offset() const = 0;
    /// Return X offset of the (0,0) point in the regular grid structure
    virtual typename PointType::CoordType offsetX() const = 0;
    /// Return Y offset of the (0,0) point in the regular grid structure
    virtual typename PointType::CoordType offsetY() const = 0;
    /// Return Z offset of the (0,0) point in the regular grid structure
    virtual typename PointType::CoordType offsetZ() const = 0;
    /// Return scaling (step) in X direction for points in the regular grid structure
    virtual typename PointType::CoordType gridScalingX() const = 0;
    /// Return scaling (step) in Y direction for points in the regular grid structure
    virtual typename PointType::CoordType gridScalingY() const = 0;

    static PointType invalidPoint();

    DEPRECATED("The function is deprecated. Would you please be so kind to use isValid function of PointType.")
      static bool isValid(const PointType & point);
  };

}

