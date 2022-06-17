#pragma once

#include "PointCloudCommon.h"

#include <string>
#include "PointCloud2DBase.h"
#include "PointXYZ.h"
#include "Array.h"

namespace ME
{  
  class Mesh;

  template <typename PointType>
  class PointCloud2D;
  template <typename PointType>
  using PointCloud2DPtr = std::shared_ptr<PointCloud2D<PointType>>;
  template <typename PointType>
  using PointCloud2DCPtr = std::shared_ptr<PointCloud2D<PointType> const>;
  
  /// @brief  A template virtual class that represent a PointCloud2D of specific type.
  /// This class provides read access to the PointCloud2D object data.
  ///
  /// @code
  ///   
  ///   bool hasInvalidPoints(ME::PointCloud2DBase const & point_cloud)
  ///   {
  ///       if (point_cloud.type() == ME::ME_POINT_XY_FLOAT)
  ///       {
  ///         using PointType = ME::PointXY<float>;
  ///         ME::PointCloud2D<PointType> const & src = point_cloud; // implicit conversion
  ///         for (PointType const & point : src.points())
  ///           if (!ME::PointCloud2D<PointType>::isValid(point))
  ///             return true;
  ///       }
  ///       else
  ///         . . .
  ///       return false;
  ///   }
  ///
  /// @endcode

  template <typename POINT_TYPE>
  class DLLExportMEPointCloud PointCloud2D : public PointCloud2DBase
  {
  public:
    
    using PointType = POINT_TYPE;  /// Point type. Same as the point type
    using NormalType = PointType; /// Normal vectors type. Same as the point type
    using IntensityType = unsigned short; /// Intensity values type.
    using MetaDataType = typename PointType::CoordType; /// Meta data type. Same as point coordinates type.

    using Points = Array<PointType>;  /// Points array type
    //using Normals = Array<NormalType>; /// Normals array type
    //using Intensities = Array<IntensityType>; /// Intensityes array type
    using MetaData = Array<MetaDataType>; /// Meta data array type
    
    explicit PointCloud2D(); 

    PointCloud2D(const PointCloud2D & other) = delete;
    virtual ~PointCloud2D();

    PointCloud2D& operator=(const PointCloud2D & other) = delete;
    
    static Point2DTypeID staticTypeID();
    virtual Point2DTypeID type() const;

    virtual int count()  const = 0;

    virtual bool hasMetadata(std::string const & id) const = 0;
    /// Return const reference to points array
    virtual const Points&      points()      const = 0;    
    /// Return const reference to metadata array with given id
    virtual const MetaData&    metaData(std::string const & id)    const = 0;
    
  
    /// Return vector in 3D that coresponds the X axis of the PointCloud2D
    virtual PointXYZ<typename PointType::CoordType> axisX() const = 0;
    /// Return vector in 3D that coresponds the Y axis of the PointCloud2D
    virtual PointXYZ<typename PointType::CoordType> axisY() const = 0;
    /// Return vector in 3D that coresponds the direction that is perpendicular to the PointCloud2D plane
    virtual PointXYZ<typename PointType::CoordType> planeNormal() const = 0;

    /// Return positopn of the PointCloud2D coordinate system 
    virtual PointXYZ<typename PointType::CoordType> offset() const = 0;
    /// Return X offset of the PointCloud2D coordinate system 
    virtual typename PointType::CoordType offsetX() const = 0;
    /// Return Y offset of the PointCloud2D coordinate system 
    virtual typename PointType::CoordType offsetY() const = 0;
    /// Return Z offset of the PointCloud2D coordinate system 
    virtual typename PointType::CoordType offsetZ() const = 0;
    

    static PointType invalidPoint();
    DEPRECATED("The function is deprecated. Would you please be so kind to use isValid function of PointType.")
      static bool isValid(const PointType & point);
  };

}

