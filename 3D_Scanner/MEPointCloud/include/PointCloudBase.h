#pragma once

#include <memory>

#include "PointCloudCommon.h"

#include <vector>

#include "MEPointCloudTypes.h"

namespace ME
{
  class Mesh;

  template <typename PointType>
  class PointCloud;

  class PointCloudBase;
  using PointCloudBasePtr = std::shared_ptr<PointCloudBase>;
  using PointCloudBaseCPtr = std::shared_ptr<PointCloudBase const>;

  using PointCloudIndices = std::vector<int>;
  using PointCloudIndicesPtr = std::shared_ptr<PointCloudIndices>;
  using PointCloudIndicesCPtr = std::shared_ptr<PointCloudIndices const>;

  using PointCloudDistances = std::vector<double>;
  using PointCloudDistancesPtr = std::shared_ptr<PointCloudDistances>;
  using PointCloudDistancesCPtr = std::shared_ptr<PointCloudDistances const>;


  /// @brief Base virtual non-template class for the point cloud. 
  /// This class defines base non-typed interface for point cloud object.
  /// This class implements static factory methods for creation oif object cloud
  ///
  /// @code
  ///   
  /// void doSomthingNonStructured(ME::PointCloudBase const & point_cloud, ME::PointCloudBase & result_cloud);
  ///   
  /// void doSomthingStructured(ME::PointCloudBase const & point_cloud, ME::PointCloudBase & result_cloud);
  ///
  /// ME::PointCloudBasePtr process_point_cloud(ME::PointCloudBaseCPtr const & point_cloud_ptr)
  /// {
  ///   ME::PointCloudBasePtr result_ptr = ME::PointCloudBase::create(point_cloud_ptr->type());
  ///
  ///   if (point_cloud_ptr->height > 1) //structured point cloud
  ///      doSomthingStructured(*point_cloud_ptr, *result_ptr);
  ///   else
  ///      doSomthingNonStructured(*point_cloud_ptr, *result_ptr);
  ///
  ///   return result_ptr;
  /// }
  ///
  /// @endcode

  class DLLExportMEPointCloud PointCloudBase
  {
  public:
    PointCloudBase();
    virtual ~PointCloudBase();

    static PointCloudBasePtr create(const int type_id);

    template <typename PointType>
    static PointCloudBasePtr create();

    /// Return type id of the point type in the point cloud. Please refer @ref PointTypeID enum
    virtual PointTypeID type() const = 0;

    /// Return width of the point cloud. 
    virtual int width() const = 0;
    /// Return height of the point cloud.
    virtual int height() const = 0;

    /// Return amount of points in the point cloud. 
    /// @note The amount of point is equal to width for non structure point clouds. It is equal to the product of width and height for structured point cloud.
    virtual int count() const = 0;

    /// Return true if the point cloud has normals vectors array
    virtual bool hasNormals() const = 0;
    /// Return true if the point cloud has mesh information
    virtual bool hasMesh() const = 0;
    /// Return true if the point cloud has intensities array
    virtual bool hasIntensities() const = 0;
    /// Return true if the point cloud has metadata array
    virtual bool hasMetadata() const = 0;

    /// Return const reference to mesh
    virtual const Mesh& mesh() const = 0;

    /// Search at least one invalid point and return true if such point is found
    virtual bool checkHasInvalidPoint() const = 0;
    /// Compute the number of invalid points in the point cloud
    virtual int computeNumberOfInvalidPoints() const = 0;

    /// @brief Implicit converter to pointer on typed representation of the point cloud
    /// @note it is allowed to use it only after check of the type id.
    template <typename PointType>
    operator PointCloud<PointType>* ()
    {
      return dynamic_cast<PointCloud<PointType>*>(this);
    }

    /// Implicit converter to typed representation of the point cloud
    /// @note It is allowed to use it only after check of the type id.
    template <typename PointType>
    operator PointCloud<PointType>& ()
    {
      return dynamic_cast<PointCloud<PointType>&>(*this);
    }

    /// Implicit converter to const typed representation of the point cloud
    /// @note It is allowed to use it only after check of the type id.
    template <typename PointType>
    operator const PointCloud<PointType>& () const
    {
      return dynamic_cast<const PointCloud<PointType>&>(*this);
    }

  };
}

