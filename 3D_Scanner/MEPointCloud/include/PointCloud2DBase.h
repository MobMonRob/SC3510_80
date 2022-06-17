#pragma once

#include <memory>
#include <string>

#include "PointCloudCommon.h"

#include <vector>

#include "MEPointCloudTypes.h"

namespace ME
{  

  template <typename PointType>
  class PointCloud2D;

  class PointCloud2DBase;
  using PointCloud2DBasePtr = std::shared_ptr<PointCloud2DBase>;
  using PointCloud2DBaseCPtr = std::shared_ptr<PointCloud2DBase const>;
  
  using PointCloudIndices = std::vector<int>;
  using PointCloudIndicesPtr = std::shared_ptr<PointCloudIndices>;
  using PointCloudIndicesCPtr = std::shared_ptr<PointCloudIndices const>;

  using PointCloudDistances = std::vector<double>;
  using PointCloudDistancesPtr = std::shared_ptr<PointCloudDistances>;
  using PointCloudDistancesCPtr = std::shared_ptr<PointCloudDistances const>;


  /// @brief Base virtual non-template class for the PointCloud2D. 
  /// This class defines base non-typed interface for PointCloud2D object.
  /// This class implements static factory methods for creation of the PointCloud2D
  ///
  /// @code
  ///   
  /// void doSomthingWithPointCloud2D(ME::PointCloud2DBase const & point_PointCloud2D, ME::PointCloud2DBase & result_PointCloud2D);
  ///
  /// ME::PointCloud2DBasePtr process_point_PointCloud2D(ME::PointCloud2DBaseCPtr const & point_PointCloud2D_ptr)
  /// {
  ///   ME::PointCloud2DBasePtr result_ptr = ME::PointCloud2DBase::create(point_PointCloud2D_ptr->type());
  ///
  ///   doSomthingWithPointCloud2D(*point_PointCloud2D_ptr, *result_ptr);
  ///
  ///   return result_ptr;
  /// }
  ///
  /// @endcode

  class DLLExportMEPointCloud PointCloud2DBase
  {
  public:
    PointCloud2DBase();
    virtual ~PointCloud2DBase();
    
    static PointCloud2DBasePtr create(Point2DTypeID type_id);

    template <typename PointType>
    static PointCloud2DBasePtr create();
        
    /// Return type id of the point type in the PointCloud2D. Please refer @ref PointTypeID enum
    virtual Point2DTypeID type() const = 0;
    
    /// Return amount of points in the PointCloud2D.     
    virtual int count() const = 0;

    ///// Return true if the PointCloud2D has metadata with given id (intencities, distances, etz.)    
    virtual bool hasMetadata(std::string const & id) const = 0;

    /// @brief Implicit converter to pointer on typed representation of the PointCloud2D
    /// @note it is allowed to use it only after check of the type id.
    template <typename PointType>
    operator PointCloud2D<PointType>* () 
    {
      return dynamic_cast<PointCloud2D<PointType>*>(this);
    }

    /// Implicit converter to typed representation of the PointCloud2D
    /// @note It is allowed to use it only after check of the type id.
    template <typename PointType>
    operator PointCloud2D<PointType>& () 
    {
      return dynamic_cast<PointCloud2D<PointType>&>(*this);
    }

    /// Implicit converter to const typed representation of the PointCloud2D
    /// @note It is allowed to use it only after check of the type id.
    template <typename PointType>
    operator const PointCloud2D<PointType>& () const 
    {
      return dynamic_cast<const PointCloud2D<PointType>&>(*this);
    }

  };  
}

