#pragma once

#include "PointCloudCommon.h"

#include <vector>

namespace ME
{
  /// @brief A virtual interface class for the container that is used in the point cloud class
  
  template <typename DataType>
  class DLLExportMEPointCloud Array
  {

  protected:
    Array() = default; //default constructor

  public:
    virtual ~Array() = default;

    Array(const Array & other) = delete;
    Array(const Array && other) = delete;
    Array& operator=(const Array & other) = delete;
    Array& operator=(const Array && other) = delete;

    /// Return width of the array (or amount of elements in one dimentional case)
    virtual int width() const = 0;
    /// Return height of the array (or 1 in one dimentional case)
    virtual int height() const = 0;
    /// Return amount of elements (width * height in general case)
    virtual int count() const = 0;
    /// Return capacity of the array
    /// The capacity is the number of elements that can be stored by array without allocation of new memory.
    virtual int capacity() const = 0;
    
    /// Return constant raw pointer to array data.
    virtual const DataType* data() const = 0;
    
    /// @brief Return const reference to array element with given index.
    /// @param idx Index of the value in the array
    /// @return Const reference to data value
    virtual const DataType& value(int idx) const = 0;    

    /// @brief Return const reference to array element with given coordinates.
    ///
    /// @note This function has sense only for structured arrays. The behavior with unstructured arrays 
    /// is not defined.
    ///
    /// @param u Index of the column in the structured array
    /// @param v Index of the raw in the structured array
    /// @return Const reference to data value
    virtual const DataType& value(int u, int v) const = 0;

    /// @brief Access operatoit that returns const reference to array element with given index.
    /// @param idx Index of the value in the array
    /// @return Const reference to data value
    virtual const DataType& operator[](int idx) const = 0;

    using iterator = DataType* ; /// Iterator type for the array
    using const_iterator = const DataType*; /// Const iterator type for the array


    /// Returns constant iterator pointing to the first element in the array
    virtual const_iterator begin() const = 0;
    /// Returns constant iterator pointing to the first element in the array
    virtual const_iterator cbegin() const = 0;
    /// Returns constant iterator pointing to the first element in the array
    virtual const_iterator constBegin() const = 0;
    /// Returns constant iterator pointing to the past-the-end element in the array
    virtual const_iterator end() const = 0;
    /// Returns constant iterator pointing to the past-the-end element in the array
    virtual const_iterator cend() const = 0;
    /// Returns constant iterator pointing to the past-the-end element in the array
    virtual const_iterator constEnd() const = 0;

  };

}