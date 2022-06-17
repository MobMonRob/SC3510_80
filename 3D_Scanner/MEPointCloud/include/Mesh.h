#pragma once

#include "PointCloudCommon.h"

#include <vector>
#include <memory>

namespace ME
{
  class Mesh;
  using MeshPtr = std::shared_ptr<Mesh>;
  using MeshCPtr = std::shared_ptr<Mesh const>;
  
  /// @brief A virtual interface class for the mesh data that is
  /// used in the point cloud class

  class DLLExportMEPointCloud Mesh
  {
  public:

#ifdef _MSC_VER
#pragma warning( disable : 4201)
#else
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif

    /// @brief This structure describe a triangle in a mesh.
    /// A triangle connects any three neighbour points in the point cloud.
    struct DLLExportMEPointCloud Triangle
    {
      Triangle();
      Triangle(int _a, int _b, int _c);
      union
      {
        int indices[3];
        struct
        {
          /// Index of the first point in the triangle.
          int a;
          /// Index of the second point in the triangle.
          int b;
          /// Index of the third point in the triangle.
          int c;
        };
      };
    };

    /// @brief This sctructure describe an edge in mesh strucructure. 
    /// An edge connects any two neighbour points in the point cloud.
    struct DLLExportMEPointCloud Edge
    {
      Edge();
      Edge(int _a, int _b);
      union
      {
        int indices[2];
        struct
        {
          /// Index of the first point in the edge
          int a;
          /// Index of the second point in the edge
          int b;
        };
      };
    };

#ifdef _MSC_VER 
#pragma warning( default : 4201)
#else
#pragma GCC diagnostic pop
#endif



  protected:
    Mesh() = default;
  public:
    Mesh(const Mesh& other) = delete;
    Mesh(const Mesh&& other) = delete;
    virtual ~Mesh() = default;

    Mesh& operator=(const Mesh& other) = delete;
    Mesh& operator=(const Mesh&& other) = delete;

    /// Return number of triangles in the mesh
    virtual int numOfTriangles() const = 0;
    /// Return vector of triangles
    virtual const std::vector<Triangle>& triangles() const = 0;
    
    /// Return number of indices that define triangles (num of triangles * 3)
    virtual int numOfTriangleIndices() const = 0;
    /// Return raw pointer to triangles indices array 
    virtual const int * triangleIndices() const = 0;

    /// Return number of edges in the mesh
    virtual int numOfEdges() const = 0;
    /// Return vector of edges
    virtual const std::vector<Edge>& edges() const = 0;

    /// Return number of indices that define edges (num of edges * 2)
    virtual int numOfEdgeIndices() const = 0;
    /// Return raw pointer to edges indices array 
    virtual const int * edgeIndices() const = 0;

    /// Return edges to triangles hash
    virtual const std::vector<std::vector<int>> & edgesToTrianglesHash() const = 0;

  };
}

