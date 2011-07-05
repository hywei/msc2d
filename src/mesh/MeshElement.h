#ifndef MESHLIB_MESHELEMENT_H_
#define MESHLIB_MESHELEMENT_H_

#include "../common/types.h"
#include <vector>

namespace meshlib{
  typedef int Handle;
    
  typedef Handle VertHandle;
  typedef Handle FaceHandle;
  typedef Handle EdgeHandle;
  typedef Handle HalfEdgeHandle;

  typedef std::vector<VertHandle> VertHandleArray;
  typedef std::vector<FaceHandle> FaceHandleArray;
  typedef std::vector<EdgeHandle> EdgeHandleArray;
  typedef std::vector<HalfEdgeHandle> HalfEdgeHandleArray;
    
  enum VERTFLAG{
    ISOLATED_VERT = 0x00000100, // isolated vertex flag
    NONMANIFOLD_VERT = 0x00000200, // manifold vertex flag
    BOUNDARY_VERT = 0x00000400  // boundary vertex flag
  };
  enum FACEFLAG{
    BOUNDARY_FACE = 0x00000100,
    NONMANIFOLD_FACE = 0x00000200
  };
  enum EDGEFLAG{
    BOUNDARY_EDGE = 0x00000100,
    NONMANIFOLD_EDGE = 0x00000200
  };
  enum MESHFLAG{
    TRIMESH = 0x00000100,
    QUADMESH = 0x00000200,
    POLYMESH = 0x00000400,
    MANIFOLD = 0x00000800    
  };
    
  class Vert
  {        
 public:
    Coord3D coord;
    Normal normal;        
    VERTFLAG flag; // flag bits
    HalfEdgeHandle he_handle;        
  };

  class Face
  {
 public:
    VertHandleArray vert_handle_vec;
    EdgeHandleArray edge_handle_vec;
    Normal normal;
    FACEFLAG flag;
  };

  class Edge
  {
 public:
 Edge(VertHandle h1, VertHandle h2):
    vert_handle_1(h1), vert_handle_2(h2){}

 public:
    HalfEdgeHandle he_handle_1;
    HalfEdgeHandle he_handle_2;

    VertHandle vert_handle_1;
    VertHandle vert_handle_2;
    EDGEFLAG flag;

    bool operator == (const Edge& _e){
      return _e.vert_handle_1 == vert_handle_1 &&
      _e.vert_handle_2 == vert_handle_2 ||
      _e.vert_handle_1 == vert_handle_2 &&
      _e.vert_handle_2 == vert_handle_1;
    }
  };

  class HalfEdge
  {
 public:
    HalfEdge();
    ~HalfEdge(){}
        
 public:
    HalfEdgeHandle prev_he_handle; /// previous halfedge
    HalfEdgeHandle next_he_handle; /// next halfedge
    HalfEdgeHandle oppo_he_handle; /// opposite halfedge

    VertHandle vert_handle; /// vertex of halfedge
    FaceHandle face_handle; /// face of halfedge, -1 if boundary halfedge
    EdgeHandle edge_handle; /// edge of halfedge
  };
    
  inline HalfEdge::HalfEdge():
      prev_he_handle(-1), next_he_handle(-1), oppo_he_handle(-1),
      vert_handle(-1), face_handle(-1), edge_handle(-1){}

  typedef std::vector<Vert> VertArray;
  typedef std::vector<Face> FaceArray;
  typedef std::vector<Edge> EdgeArray;
  typedef std::vector<HalfEdge> HalfEdgeArray;


  struct BoundingBox{
    Coord3D box_min, box_max, box_dim;
  };
  struct BoundingSphere{
    Coord3D center;
    double radius;
  };

  //! each boundary is a closed vertex handle loop 
  typedef VertHandleArray Boundary;
  typedef std::vector<Boundary> BoundaryArray;
}

#endif
