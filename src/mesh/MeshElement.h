#ifndef MESHLIB_MESHELEMENT_H_
#define MESHLIB_MESHELEMENT_H_

#include "../common/types.h"
#include <string>

namespace MeshLib{

    typedef ptrdiff_t Handle;
    
    typedef Handle VertHandle;
    typedef Handle FaceHandle;
    typedef Handle EdgeHandle;
    typedef Handle HalfEdgeHandle;

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
        std::vector<VertHandle> vert_handle_vec;
        Normal normal;
        FACEFLAG flag;
    };

    class Edge
    {
    public:
        HalfEdgeHandle he_handle_1;
        HalfEdgeHandle he_handle_2;

        EDGEFLAG flag;
    };

    class HalfEdge
    {
    public:
        HalfEdge();
        ~HalfEdge();
        
    public:
        HalfEdgeHandle prev_he_handle; /// previous halfedge
        HalfEdgeHandle next_he_handle; /// next halfedge
        HalfEdgeHandle oppo_he_handle; /// opposite halfedge

        VertHandle vert_handle; /// vertex of halfedge
        FaceHandle face_handle; /// face of halfedge, -1 if boundary halfedge
        EdgeHandle edge_handle; /// edge of halfedge
    };
    

    class MeshInfo
    {
    public:
        MESHFLAG flag;

        bool IsTriMesh() const {return tri_mesh; }
        bool IsQuadMesh() const { return quad_mesh; }
        bool IsPolyMesh() const { return poly_mesh; }
        bool IsManifold() const { return manifold; }

        std::string GetModelName() const { return model_name; }
        size_t GetVertNum() const { return vert_num; }
        size_t GetFaceNum() const { return face_num; }
        size_t GetEdgeNum() const { return edge_num; }
        
    private:
        bool tri_mesh;
        bool quad_mesh;
        bool poly_mesh;
        bool manifold;

        std::string model_name;
        size_t vert_num;
        size_t face_num;
        size_t edge_num;

        friend class MeshKernel;
        friend class MeshIO;
    };

    inline HalfEdge::HalfEdge():
        prev_he_handle(-1), next_he_handle(-1), oppo_he_handle(-1),
        vert_handle(-1), face_handle(-1), edge_handle(-1)
    {
    }

    inline HalfEdge::~HalfEdge(){}
}

#endif
