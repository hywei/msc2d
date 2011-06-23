#ifndef MESHLIB_MESHKERNEL_H_
#define MESHLIB_MESHKERNEL_H_

#include "MeshElement.h"
#include "MeshInfo.h"

#include <vector>

namespace MeshLib{

    class Mesh;
    class MeshKernel
    {
    public:
        MeshKernel(Mesh& _mesh);
        ~MeshKernel();

        //! analysis model info
        void AnalysisModel();
        //! create halfedge data structure        
        bool CreateHalfEdgeDS();

        std::vector<Vert>& GetVertArray() { return vert_vec; }
        std::vector<Face>& GetFaceArray() { return face_vec; }
        std::vector<Edge>& GetEdgeArray() { return edge_vec; }
        std::vector<HalfEdge>& GetHEArray() { return he_vec; }
        
    private:
        std::vector<Vert> vert_vec;
        std::vector<Face> face_vec;
        std::vector<Edge> edge_vec;
        std::vector<HalfEdge> he_vec;

        MeshInfo mesh_info;
        
        Mesh& mesh;

        friend class MeshIO;
    };
}
#endif
