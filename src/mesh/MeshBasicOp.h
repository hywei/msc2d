#ifndef MESHLIB_MESHBASICOP_H_
#define MESHLIB_MESHBASICOP_H_

#include <vector>
#include "../common/types.h"
#include "MeshElement.h"

namespace meshlib{

    class Mesh;
    
    class MeshBasicOP
    {   
    public:
        MeshBasicOP(Mesh& mesh);
        ~MeshBasicOP();

        void initModel();

        const VertHandleArray& getAdjVertArray(const VertHandle&) const;
        const FaceHandleArray& getAdjFaceArray(const VertHandle&) const;
        bool getInnerFaces(const PATH& loop, FaceHandleArray& fh_vec) const;
        EdgeHandle getEdgeHandle(VertHandle vh1, VertHandle vh2) const;
        HalfEdgeHandle getHalfEdgeHandle(VertHandle vh1, VertHandle vh2) const;
        
    private:
        void genEdgeInfo();
        void genHalfEdgeDS(); //! only be call for manifold mesh

        void genAdjacentInfo();
        void genVertAdjacentInfo();
        void genEdgeAdjacentInfo();
        void genFaceAdjacentInfo();
      
        void calVertNormal();
        void calFaceNormal();

        BoundingBox calBoundingBox() const;
        BoundingSphere calBoundingSphere() const;

        size_t countComponentNum() const; 
        double calAvgEdgeLength() const;

        void analysisModel();
        void sortAdjacentInfo();
        
    private:
        Mesh& mesh;
        VertArray& vert_vec;
        FaceArray& face_vec;
        EdgeArray& edge_vec;
        HalfEdgeArray& he_vec;
        
        std::vector<VertHandleArray > vert_adj_vert_vec;
        std::vector<FaceHandleArray > vert_adj_face_vec;
        std::vector<EdgeHandleArray > vert_adj_edge_vec;
        std::vector<FaceHandleArray > edge_adj_face_vec;
        std::vector<FaceHandleArray > face_adj_face_vec;
    };
}
#endif
