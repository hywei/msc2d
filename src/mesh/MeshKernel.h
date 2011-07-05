#ifndef MESHLIB_MESHKERNEL_H_
#define MESHLIB_MESHKERNEL_H_

#include "MeshElement.h"
#include "MeshInfo.h"

#include <vector>

namespace meshlib{

  class Mesh;
  class MeshKernel
  {
 public:
 MeshKernel(Mesh& _mesh): mesh(_mesh) {}
    ~MeshKernel(){}

    //! analysis model info
    void AnalysisModel();
    //! create halfedge data structure        
    bool CreateHalfEdgeDS();        

    VertArray& getVertArray() { return vert_vec; }
    FaceArray& getFaceArray() { return face_vec; }
    EdgeArray& getEdgeArray() { return edge_vec; }
    HalfEdgeArray& getHEArray() { return he_vec; }

 private:
    VertArray vert_vec;
    FaceArray face_vec;
    EdgeArray edge_vec;
    HalfEdgeArray he_vec;        
        
    Mesh& mesh;

    friend class MeshIO;
    friend class MeshBasicOP;
    friend class MeshInfo;
  };
}
#endif
