#ifndef MESHLIB_MESH_H_
#define MESHLIB_MESH_H_

#include <boost/shared_ptr.hpp>
#include <string>
#include "../common/types.h"
#include "MeshElement.h"

namespace meshlib{

  class MeshKernel;
  class MeshIO;
  class MeshBasicOP;
  class MeshInfo;
    
  class Mesh
  {
 public:
    // Constructor/Destructor
    Mesh();
    ~Mesh();
        
    // Input/Output functions
    bool attachModel(const std::string& filename);
    bool storeModel(const std::string& filename) const;

    size_t getVertexNumber() const;
    size_t getFaceNumber() const;
    size_t getEdgeNumber() const;
        
    const Coord3D& getVertexCoord(VertHandle vh) const;
    const Coord3D& getVertexNorm(VertHandle vh) const;
    const Coord3D& getFaceNorm(FaceHandle fh) const;

    const VertHandleArray& getAdjVertices(VertHandle vh) const;
    const FaceHandleArray& getAdjFaces(VertHandle vh) const;
    const VertHandleArray& getFaceVertices(FaceHandle fh) const;

    bool isBoundaryVertex(VertHandle vh) const;
    bool isBoundaryFace(FaceHandle fh) const;
    bool isBoundaryEdge(EdgeHandle eh) const;        

    bool isManifold() const;
 private:            
    boost::shared_ptr<MeshKernel> p_Kernel;
    boost::shared_ptr<MeshIO> p_IO;
    boost::shared_ptr<MeshBasicOP> p_BasicOP;
    boost::shared_ptr<MeshInfo> p_Info;

    friend class MeshKernel;
    friend class MeshIO;
    friend class MeshBasicOP;
    friend class MeshInfo;
  };


} // 
#endif
