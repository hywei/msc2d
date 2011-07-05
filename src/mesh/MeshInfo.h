#ifndef MESHLIB_MESHINFO_H_
#define MESHLIB_MESHINFO_H_

#include <string>
#include "../common/types.h"
#include "MeshElement.h"

namespace meshlib{
  class Mesh;
  class MeshInfo
  {
 public:
    // Constructor / Destructor
    MeshInfo(Mesh& _mesh): mesh(_mesh){}
    ~MeshInfo(){}
    
    size_t getVertNum() const { return m_nVertices; }
    size_t getFaceNum() const { return m_nFaces; }
    size_t getHalfEdgeNum() const { return m_nHalfEdges; }    
    size_t getBoundaryNum() const { return m_nBoundaries; }
    size_t getComponentNum() const { return m_nComponents; }
    double getAvgEdgeLength() const { return m_AvgEdgeLength; }
    double getAvgFaceArea() const{ return m_AvgFaceArea; }

    const BoundingBox& getBoundingBox() const { return m_BoundingBox; }
    const BoundingSphere& getBoundingSphere() const { return m_BoundingSphere; }
    const BoundaryArray& getBoundaries() const { return m_Boundaries; }
        
    bool isTriMesh() const; // model is a triangle mesh (only containing triangles)
    bool isQuadMesh() const; // model is a quadrangle mesh (only containing quadangles)
    bool isGeneralMesh() const; // model is a general mesh (the rest, not a tri- or quad)
    bool isClosed() const; // model is a closed one (no boundaries)
    bool isManifold() const; // model is a 2-manifold one (locally disc-like)
    bool isTriManifold() const; // model is a 2-mainfold triangle mesh
    bool isPatch() const; // model is a patch (single boundary, 2-manifold)

    bool isBoundaryVertex(VertHandle vh) const;
    bool isBoundaryFace(FaceHandle fh) const;
    bool isBoundaryEdge(EdgeHandle eh) const;
 private:
    Mesh& mesh;
    size_t     m_nVertices; // Number of valid vertices
    size_t     m_nFaces;    // Number of valid faces
    size_t     m_nHalfEdges; // Number of valid half edges
    
    size_t     m_nBoundaries;  // Number of boundaries
    size_t     m_nComponents;  // Number of connected components

    BoundingBox m_BoundingBox;
    BoundingSphere m_BoundingSphere;
    BoundaryArray m_Boundaries;
    
    double  m_AvgEdgeLength;
    double  m_AvgFaceArea;

    MESHFLAG flag;        
    friend class MeshIO;
    friend class MeshBasicOP;
  };
}
#endif
