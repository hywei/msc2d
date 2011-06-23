#ifndef MESHLIB_MESHINFO_H_
#define MESHLIB_MESHINFO_H_

#include <string>
#include "../common/types.h"

namespace MeshLib{
    class Mesh;
    
    class ModelInfo
    {
    public:
        // Constructor / Destructor
        ModelInfo(Mesh& _mesh){}
        ~ModelInfo(){}
        
        std::string GetFileName() const { return m_FileName; }
    
        size_t GetVertNum() const { return m_nVertices; }
        size_t GetFaceNum() const { return m_nFaces; }
        size_t GetHalfEdgeNum() const { return m_nHalfEdges; }    
        size_t GetBoundaryNum() const { return m_nBoundaries; }
        size_t GetComponentNum() const { return m_nComponents; }
        double GetAvgEdgeLength() const { return m_AvgEdgeLength; }
        double GetAvgFaceArea() const{ return m_AvgFaceArea; }        
        void GetBoundingBox(Coord& BoxMin, Coord& BoxMax, Coord& BoxDim) const { BoxMin = m_BoxMin; BoxMax = m_BoxMax; BoxDim = m_BoxDim; }
        void GetBoundingSphere(Coord& Center, double& radius) const { Center = m_SphereCenter; radius = m_SphereRadius; }
        PolyIndexArray& GetBoundary() { return m_Boundaries; }
        
        bool IsTriMesh() const; // model is a triangle mesh (only containing triangles)
        bool IsQuadMesh() const; // model is a quadrangle mesh (only containing quadangles)
        bool IsGeneralMesh() const; // model is a general mesh (the rest, not a tri- or quad)
        bool IsClosed() const; // model is a closed one (no boundaries)
        bool IsManifold() const; // model is a 2-manifold one (locally disc-like)
        bool IsTriManifold() const; // model is a 2-mainfold triangle mesh
        bool IsPatch() const; // model is a patch (single boundary, 2-manifold)

    private:
        std::string  m_FileName; // File name(Path + Tilte + Ext)
        size_t     m_nVertices; // Number of valid vertices
        size_t     m_nFaces;    // Number of valid faces
        size_t     m_nHalfEdges; // Number of valid half edges

        size_t     m_nBoundaries;  // Number of boundaries
        size_t     m_nComponents;  // Number of connected components

        Coord   m_BoxMin, m_BoxMax, m_BoxDim;
        Coord   m_SphereCenter;
        double  m_SphereRadius;

        double  m_AvgEdgeLength;
        double  m_AvgFaceArea;

        PolyIndexArray  m_Boundaries;   // Boundary vertex loop
        
        friend class MeshKernel;
        friend class MeshIO;
        friend class MeshBasicOP;
    };
}
#endif
