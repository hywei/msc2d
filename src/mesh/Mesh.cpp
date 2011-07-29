#include "Mesh.h"
#include "MeshKernel.h"
#include "MeshIO.h"
#include "MeshBasicOp.h"
#include "MeshInfo.h"

namespace meshlib{
    
Mesh::Mesh(){}
Mesh::~Mesh(){}

// Input/Output functions
bool Mesh::attachModel(const std::string& filename){

  p_Kernel.reset(); p_IO.reset(); p_BasicOP.reset(); p_Info.reset();
    
  p_Kernel = boost::shared_ptr<MeshKernel> (new MeshKernel(*this));
  p_IO = boost::shared_ptr<MeshIO> (new MeshIO(*this));
  p_BasicOP = boost::shared_ptr<MeshBasicOP> (new MeshBasicOP(*this));
  p_Info = boost::shared_ptr<MeshInfo> (new MeshInfo(*this));
    
  if(!p_IO->LoadModel(filename)) return false;

  p_BasicOP->initModel();
  return true;
}

bool Mesh::storeModel(const std::string& filename) const { return p_IO->StoreModel(filename); }
size_t Mesh::getVertexNumber() const { return  p_Kernel->getVertArray().size(); }
size_t Mesh::getFaceNumber() const { return p_Kernel->getFaceArray().size(); }
size_t Mesh::getEdgeNumber() const { return p_Kernel->getEdgeArray().size(); }
    
const Coord3D& Mesh::getVertexCoord(VertHandle vh) const
{
  return p_Kernel->getVertArray()[vh].coord;
}
    
const Coord3D& Mesh::getVertexNorm(VertHandle vh) const
{
  return p_Kernel->getVertArray()[vh].normal;
}
    
const Coord3D& Mesh::getFaceNorm(FaceHandle fh) const
{
  return p_Kernel->getFaceArray()[fh].normal;
}

const std::vector<VertHandle>& Mesh::getAdjVertices(VertHandle vh) const
{
  return p_BasicOP->getAdjVertArray(vh);
}
    
const std::vector<FaceHandle>& Mesh::getAdjFaces(VertHandle vh) const
{
  return p_BasicOP->getAdjFaceArray(vh);
}

const std::vector<VertHandle>& Mesh::getFaceVertices(FaceHandle fh) const
{
  return p_Kernel->getFaceArray()[fh].vert_handle_vec;
}

const EdgeHandleArray& Mesh::getFaceEdges(FaceHandle fh) const{
  return p_Kernel->getFaceArray()[fh].edge_handle_vec;
}

const HalfEdgeHandleArray& Mesh::getFaceHalfEdges(FaceHandle fh) const{
  return p_Kernel->getFaceArray()[fh].he_handle_vec;
}

const VertArray& Mesh::getVertexArray() const {
  return p_Kernel->getVertArray();
}

const EdgeArray& Mesh::getEdgeArray() const{
  return p_Kernel->getEdgeArray();
}

const FaceArray& Mesh::getFaceArray() const{
  return p_Kernel->getFaceArray();
}

const HalfEdgeArray& Mesh::getHalfEdgeArray() const{
  return p_Kernel->getHEArray();
}

std::pair<VertHandle, VertHandle> Mesh::getEdgeVertices(EdgeHandle eh) const{
  const Edge& e = p_Kernel->getEdgeArray()[eh];
  return std::make_pair(e.vert_handle_1, e.vert_handle_2);
}

EdgeHandle Mesh::getEdgeHandle(VertHandle vh1, VertHandle vh2) const{
  return p_BasicOP->getEdgeHandle(vh1, vh2);
}

HalfEdgeHandle Mesh::getHalfEdgeHandle(VertHandle vh1, VertHandle vh2) const{
  return p_BasicOP->getHalfEdgeHandle(vh1, vh2);
}

bool Mesh::getInnerFaces(const PATH &loop, FaceHandleArray &fh_vec) const{
  return p_BasicOP->getInnerFaces(loop, fh_vec);
}

bool Mesh::isBoundaryVertex(VertHandle vh) const { return p_Info->isBoundaryVertex(vh); }
bool Mesh::isBoundaryFace(FaceHandle fh) const { return p_Info->isBoundaryFace(fh); }
bool Mesh::isBoundaryEdge(EdgeHandle eh) const { return p_Info->isBoundaryEdge(eh); }

bool Mesh::isManifold() const
{
  return p_Info->isManifold();
}

bool Mesh::getShortestPath(VertHandle vh1, VertHandle vh2,
                           PATH &path, std::set<EdgeHandle> &edge_set) const{
  return p_BasicOP->getShortestPath(vh1, vh2, path, edge_set);
}

}
