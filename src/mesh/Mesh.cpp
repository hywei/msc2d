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

bool Mesh::isBoundaryVertex(VertHandle vh) const { return p_Info->isBoundaryVertex(vh); }
bool Mesh::isBoundaryFace(FaceHandle fh) const { return p_Info->isBoundaryFace(fh); }
bool Mesh::isBoundaryEdge(EdgeHandle eh) const { return p_Info->isBoundaryEdge(eh); }

bool Mesh::isManifold() const
{
  return p_Info->isManifold();
}

}
