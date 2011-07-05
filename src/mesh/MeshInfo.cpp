#include "MeshInfo.h"
#include "Mesh.h"
#include "MeshKernel.h"
#include "../util/utility.h"

using namespace std;

namespace meshlib{

bool MeshInfo::isBoundaryVertex(VertHandle vh) const
{
  const Vert& v = (mesh.p_Kernel->vert_vec)[vh];
  return Util::IsSetFlag(v.flag, BOUNDARY_VERT);
}

bool MeshInfo::isBoundaryFace(FaceHandle fh) const
{
  const Face& f = (mesh.p_Kernel->face_vec)[fh];
  return Util::IsSetFlag(f.flag, BOUNDARY_FACE);
}

bool MeshInfo::isBoundaryEdge(EdgeHandle eh) const
{
  const Edge& e = (mesh.p_Kernel->edge_vec)[eh];
  return Util::IsSetFlag(e.flag, BOUNDARY_EDGE);
}

bool MeshInfo::isManifold() const
{
  return !Util::IsSetFlag(flag, MANIFOLD);
}

}
