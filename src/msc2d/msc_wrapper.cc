#include "msc_wrapper.h"
#include <limits>
#include "../common/macro.h"
#include "../mesh/Mesh.h"

using namespace std;
using namespace meshlib;

namespace msc2d{
MscWrapper::MscWrapper(MSComplex2D& msc):
    mesh(*msc.mesh),
    sf(msc.scalar_field),
    cp_vec(msc.cp_vec),
    il_vec(msc.il_vec){}

MscWrapper::~MscWrapper(){}

int MscWrapper::cmpScalarValue(size_t vid1, size_t vid2) const{
  if( fabs(sf[vid1] - sf[vid2]) < LARGE_ZERO_EPSILON ){
    int pri_1 = vert_priority_mp.find(vid1)->second;
    int pri_2 = vert_priority_mp.find(vid2)->second;
    assert(pri_1 != pri_2);
    return (pri_1 > pri_2) ? 1 : -1;
  }else{

    return sf[vid1] > sf[vid2] ? 1 : -1;
  }
}

double MscWrapper::calGradient(size_t vid1, size_t vid2) const{
  if(vid1 == vid2) return 0.0;
  const Coord3D& coord1 = mesh.getVertexCoord(vid1);
  const Coord3D& coord2 = mesh.getVertexCoord(vid2);
  double dis = (coord1-coord2).abs();
  if(dis < LARGE_ZERO_EPSILON) return numeric_limits<double>::infinity();
  return (sf[vid1] - sf[vid2])/dis;
}

}
