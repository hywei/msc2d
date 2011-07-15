#include "quad_patch_generator.h"
using namespace std;
using namespace meshlib;

namespace msc2d{
QPGenerator::QPGenerator(MSComplex2D& _msc): msc(_msc){}
QPGenerator::~QPGenerator(){}

void QPGenerator::genQuadPatchArray(const CriticalPoint& cp){
  size_t cn = cp.neighbor.size();
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    const CriticalPointNeighbor& cp_nb1 = cp.neighbor[k];
    const CriticalPointNeighbor& cp_nb2 = cp.neighbor[(k+1)%cn];
    vector<CriticalPointNeighbor> nb_vec; nb_vec.push_back(cp_nb1);
    vector<int> il_index_vec; il_index_vec.push_back(cp_nb1.integrationLineIndex);
    for(size_t i=0; i<4; ++i){
      nb_vec.push_back(getNextCPNeighbor(nb_vec[nb_vec.size()-1]));
      il_index_vec.push_back(nb_vec[nb_vec.size()-1].integrationLineIndex);
    }
    if(nb_vec[4] == nb_vec[1]){
      il_index_vec.pop(); sort(il_index_vec.begin(), il_index_vec.end());
      if(!Util::isIn(formed_patch, il_index_vec)){
        //! make a new patch
        
      }
    }else{
      //! TODO boundary 
    }
  }
}

CriticalPointNeighbor QPGenerator::getNextCPNeighbor(const CriticalPointNeighbor& nb) const{
  const CriticalPoint& _cp = msc.cp_vec[nb.pointIndex];
  const vector<CriticalPointNeighbor>& _nb_vec = _cp.neighbor;
  int il_index = nb.integrationLineIndex, idx=-1;
  size_t _nb_num = _nb_vec.neighbor.size();;
  for(size_t k=0; k<_nb_num; ++k)
    if(_nb_vec[k].integrationLineIndex == il_index) {idx=k; break;}
  assert(idx != -1);
  return _nb_vec[idx];
}
