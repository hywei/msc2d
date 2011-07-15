#include "quad_patch_generator.h"
#include "../mesh/Mesh.h"
#include "../util/utility.h"
using namespace std;
using namespace meshlib;

namespace msc2d{
QPGenerator::QPGenerator(MSComplex2D& _msc): msc(_msc){}
QPGenerator::~QPGenerator(){}

void QPGenerator::genQuadPatchArray(const CriticalPoint& cp){
  size_t cn = cp.neighbor.size();
  formed_patchs.clear();
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    const CriticalPointNeighbor& cp_nb1 = cp.neighbor[k];
    const CriticalPointNeighbor& cp_nb2 = cp.neighbor[(k+1)%cn];
    vector<CriticalPointNeighbor> nb_vec; nb_vec.push_back(cp_nb1);
    vector<int> il_index_vec; il_index_vec.push_back(cp_nb1.integrationLineIndex);
    for(size_t i=0; i<4; ++i){
      nb_vec.push_back(getNextCPNeighbor(nb_vec[nb_vec.size()-1]));
      il_index_vec.push_back(nb_vec[nb_vec.size()-1].integrationLineIndex);
    }
    if(il_index_vec[4] == il_index_vec[0]){
      il_index_vec.pop_back();
      vector<int> il_index_vec_bak = il_index_vec;
      sort(il_index_vec.begin(), il_index_vec.end());
      if(!Util::isIn(formed_patchs, il_index_vec)){
        //! make a new patch
        msc.qp_vec.push_back(QuadPatch());
        QuadPatch& patch = msc.qp_vec[msc.qp_vec.size()-1];
        patch.boundaryIntegrationLineIndex = il_index_vec_bak;
        formed_patchs.push_back(il_index_vec);
      }
    }else{
      //! TODO boundary
      
    }
  }

  for(size_t k=0; k<msc.qp_vec.size(); ++k){
    QuadPatch& patch = msc.qp_vec[k];
    if(!findPatchInnerFace(patch)){
      cerr << "Cannot find inner faces for patch " << k << endl;
    }
  }
}

CriticalPointNeighbor QPGenerator::getNextCPNeighbor(const CriticalPointNeighbor& nb) const{
  const CriticalPoint& _cp = msc.cp_vec[nb.pointIndex];
  const vector<CriticalPointNeighbor>& _nb_vec = _cp.neighbor;
  int il_index = nb.integrationLineIndex, idx=-1;
  size_t _nb_num = _nb_vec.size();;
  for(size_t k=0; k<_nb_num; ++k)
    if(_nb_vec[k].integrationLineIndex == il_index) {idx=k; break;}
  assert(idx != -1);
  return _nb_vec[idx];
}

bool QPGenerator::findPatchInnerFace(QuadPatch& patch) const{
  PATH bd_loop;
  for(size_t k=0; k<patch.boundaryIntegrationLineIndex.size(); ++k){
    int il_index = patch.boundaryIntegrationLineIndex[k];
    const PATH& path = msc.il_vec[il_index].path;
    if(bd_loop.size() == 0) bd_loop.insert(bd_loop.begin(), path.begin(), path.end());
    else{
      int prev_vid = bd_loop[bd_loop.size()-1];
      int first_vid = path[0], last_vid = path[path.size()-1];
      if(first_vid == prev_vid)
        bd_loop.insert(bd_loop.end(), bd_loop.begin()+1, bd_loop.end());
      else if(last_vid == prev_vid)
        bd_loop.insert(bd_loop.end(), bd_loop.rbegin()+1, bd_loop.rend());
      else return false;
    }
  }
  assert(bd_loop[bd_loop.size()-1] == bd_loop[0]);
  return true;
}

} // end namespace 
