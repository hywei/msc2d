#include "msc2d_simplification.h"
#include "mscomplex.h"
#include "../common/macro.h"

using namespace std;
namespace msc2d{
Simplifor::Simplifor(MSComplex2D& _msc):
    msc(_msc), cp_vec(_msc.cp_vec), il_vec(_msc.il_vec){}
Simplifor::~Simplifor(){}

void Simplifor::calPersistence(){
  vector<PersPair> pers_vec;
  sum_persistence=0.0;
  for(size_t k=0; k<il_vec.size(); ++k){
    const IntegrationLine& il = il_vec[k];
    double value = msc.calPersistence(il.startIndex, il.endIndex);
    sum_persistence += value*value;
    pers_vec.push_back(PersPair(k, value));
  }
  // normalize
  sum_persistence = sqrt(sum_persistence);
  persistence_set.clear();
  if(fabs(sum_persistence) < meshlib::LARGE_ZERO_EPSILON ) sum_persistence = 1.0;
  for(size_t k=0; k<pers_vec.size(); ++k){
    persistence_set.insert(PersPair(pers_vec[k].first,
                                    pers_vec[k].second/sum_persistence));
  }
}

void Simplifor::simplify(double threshold){
  calPersistence();
  while(persistence_set.size()>4){
    const PersPair& pp = *persistence_set.begin();
    if(pp.second > threshold) break;
    cancel(pp.first);
  }

  update();
  refinePath();
}


bool Simplifor::cancel(int cancelIL_index){
  const IntegrationLine& cancelIL = il_vec[cancelIL_index];
  int scp_index(cancelIL.startIndex), mcp_index(cancelIL.endIndex);
  assert(removed_cp_flag[scp_index] == removed_cp_flag[mcp_index]);
  if(!removed_cp_flag[scp_index] || !removed_cp_flag[mcp_index]) return false;
  
  const CriticalPoint& s = cp_vec[scp_index];
  const CriticalPoint& m = cp_vec[mcp_index];

  int bridgeIL_index = (cancelIL_index+2)%s.neighbor.size();
  if(bridgeIL_index == cancelIL_index){// at boundary
    removeSad(scp_index);
    return true;
  }

  if(il_vec[bridgeIL_index].endIndex == mcp_index){
    // in this case(two il's endpoints are the same), 
    // we cancel the il between this two il
    int mid_il_idx(-1), il_idx1, il_idx2;
    size_t nb_num = m.neighbor.size();
    for(size_t k=0; k<nb_num; ++k){
      il_idx1 = m.neighbor[k].integrationLineIndex;
      il_idx2 = m.neighbor[(k+2)%nb_num].integrationLineIndex;
      if(il_idx1 == cancelIL_index && il_idx2 == bridgeIL_index ||
         il_idx1 == bridgeIL_index && il_idx2 == cancelIL_index){
        mid_il_idx = m.neighbor[(k+1)%nb_num].integrationLineIndex;
        break;
      }
    }
    if(mid_il_idx == -1){
      for(size_t k=0; k<nb_num; ++k){
        int cp_idx = m.neighbor[k].pointIndex;
        if(cp_vec[cp_idx].neighbor.size() == 1){
          mid_il_idx = m.neighbor[k].integrationLineIndex; break;
        }
      }
    }
    if(mid_il_idx == -1) return false;
    return cancel(mid_il_idx); 
  }
  
  const CriticalPoint& _m = cp_vec[il_vec[bridgeIL_index].endIndex];
  transferConnection(il_vec[cancelIL_index].endIndex,
                     il_vec[bridgeIL_index].endIndex, cancelIL_index, bridgeIL_index);

  // remove the saddle and the maximal/minimal
  removed_cp_flag[mcp_index] = false;
  removeSad(scp_index);

  return true;
}

void Simplifor::transferConnection(int cp1_idx, int cp2_idx, int il1_idx, int il2_idx){
  //! transfer cp1's neighbor to cp2 
  CriticalPointNeighborArray& nb1 = cp_vec[cp1_idx].neighbor;
  CriticalPointNeighborArray& nb2 = cp_vec[cp2_idx].neighbor;

  int nb_il_idx1 = getILIndexInNeighbor(cp_vec[cp1_idx], il1_idx);
  int nb_il_idx2 = getILIndexInNeighbor(cp_vec[cp2_idx], il2_idx);
  assert(nb_il_idx1 != -1 && nb_il_idx2 != -1);
  nb2.erase(nb2.begin() + nb_il_idx2);
  for(int i=(nb_il_idx1+1)%nb1.size(); i != nb_il_idx1; i=(i+1)%nb1.size()){
    nb2.insert(nb2.begin() + nb_il_idx2, nb1[i]);
    ++nb_il_idx2;
  }
  
  // make new path
  PATH ext_path = il_vec[il1_idx].path; ext_path.pop_back();
  reverse(ext_path.begin(), ext_path.end()); 
  ext_path.insert(ext_path.end(), il_vec[il2_idx].path.begin()+1, il_vec[il2_idx].path.end());
  
  for(size_t i=0; i<nb1.size(); ++i){
    int il_idx = nb1[i].integrationLineIndex;
    if(il_idx == il1_idx) continue;
    IntegrationLine& il = il_vec[il_idx];
    il.path.insert(il.path.end(), ext_path.begin(),  ext_path.end());
    il.endIndex = cp2_idx;
    double new_ps = msc.calPersistence(il.startIndex, il.endIndex)/sum_persistence;
    persistence_set.insert(make_pair(il_idx, new_ps));
  }
}

void Simplifor::removeSad(int cp_index) {
  const CriticalPoint& cp = cp_vec[cp_index];  
  removed_cp_flag[cp_index] = false;
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    int il_index = cp.neighbor[k].integrationLineIndex;
    removed_il_flag[il_index] = false;
    const IntegrationLine& il = il_vec[il_index];
    CriticalPoint& _cp = cp_vec[il.endIndex];
    int nb_il_idx = getILIndexInNeighbor(_cp, il_index);
    assert(nb_il_idx != -1);
    _cp.neighbor.erase(_cp.neighbor.begin() + nb_il_idx);
    double _ps = msc.calPersistence(il_vec[il_index].startIndex, il_vec[il_index].endIndex);
    set<PersPair, PersPairCmp>::iterator is = persistence_set.find(PersPair(il_index, _ps));
    assert(is != persistence_set.end());
    persistence_set.erase(is);
  }
}

void Simplifor::update(){
  //! make index mapping
  vector<int> cp_index_mapping(cp_vec.size());
  vector<int> il_index_mapping(il_vec.size());
  int cp_index=0, il_index=0;
  for(size_t i=0; i<cp_vec.size(); ++i)
    if(!removed_cp_flag[i]) cp_index_mapping[i] = cp_index++;
  for(size_t i=0; i<il_vec.size(); ++i)
    if(!removed_il_flag[i]) il_index_mapping[i] = il_index++;

  //! update neighbor
  for(size_t i=0; i<cp_vec.size(); ++i){
    CriticalPointNeighborArray& nb = cp_vec[i].neighbor;
    for(size_t k=0; k<nb.size(); ++k){
      nb[k].pointIndex = cp_index_mapping[nb[k].pointIndex];
      nb[k].integrationLineIndex = il_index_mapping[nb[k].integrationLineIndex];
    }
  }
  for(size_t i=0; i<il_vec.size(); ++i){
    IntegrationLine& il = il_vec[i];
    il.startIndex = cp_index_mapping[il.startIndex];
    il.endIndex = cp_index_mapping[il.endIndex];
  }

  //! remove critial points and integration lines
  CriticalPointArray new_cp_vec;
  for(size_t i=0; i<cp_vec.size(); ++i)
    if(!removed_cp_flag[i]) new_cp_vec.push_back(cp_vec[i]);
  swap(cp_vec, new_cp_vec);
  IntegrationLineArray new_il_vec;
  for(size_t i=0; i<il_vec.size(); ++i)
    if(!removed_il_flag[i]) new_il_vec.push_back(il_vec[i]);
  swap(il_vec, new_il_vec);

  fill(msc.vert_cp_index_mp.begin(), msc.vert_cp_index_mp.end(), -1);
  for(size_t k=0; k<cp_vec.size(); ++k){
    msc.vert_cp_index_mp[cp_vec[k].meshIndex] = k;
  }

}

void Simplifor::refinePath(){
  //! remove circle
  for(size_t i=0; i<il_vec.size(); ++i){
    PATH& path  = il_vec[i].path;
    for(size_t k=0; k<path.size(); ++k){
      for(size_t j=path.size()-1; j!=k; --j){
        if(path[j] == path[k]){
          path.erase(path.begin()+k+1, path.begin()+j+1);
        }
      }
    }
  }
}

int Simplifor::getILIndexInNeighbor(const CriticalPoint& cb, int il_index) const{
  for(size_t i=0; i<cb.neighbor.size(); ++i){
    if(cb.neighbor[i].integrationLineIndex == il_index) return i;
  }
  return -1;
}


} // end namespace
