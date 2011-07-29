#include "msc2d_simplification.h"
#include "mscomplex.h"
#include "../common/macro.h"
#include <limits>

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
  if(fabs(sum_persistence) < meshlib::LARGE_ZERO_EPSILON ) sum_persistence = 1.0;
  persistence_map.clear();
  for(size_t i=0; i<pers_vec.size(); ++i){
    double value = pers_vec[i].second/sum_persistence;
    if(value <= cancel_threshold) persistence_map[pers_vec[i].first] = value;
  }
}

void Simplifor::simplify(double threshold){
  cancel_threshold = threshold;
  cout << "Simplication, threshold = " << threshold << endl;
  removed_il_flag.clear(); removed_il_flag.resize(il_vec.size(), false);
  removed_cp_flag.clear(); removed_cp_flag.resize(cp_vec.size(), false);
  calPersistence();
  while(persistence_map.size()){
    double min_pers = numeric_limits<double>::infinity();
    map<size_t, double>::iterator im, min_im;
    for(im=persistence_map.begin(); im!=persistence_map.end(); ++im){
      if(im->second < min_pers){
        min_pers = im->second;
        min_im = im;
      }
    }
    size_t il_index = min_im->first;
    persistence_map.erase(il_index);
    cancel(il_index);

  }

  update();
  refinePath();
}


bool Simplifor::cancel(int cancelIL_index){

  const IntegrationLine& cancelIL = il_vec[cancelIL_index];

//  cout << "cancel " << cp_vec[cancelIL.startIndex].meshIndex << " "
//      << cp_vec[cancelIL.endIndex].meshIndex << " " << cancelIL_index << endl;

  int scp_index(cancelIL.startIndex), mcp_index(cancelIL.endIndex);
  if(removed_cp_flag[scp_index] || removed_cp_flag[mcp_index]) return false;
  assert(removed_cp_flag[scp_index] == removed_cp_flag[mcp_index]);
  
  const CriticalPoint& s = cp_vec[scp_index];
  const CriticalPoint& m = cp_vec[mcp_index];

  if(s.neighbor.size() == 1) { removeSad(scp_index); return false; }

  int cancel_nb_idx = getILIndexInNeighbor(s, cancelIL_index);

  int bridgeIL_index;
  int next_il_index1 = s.neighbor[(cancel_nb_idx+1)%s.neighbor.size()].integrationLineIndex;
  int next_il_index2 = s.neighbor[(cancel_nb_idx+2)%s.neighbor.size()].integrationLineIndex;
  bool is_ascending_il1 = isAscendingIL(il_vec[cancelIL_index]);
  bool is_ascending_il2 = isAscendingIL(il_vec[next_il_index1]);
  bool is_ascending_il3 = isAscendingIL(il_vec[next_il_index2]);
  if(is_ascending_il1 == is_ascending_il2) bridgeIL_index = next_il_index1; // at boundary
  else if(is_ascending_il1 == is_ascending_il3) bridgeIL_index = next_il_index2;
  else{ // at boundary
    removeSad(scp_index);
    return false;
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
  
  transferConnection(il_vec[cancelIL_index].endIndex,
                     il_vec[bridgeIL_index].endIndex, cancelIL_index, bridgeIL_index);

  // remove the saddle and the maximal/minimal
  removed_cp_flag[mcp_index] = true;
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
  size_t nb1_num = nb1.size();
  for(int i=(nb_il_idx1+1)%nb1_num; i != nb_il_idx1; i=(i+1)%nb1_num){
    CriticalPoint& _s = cp_vec[nb1[i].pointIndex];
    int _nb_idx = getILIndexInNeighbor(_s, nb1[i].integrationLineIndex);
    _s.neighbor[_nb_idx].pointIndex = cp2_idx;
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
    if(new_ps <= cancel_threshold ) persistence_map[il_idx] = new_ps;
    else{
      map<size_t, double>::iterator iter = persistence_map.find(il_idx);
      if(iter != persistence_map.end()) persistence_map.erase(iter);
    }
  }
}

void Simplifor::removeSad(int cp_index) {
  const CriticalPoint& cp = cp_vec[cp_index];

  removed_cp_flag[cp_index] = true;
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    int il_index = cp.neighbor[k].integrationLineIndex;
    removed_il_flag[il_index] = true;
    const IntegrationLine& il = il_vec[il_index];
    CriticalPoint& _cp = cp_vec[il.endIndex];
    int nb_il_idx = getILIndexInNeighbor(_cp, il_index);
    assert(nb_il_idx != -1);
    _cp.neighbor.erase(_cp.neighbor.begin() + nb_il_idx);
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
          break;
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

bool Simplifor::isAscendingIL(const IntegrationLine& il) const{
  int vid1 = il.path[0], vid2 = il.path[1];
  return msc.cmpScalarValue(vid1, vid2) == 1;
}

} // end namespace
