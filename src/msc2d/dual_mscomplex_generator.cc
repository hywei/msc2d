#include "dual_mscomplex_generator.h"
#include "../mesh/Mesh.h"
#include <set>

using namespace std;
using namespace meshlib;
namespace msc2d{

DualGenerator::DualGenerator(MSComplex2D& _msc):
    msc(_msc), cp_vec(_msc.cp_vec), il_vec(_msc.il_vec),
    qp_vec(_msc.qp_vec){}
DualGenerator::~DualGenerator(){}

void DualGenerator::generateDualCP(){
  cp_mapping.resize(cp_vec.size(), -1);
  vert_dual_cp_index_mapping.resize(msc.mesh->getVertexNumber(), -1);
  for(size_t i=0; i<cp_vec.size(); ++i){
    const CriticalPoint& cp =  cp_vec[i];
    if(cp.type != SADDLE){
      dual_cp_vec.push_back(cp);
      cp_mapping[i] = dual_cp_vec.size()-1;
      vert_dual_cp_index_mapping[cp.meshIndex] = dual_cp_vec.size()-1;
    }
  }
}

void DualGenerator::generateDualIL() {
  qp_dual_il_map.resize(qp_vec.size(), -1);
  for(size_t i=0; i<qp_vec.size(); ++i){
    const QuadPatch& qp = qp_vec[i];
    IntegrationLine dual_il;
    if(qp.boundaryIntegrationLineIndex.size() == 3){
      const IntegrationLine& il = il_vec[qp.boundaryIntegrationLineIndex[1]];
      dual_il.startIndex = cp_mapping[il.startIndex];
      dual_il.endIndex = cp_mapping[il.endIndex];
      dual_il.path = il.path;
      assert(dual_il.startIndex != -1 && dual_il.endIndex != -1);
      dual_il_vec.push_back(dual_il);
    }else if(qp.boundaryIntegrationLineIndex.size() == 4){
      pair<int, int> mm_pair = getMaxMinPair(qp);
      dual_il.startIndex = cp_mapping[mm_pair.first];
      dual_il.endIndex = cp_mapping[mm_pair.second];
      set<EdgeHandle> edge_set;
      for(size_t j=0; j<qp.boundaryIntegrationLineIndex.size(); ++j){
        const PATH& path = il_vec[qp.boundaryIntegrationLineIndex[j]].path;
        for(size_t k=0; k<path.size()-1; ++k){
          edge_set.insert(msc.mesh->getEdgeHandle(path[k], path[k+1]));
        }
      }
      for(size_t j=0; j<qp.face.size(); ++j){
        const VertHandleArray& eh_vec = msc.mesh->getFaceEdges(qp.face[j]);
        for(size_t k=0; k<eh_vec.size(); ++k) edge_set.insert(eh_vec[k]);
      }
      int startVid = dual_cp_vec[dual_il.startIndex].meshIndex;
      int endVid = dual_cp_vec[dual_il.endIndex].meshIndex;
      if(!msc.mesh->getShortestPath(startVid, endVid, dual_il.path, edge_set))
        cerr << "cannot form dual path for patch " << i << endl;

      dual_il_vec.push_back(dual_il);
    }
    qp_dual_il_map[i] = dual_il_vec.size()-1;
  }
}

void DualGenerator::generateDualPatch(){
  generateDualCP();
  generateDualIL();
  generateCPAdjInfo();
  for(size_t i=0; i<cp_vec.size(); ++i) {
    if(cp_vec[i].type == SADDLE)
      generateDualPatch(i);
  }  
}

void DualGenerator::generateDualPatch(int cp_index) {
  const CriticalPoint& cp = cp_vec[cp_index];
  const vector<int>& dual_patch_index_vec = cp_adj_patch_vec[cp_index];
  vector<int> dual_il_index_vec;
  for(size_t i=0; i<dual_patch_index_vec.size(); ++i){
    int pid = dual_patch_index_vec[i];
    int dual_iid = qp_dual_il_map[pid];
    if(dual_iid != -1) dual_il_index_vec.push_back(dual_iid);
  }

  if(dual_il_index_vec.size() != 4) return;

  QuadPatch patch;
  patch.boundaryIntegrationLineIndex = dual_il_index_vec;
  // make boundary loop
  PATH loop;
  const CriticalPoint& cp_0 = cp_vec[cp.neighbor[0].pointIndex];
  loop.push_back(cp_0.meshIndex);
  for(size_t i=0; i<4; ++i){
    const IntegrationLine& il = dual_il_vec[dual_il_index_vec[i]];
    if(il.startIndex == cp_mapping[cp.neighbor[i].pointIndex]){
      loop.insert(loop.end(), il.path.begin()+1, il.path.end());
    }else if(il.endIndex == cp_mapping[cp.neighbor[i].pointIndex]){
      loop.insert(loop.end(), il.path.rbegin()+1, il.path.rend());
    }else {
      cerr << "error " << __FILE__ << " " << __LINE__ << endl;
    }
  }
  assert(loop[0] == loop[loop.size()-1]);
  msc.mesh->getInnerFaces(loop, patch.face);
  if(!checkInnerFaces(loop, patch.face)) patch.face.clear();
  dp_vec.push_back(patch);
}

void DualGenerator::generateCPAdjInfo(){
  cp_adj_patch_vec.resize(cp_vec.size());
  for(size_t k=0; k<cp_vec.size(); ++k){
    const CriticalPoint& cp = cp_vec[k];
    for(size_t i=0; i<cp.neighbor.size(); ++i){
      size_t j = (i+1) % cp.neighbor.size();
      int il_index1 = cp.neighbor[i].integrationLineIndex;
      int il_index2 = cp.neighbor[j].integrationLineIndex;
      int pid = getPatchIndex(il_index1, il_index2);
      if(pid == -1){
        continue;
      }
      cp_adj_patch_vec[k].push_back(pid);
    }
  }
}

int DualGenerator::getPatchIndex(int il_index1, int il_index2) const{
  const IntegrationLine& il1 = il_vec[il_index1];
  const IntegrationLine& il2 = il_vec[il_index2];
  vector<int> common_pid_vec;
  for(size_t i=0; i<il1.quadPatchIndex.size(); ++i){
    int qp_idx = il1.quadPatchIndex[i];
    for(size_t j=0; j<il2.quadPatchIndex.size(); ++j){
      if(qp_idx == il2.quadPatchIndex[j])
        common_pid_vec.push_back(qp_idx);
    }
  }
  if(common_pid_vec.size() > 2){
    cerr <<"Warning: too many common-patch "<< il_index1 << " " <<il_index2 << endl;
  }
  if(common_pid_vec.size() == 0)
    return -1;
  return common_pid_vec[0];
}

pair<int,int> DualGenerator::getMaxMinPair(const QuadPatch& qp) const {
  int max_pid(-1), min_pid(-1);
  for(size_t i=0; i<qp.boundaryIntegrationLineIndex.size(); ++i){
    const IntegrationLine& il = il_vec[qp.boundaryIntegrationLineIndex[i]];
    const CriticalPoint& cp = cp_vec[il.endIndex];
    if(cp.type == MAXIMAL) max_pid = il.endIndex;
    else if(cp.type == MINIMAL) min_pid = il.endIndex;
  }
  return make_pair(max_pid, min_pid);
}

bool DualGenerator::checkInnerFaces(const PATH &loop,
                                    const std::vector<int> &faces) const{
  const HalfEdgeArray& he_vec = msc.mesh->getHalfEdgeArray();
  set<HalfEdgeHandle> bd_he_set;
  for(size_t i=0; i<loop.size()-1; ++i){
    int vid1 = loop[i], vid2 = loop[i+1];
    HalfEdgeHandle hh = msc.mesh->getHalfEdgeHandle(vid1, vid2);
    bd_he_set.insert(hh);
  }

  set<int> face_set(faces.begin(), faces.end());
  for(size_t i=0; i<loop.size()-1; ++i){
    HalfEdgeHandle curr_hh = msc.mesh->getHalfEdgeHandle(loop[i], loop[i+1]);
    const HalfEdge& curr_he = he_vec[curr_hh];
    HalfEdgeHandle oppo_hh = curr_he.oppo_he_handle;
    const HalfEdge& oppo_he = he_vec[oppo_hh];
    if(bd_he_set.find(curr_hh) != bd_he_set.end() &&
       bd_he_set.find(oppo_hh) != bd_he_set.end()) continue;
    int fh1 = curr_he.face_handle, fh2 = oppo_he.face_handle;
    if(face_set.find(fh1) != face_set.end() &&
       face_set.find(fh2) != face_set.end()){
      cerr << "Not valid face set for dual patch" << endl;
      return false;
    }
  }
  return true;
}

vector<int> DualGenerator::getDualPatchCP(const QuadPatch& dp) const{
  vector<int> cp_index_vec;
  bool flag = true;
  for(size_t i=0; i<dp.boundaryIntegrationLineIndex.size(); ++i){
    const IntegrationLine& il = dual_il_vec[dp.boundaryIntegrationLineIndex[i]];
    const CriticalPoint& cp1 = dual_cp_vec[il.startIndex];
    const CriticalPoint& cp2 = dual_cp_vec[il.endIndex];
    int cp_index = flag ? il.startIndex : il.endIndex;
    cp_index_vec.push_back(cp_index);
    flag = !flag;
  }
  return cp_index_vec;
}

bool DualGenerator::saveDualMSComplex(const std::string& file_name) const{
  ofstream fout(file_name.c_str());
  if(fout.fail()){
    cerr << "Can open file " << file_name << endl;
    return false;
  }
  fout << "# Critical Points : CP meshIndex type" << endl;
  for(size_t k=0; k<dual_cp_vec.size(); ++k){
    fout << "CP " << dual_cp_vec[k].meshIndex;
    if(dual_cp_vec[k].type == MINIMAL) fout << " MINIMAL" << endl;
    else if(dual_cp_vec[k].type == MAXIMAL) fout << " MAXIMAL" << endl;
  }

  fout << "# IntegrationLine Lines: cp_index_1, cp_index_2 path" << endl;
  for(size_t k=0; k<dual_il_vec.size(); ++k){
    fout << "IL " << dual_il_vec[k].startIndex << " " << dual_il_vec[k].endIndex << " ";
    const PATH& path = dual_il_vec[k].path;
    if(path.size() == 0){ fout << endl; continue; }
    for(size_t i=0; i<path.size()-1; ++i){
      fout << path[i];
      if((i+1)%10 == 0) fout << " \\\n";
      else fout << " ";
    }
    fout << path[path.size()-1] << endl;
  }

  fout << "# Patchs: " << endl;
  for(size_t k=0; k<dp_vec.size(); ++k){
    fout << "QP ";
    const QuadPatch& qp = dp_vec[k];
    const vector<int>& il_index_vec = qp.boundaryIntegrationLineIndex;
    if(il_index_vec.size() ==0) continue;
    for(size_t i=0; i<il_index_vec.size()-1; ++i){
      fout << il_index_vec[i] << " ";
    }
    fout << il_index_vec[il_index_vec.size()-1] << endl;
    if(qp.face.size() == 0) { fout << endl; continue; }
    for(size_t i=0; i<qp.face.size()-1; ++i){
      fout << qp.face[i];
      if((i+1)%10 == 0) fout << " \\\n";
      else fout << " ";
    }
    fout << qp.face[qp.face.size()-1] << endl;
  }
  return true;
}

bool DualGenerator::saveQuadFile(const std::string &file_name) const{
  ofstream fout(file_name.c_str());
  if(fout.fail()){
    cerr << "Cannot save to " << file_name << endl;
    return false;
  }
  fout << dual_cp_vec.size() << endl;
  for(size_t i=0; i<dual_cp_vec.size(); ++i){
    fout << dual_cp_vec[i].meshIndex;
    if(dual_cp_vec[i].type == MAXIMAL) fout <<" 0"<<endl;
    else fout << " 1" << endl;
  }
  fout << dual_il_vec.size() << endl;
  for(size_t i=0; i<dual_il_vec.size(); ++i){
    const PATH& path = dual_il_vec[i].path;
    size_t pn = path.size();
    fout << path[0] << " " << path[pn-1] << endl << pn << endl;
    for(size_t k=0; k<pn; ++k){
      fout << path[k];
      if((k+1)%10 == 0) fout << endl;
      else fout << " ";
    }
    fout << endl;
  }
  fout << dp_vec.size() << endl;
  for(size_t i=0; i<dp_vec.size(); ++i){
    const QuadPatch& dp = dp_vec[i];
    vector<int> cp_index_vec = getDualPatchCP(dp);
    for(size_t k=0; k<cp_index_vec.size(); ++k)
      fout << dual_cp_vec[cp_index_vec[k]].meshIndex << " ";
    fout << endl;
    for(size_t k=0; k<dp.boundaryIntegrationLineIndex.size(); ++k)
      fout << dp.boundaryIntegrationLineIndex[k] << " ";
    fout << endl;
    fout << dp.face.size() << endl;
    for(size_t k=0; k<dp.face.size(); ++k){
      fout << dp.face[k] << " ";
      if((k+1)%10 == 0) fout << endl;
      else fout << " ";
    }
    fout << endl;
  }
  return true;
}

}
