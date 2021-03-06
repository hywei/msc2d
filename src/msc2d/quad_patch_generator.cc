#include "quad_patch_generator.h"
#include "../mesh/Mesh.h"
#include "../util/utility.h"
#include <queue>
using namespace std;
using namespace meshlib;

namespace msc2d{
QPGenerator::QPGenerator(MSComplex2D& _msc): msc(_msc){}
QPGenerator::~QPGenerator(){}

void QPGenerator::genQuadPatch(){
  cout << "Generator Quad Patchs" << endl;
  formed_patchs.clear();
  face_patch_index_mp.clear();
  face_patch_index_mp.resize(msc.mesh->getFaceNumber(), -1);
  covered_edge_set.clear();
  tri_patch_il_index_vec.clear();
  tri_patch_cp_index_vec.clear();
  for(size_t i=0; i<msc.il_vec.size(); ++i){
    const PATH& path = msc.il_vec[i].path;
    for(int i=0; i<path.size()-1; ++i){
      EdgeHandle eid = msc.mesh->getEdgeHandle(path[i], path[i+1]);
      covered_edge_set.insert(eid);
    }
  }
  genMMSadMapping();
  for(size_t k=0; k<msc.cp_vec.size(); ++k){
    if(msc.cp_vec[k].type == SADDLE) genQuadPatch(k);
  }
  genTriPatch();
  cout << "Generator " << msc.qp_vec.size() << " quad patchs" << endl;
  genILNeighbor();
}

void QPGenerator::genMMSadMapping(){
  for(size_t k=0; k<msc.cp_vec.size(); ++k){
    const CriticalPoint& cp = msc.cp_vec[k];
    for(size_t i=0; i<cp.neighbor.size(); ++i){
      int cp_index1 = cp.neighbor[i].pointIndex;
      int cp_index2 = cp.neighbor[(i+1)%cp.neighbor.size()].pointIndex;
      if(msc.cp_vec[cp_index1].type != msc.cp_vec[cp_index2].type){
        pair<int, int> mm_pair = (msc.cp_vec[cp_index1].type == MAXIMAL) ?
              make_pair(cp_index1, cp_index2) : make_pair(cp_index2, cp_index1);
        mm_sad_mp[mm_pair].push_back(k);
      }
    }
  }
}

void QPGenerator::genQuadPatch(size_t sad_cp_idx){
  const CriticalPoint& cp = msc.cp_vec[sad_cp_idx];
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    int cp_index1 = cp.neighbor[k].pointIndex;
    int cp_index2 = cp.neighbor[(k+1)%cp.neighbor.size()].pointIndex;
    if(msc.cp_vec[cp_index1].type == msc.cp_vec[cp_index2].type) continue;

    const CriticalPointNeighbor& cp_nb1 = cp.neighbor[k];
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

        if(!findPatchInnerFace(patch)){
          cerr << "Cannot find inner faces for patch " << k << endl;
        }
      }
    }else{
      //! TODO boundary
      pair<int, int> mm_pair;
      if(msc.cp_vec[cp_index1].type == MAXIMAL) mm_pair = make_pair(cp_index1, cp_index2);
      else mm_pair = make_pair(cp_index2, cp_index1);
      assert(mm_sad_mp.find(mm_pair) != mm_sad_mp.end());
      const vector<int>& sad_index_vec = mm_sad_mp[mm_pair];
      if(sad_index_vec.size() !=1){
//        if(!msc.mesh->isBoundaryVertex(cp.meshIndex)){
//          cerr << "There are something mistake at sort critical neighbor! "
//               << __FILE__ << __LINE__ << endl;
//          cout << cp.meshIndex << " ";
//          for(size_t k=0; k<nb_vec.size(); ++k){
//            const CriticalPoint& _cp = msc.cp_vec[nb_vec[k].pointIndex];
//            cout << _cp.meshIndex << " ";
//          }
//          cout << "# " << k << endl;
//        }
      }else{
        int il_index1 = cp.neighbor[k].integrationLineIndex;
        int il_index2 = cp.neighbor[(k+1)%cp.neighbor.size()].integrationLineIndex;

        tri_patch_il_index_vec.push_back(make_pair(il_index1, il_index2));
        tri_patch_cp_index_vec.push_back(sad_cp_idx);
      }
    }
  }
}

void QPGenerator::genTriPatch() {
  for(size_t i=0; i<tri_patch_cp_index_vec.size(); ++i){
    size_t il_index1 = tri_patch_il_index_vec[i].first;
    size_t il_index2 = tri_patch_il_index_vec[i].second;
    const IntegrationLine& il1 = msc.il_vec[il_index1];
    const IntegrationLine& il2 = msc.il_vec[il_index2];
    size_t cp_index1 = il1.endIndex, cp_index2 = il2.endIndex;
    const PATH& path1 = msc.il_vec[il_index1].path;
    const PATH& path2 = msc.il_vec[il_index2].path;
    PATH boundary(path2.rbegin(), path2.rend());
    boundary.insert(boundary.end(), path1.begin()+1, path1.end());
    const HalfEdgeArray& he_vec = msc.mesh->getHalfEdgeArray();
    set<int> bd_hes, valid_edge_set;
    for(size_t i=0; i<boundary.size()-1; ++i){
      HalfEdgeHandle hh = msc.mesh->getHalfEdgeHandle(boundary[i], boundary[i+1]);
      bd_hes.insert(hh);
      valid_edge_set.insert(he_vec[hh].edge_handle);
    }
    set<int> visited_faces;

    for(size_t i=0; i<boundary.size()-1; ++i){
      int vid1 = boundary[i], vid2 = boundary[i+1];
      HalfEdgeHandle hh = msc.mesh->getHalfEdgeHandle(vid1, vid2);
      HalfEdgeHandle oppo_hh = he_vec[hh].oppo_he_handle;
      if(bd_hes.find(hh) != bd_hes.end() && bd_hes.find(oppo_hh) != bd_hes.end()) continue;
      int fid = he_vec[hh].face_handle;
      if(fid == -1) continue;
      if(visited_faces.find(fid) == visited_faces.end() && face_patch_index_mp[fid] == -1) {
        queue <int> q; q.push(fid); visited_faces.insert(fid);

        while(!q.empty()){
          int _fid = q.front(); q.pop();
          const EdgeHandleArray& eh_vec = msc.mesh->getFaceEdges(_fid);
          for(size_t i=0; i<eh_vec.size(); ++i) valid_edge_set.insert(eh_vec[i]);
          const HalfEdgeHandleArray& hh_vec = msc.mesh->getFaceHalfEdges(_fid);
          for(size_t i=0; i<hh_vec.size(); ++i){
            if(bd_hes.find(hh_vec[i]) != bd_hes.end()) continue;
            int eid = he_vec[hh_vec[i]].edge_handle;
            if(covered_edge_set.find(eid) != covered_edge_set.end()) continue;
            int oppo_hh = he_vec[hh_vec[i]].oppo_he_handle;
            int fh = he_vec[oppo_hh].face_handle;
            if(fh!= -1 && visited_faces.find(fh) == visited_faces.end() &&
               face_patch_index_mp[fh] == -1){
              q.push(fh); visited_faces.insert(fh);
            }
          }
        }
      }
    }
    IntegrationLine dual_il;
    dual_il.startIndex = cp_index1; dual_il.endIndex = cp_index2;
    int start_vid = msc.cp_vec[cp_index1].meshIndex;
    int end_vid = msc.cp_vec[cp_index2].meshIndex;
    if(msc.mesh->getShortestPath(start_vid, end_vid, dual_il.path, valid_edge_set)){
        msc.il_vec.push_back(dual_il);
        msc.qp_vec.push_back(QuadPatch());
        QuadPatch& patch = msc.qp_vec[msc.qp_vec.size()-1];
        patch.boundaryIntegrationLineIndex.push_back(il_index1);
        patch.boundaryIntegrationLineIndex.push_back(msc.il_vec.size()-1);
        patch.boundaryIntegrationLineIndex.push_back(il_index2);
        if(!findPatchInnerFace(patch)){
          cerr <<"Cannot find inner face of patch " << msc.qp_vec.size() << endl;
        }
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
  return _nb_vec[(idx+_nb_num-1)%_nb_num];
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
        bd_loop.insert(bd_loop.end(), path.begin()+1, path.end());
      else if(last_vid == prev_vid)
        bd_loop.insert(bd_loop.end(), path.rbegin()+1, path.rend());
      else return false;
    }
  }
  assert(bd_loop[bd_loop.size()-1] == bd_loop[0]);

  for(size_t i=0; i<bd_loop.size()-1; ++i){
    int vid1 = bd_loop[i], vid2 = bd_loop[i+1];
    int eid = msc.mesh->getEdgeHandle(vid1, vid2);
    covered_edge_set.insert(eid);
  }

  bool flag = getInnerFaces(bd_loop, patch.face);
  if(flag){
    for(size_t i=0; i<patch.face.size(); ++i){
      int fid = patch.face[i];
      if(face_patch_index_mp[fid] != -1){
        cerr <<"Warning: face " << fid << " is assigned to more than one patch" << endl;
      }
      const EdgeHandleArray& eh_vec = msc.mesh->getFaceEdges(fid);
      for(size_t i=0; i<eh_vec.size(); ++i) {
        covered_edge_set.insert(eh_vec[i]);
      }
    }
  }
  return flag;
}

bool QPGenerator::getInnerFaces(const PATH &loop, std::vector<int>& fh_vec) const{
  fh_vec.clear();
  if(loop.size()<3 || loop[0] != loop[loop.size()-1]) return false;
  const VertArray& vert_vec = msc.mesh->getVertexArray();
  const FaceArray& face_vec = msc.mesh->getFaceArray();
  const HalfEdgeArray& he_vec = msc.mesh->getHalfEdgeArray();
  for(size_t k=0; k<loop.size(); ++k) {
    const Vert& vert = vert_vec[loop[k]];
    if(Util::IsSetFlag(vert.flag, NONMANIFOLD_VERT)) return false;
  }

  set<HalfEdgeHandle> bd_edge_set;
  for(size_t k=0; k<loop.size()-1; ++k) {
    HalfEdgeHandle hh = msc.mesh->getHalfEdgeHandle(loop[k], loop[k+1]);
    if(hh == -1){
      cerr << "Not a close loop" << endl;
      return false;
    }
    bd_edge_set.insert(hh);
  }

  set<FaceHandle> faces;
  queue<FaceHandle> q;

  for(size_t k=0; k<loop.size()-1; ++k){
    HalfEdgeHandle hh = msc.mesh->getHalfEdgeHandle(loop[k], loop[k+1]);
    HalfEdgeHandle oppo_hh = he_vec[hh].oppo_he_handle;
    if(bd_edge_set.find(hh) != bd_edge_set.end() &&
       bd_edge_set.find(oppo_hh) != bd_edge_set.end()) continue;
    const HalfEdge& he = he_vec[hh];
    FaceHandle fh = he.face_handle;
    if(fh == -1) return false;
    if(faces.find(fh) == faces.end()){
      q.push(fh); faces.insert(fh);
      while(!q.empty()){
        FaceHandle fh = q.front(); q.pop();
        if(face_patch_index_mp[fh] != -1) return false;
        const HalfEdgeHandleArray& hh_vec = face_vec[fh].he_handle_vec;
        for(size_t k=0; k<hh_vec.size(); ++k){
          if(bd_edge_set.find(hh_vec[k]) != bd_edge_set.end()) continue;
          HalfEdgeHandle oppo_hh = he_vec[hh_vec[k]].oppo_he_handle;
          FaceHandle fh = he_vec[oppo_hh].face_handle;
          if(fh != -1 && faces.find(fh) == faces.end()) {
            faces.insert(fh);
            q.push(fh);
          }
        }
      }// end while
    }
  }

  // check valid
  for(size_t k=0; k<loop.size()-1; ++k) {
    HalfEdgeHandle hh = msc.mesh->getHalfEdgeHandle(loop[k], loop[k+1]);
    HalfEdgeHandle oppo_hh = he_vec[hh].oppo_he_handle;
    if(bd_edge_set.find(hh) != bd_edge_set.end() &&
       bd_edge_set.find(oppo_hh) != bd_edge_set.end()) continue;
    int fh1 = he_vec[hh].face_handle;
    int fh2 = he_vec[oppo_hh].face_handle;
    if(faces.find(fh1) != faces.end() && faces.find(fh2) != faces.end()){
      cerr << "Not a valid face set" << endl;
      return false;
    }
  }

  fh_vec.clear(); fh_vec.resize(faces.size());
  fh_vec.assign(faces.begin(), faces.end());
  return true;
}

void QPGenerator::genILNeighbor(){
  for(size_t i=0; i<msc.qp_vec.size(); ++i){
    const QuadPatch& p = msc.qp_vec[i];
    for(size_t k=0; k<p.boundaryIntegrationLineIndex.size(); ++k){
      IntegrationLine& il = msc.il_vec[p.boundaryIntegrationLineIndex[k]];
      il.quadPatchIndex.push_back(i);
    }
  }
  for(size_t i=0; i<msc.il_vec.size(); ++i){
    const IntegrationLine& il = msc.il_vec[i];
    if(il.quadPatchIndex.size() > 2) {
      cerr <<"Warning: too many neighbor patchs for il " << i << endl;
    }
  }
}

} // end namespace 
