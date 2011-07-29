#include "integration_line_tracer.h"
#include "mscomplex.h"
#include "../mesh/Mesh.h"
#include "../util/utility.h"
#include <stack>

using namespace std;
using namespace meshlib;
namespace msc2d{

ILTracer::ILTracer(MSComplex2D& _msc): msc(_msc), mesh(*_msc.mesh){}
ILTracer::~ILTracer(){}

size_t ILTracer::next(int vid, size_t curr_index) const{
  size_t adj_num = mesh.getAdjVertices(vid).size();
  if(mesh.isBoundaryVertex(vid) && curr_index == adj_num-1)
    return adj_num;
  return (curr_index+1)%adj_num;
}

size_t ILTracer::prev(int vid, size_t curr_index) const{
  size_t adj_num = mesh.getAdjVertices(vid).size();
  if(mesh.isBoundaryVertex(vid) && curr_index == 0)
    return adj_num;
  return (curr_index+adj_num-1)%adj_num;
}

bool ILTracer::traceIntegrationLine(){
  if(!createWEdge()) return false;
  if(!traceAscendingPath()) return false;
  setAscendingPathData();
  if(!traceDescendingPath()) return false;
  genCPNeighbor();
  unfoldMultiSaddle();
  return true;
}

bool ILTracer::createWEdge(){
  wedge_vec.clear(); wedge_vec.resize(mesh.getVertexNumber());
    
  for(size_t vid = 0; vid<mesh.getVertexNumber(); ++vid){
    WEdge& we = wedge_vec[vid];
    const VertHandleArray& adj_vertices = mesh.getAdjVertices(vid);
    size_t n = adj_vertices.size();
    if(n==0) continue;
    if(msc.getVertexType(vid) == MAXIMAL){
      we.max_ranges.push_back(make_pair(0, n)); continue;
    }else if(msc.getVertexType(vid) == MINIMAL){
      we.min_ranges.push_back(make_pair(0, n)); continue;
    }

    size_t start_maxp = n;    
    //! get the first max-range's start position
    for(size_t i=0; i<n; ++i){
      if(msc.cmpScalarValue(vid, adj_vertices[i]) == -1){
        if( mesh.isBoundaryVertex(vid) && i==0){
          start_maxp = 0; break;
        }else{
          if(msc.cmpScalarValue(vid, adj_vertices[prev(vid, i)]) == 1){
            start_maxp = i; break;
          }
        }
      }
    }    
    assert(start_maxp != n);

    size_t start_idx(start_maxp);
    for(size_t i=next(vid, start_maxp), j=0; j<n; i=next(vid, i), ++j){
      if(mesh.isBoundaryVertex(vid) && i==n){ //! handle boundary
        if(msc.cmpScalarValue(vid, adj_vertices[i-1]) == -1){
          we.max_ranges.push_back(make_pair(start_idx, n)); start_idx = 0;
        }else if(msc.cmpScalarValue(vid, adj_vertices[i-1]) == 1){
          we.min_ranges.push_back(make_pair(start_idx, n)); start_idx = 0;
        }
      }else{ //! general case
        if(msc.cmpScalarValue(vid, adj_vertices[i]) == -1 &&
           msc.cmpScalarValue(vid, adj_vertices[prev(vid, i)]) ==1){
          we.min_ranges.push_back(make_pair(start_idx, i)); start_idx = i;
        }else if(msc.cmpScalarValue(vid, adj_vertices[i]) == 1 &&
                 msc.cmpScalarValue(vid, adj_vertices[prev(vid,i)]) == -1){
          we.max_ranges.push_back(make_pair(start_idx, i)); start_idx = i;
        }
      }
    }    
    if(!mesh.isBoundaryVertex(vid))
      assert(we.max_ranges.size() == we.min_ranges.size());
  }

  return true;
}

bool ILTracer::traceAscendingPath(){
  cout << "Trace ascending path" << endl;
  for(vector<CriticalPoint>::iterator it = msc.cp_vec.begin(); it != msc.cp_vec.end(); ++it){
    if(it->type == SADDLE) {
      CriticalPoint& sad = *it;
      const vector<pair<size_t, size_t> >& max_ranges = wedge_vec[sad.meshIndex].max_ranges;
      for(size_t k=0; k<max_ranges.size(); ++k){
        int prev_vid = -1, curr_vid = sad.meshIndex;
        msc.il_vec.push_back(IntegrationLine());        
        IntegrationLine& il=msc.il_vec[msc.il_vec.size()-1];
        PATH& mesh_path = il.path; mesh_path.push_back(curr_vid);
        do{
          pair<int, int> range;// get right range to trace gradient
          if(mesh_path.size()==1) range = max_ranges[k];
          else{
            if(msc.getVertexType(curr_vid) == SADDLE){ // select the previous adjacent maxrange              
              int range_idx = getRangeIndex(curr_vid, prev_vid);              
              if(wedge_vec[curr_vid].max_ranges.size() <= range_idx){//at boundary 
                --range_idx;
                error_rule_vec.push_back(make_pair(curr_vid, range_idx+1));
              }
              range = wedge_vec[curr_vid].max_ranges[range_idx];
            }else range = wedge_vec[curr_vid].max_ranges[0];
          }
          prev_vid = curr_vid;
          curr_vid = getGradDirection(curr_vid, range);
          assert(curr_vid != -1);
          mesh_path.push_back(curr_vid);
          assert(mesh_path.size() < mesh.getEdgeNumber());
        }while(msc.getVertexType(curr_vid) != MAXIMAL);
        il.startIndex = msc.vert_cp_index_mp[it->meshIndex];
        il.endIndex = msc.vert_cp_index_mp[curr_vid];        
      }
    }
  }
  return true;
}

void ILTracer::setAscendingPathData(){
  //! set junction flag , in/out vertices info and vert-path mapping
  size_t vert_num = mesh.getVertexNumber();
  junction_flag.clear(); junction_flag.resize(vert_num);
  in_vertices.clear(); in_vertices.resize(vert_num);
  out_vertices.clear(); out_vertices.resize(vert_num);
  edge_path_mp.clear();
  for(size_t k=0; k<msc.il_vec.size(); ++k){
    const PATH& path = msc.il_vec[k].path;
    for(size_t i=0; i<path.size(); ++i){
      int vid = path[i];
      if(msc.getVertexType(vid) == REGULAR) junction_flag[vid] = true;
      if(i!=0) in_vertices[vid].push_back(path[i-1]);
      if(i!=path.size()-1) out_vertices[vid].push_back(path[i+1]);
      if(i>0) edge_path_mp[make_pair(path[i-1], vid)].push_back(k);
    }    
  }
}


int ILTracer::getGradDirection(int vid, const pair<size_t, size_t>& range) const{
  const VertHandleArray& adj_vertices = mesh.getAdjVertices(vid);
  double grad = -1.0;
  int grad_dir = -1;
  for(size_t i=range.first; i!=range.second; i=next(vid, i)){
    double cur_grad = fabs(msc.calGradient(vid, adj_vertices[i]));
    if(cur_grad > grad) { grad = cur_grad; grad_dir = adj_vertices[i];}
  }
  return grad_dir; 
}

int ILTracer::getRangeIndex(int vid, int adj_vid) const{
  const vector<int>& adj_vertices = mesh.getAdjVertices(vid);
  if(!Util::isIn(adj_vertices, adj_vid)) return -1;
  const WEdge& we = wedge_vec[vid];
  vector<pair<size_t, size_t> >  ranges;
  if(msc.cmpScalarValue(vid, adj_vid) == 1) ranges = we.min_ranges;
  else ranges = we.max_ranges;
  for(size_t k=0; k<ranges.size(); ++k){
    for(size_t j=ranges[k].first; j!=ranges[k].second; j=next(vid, j)){
      if(adj_vertices[j] == adj_vid) return k;
    }
  }
  return -1;
}


bool ILTracer::traceDescendingPath(){
  cout << "Trace descending path" << endl;
  for(vector<CriticalPoint>::iterator it=msc.cp_vec.begin(); it!=msc.cp_vec.end(); ++it){
    if(it->type == SADDLE){
      const vector<pair<size_t, size_t> >& min_ranges = wedge_vec[it->meshIndex].min_ranges;
      for(size_t k=0; k<min_ranges.size(); ++k){
        path_side_record.clear();
        int prev_vid = it->meshIndex;
        int curr_vid = getDescendingPathSecondVert(*it, k);

        IntegrationLine il;
        PATH& path = il.path;
        path.push_back(prev_vid); path.push_back(curr_vid);
        while(msc.getVertexType(curr_vid) != MINIMAL){
          pair<int, int> range;
          if(msc.getVertexType(curr_vid) == SADDLE){
            range = getMinRangeAtSaddle(curr_vid, prev_vid);
          }else if(msc.getVertexType(curr_vid) == REGULAR){
            if(junction_flag[curr_vid] == true){
              range = getMinRangeAtJunction(curr_vid, prev_vid);
            }else range = wedge_vec[curr_vid].min_ranges[0];
          }
          if(range.first == -1 && range.second == -1) break;
          prev_vid = curr_vid;
          curr_vid = getGradDirection(curr_vid, range);
          assert(curr_vid != -1);
          path.push_back(curr_vid);
          assert(path.size() < mesh.getEdgeNumber());
        }
        if(msc.getVertexType(curr_vid) != MINIMAL) continue;
        il.startIndex = msc.vert_cp_index_mp[it->meshIndex];
        il.endIndex = msc.vert_cp_index_mp[curr_vid];
        msc.il_vec.push_back(il);
      }
    }
  }
  return true;
}

int ILTracer::getDescendingPathSecondVert(const CriticalPoint& cp, int range_index){
  const VertHandleArray& adj_vertices = mesh.getAdjVertices(cp.meshIndex);
  const pair<size_t, size_t>& min_r = wedge_vec[cp.meshIndex].min_ranges[range_index];
  pair<size_t, size_t> range= min_r;
  int last_adj_vid(-1);
  bool err_rule = Util::isIn(error_rule_vec, make_pair(cp.meshIndex, range_index));
  for(size_t k=min_r.first; k!=min_r.second; k=next(cp.meshIndex, k)){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_vertices[cp.meshIndex], adj_vid)){
      last_adj_vid = adj_vid;
      if(!err_rule) {range.first = k; }
      else { range.second = next(cp.meshIndex,k); break;}
    }
  }
  if(last_adj_vid !=-1){
    const vector<size_t>& paths = edge_path_mp[make_pair(last_adj_vid, cp.meshIndex)];
    for(size_t k=0; k<paths.size(); ++k){
      if(!err_rule) path_side_record[paths[k]] = 1; // back direction
      else path_side_record[paths[k]] = -1;
    }
  }
  return getGradDirection(cp.meshIndex, range);
}

pair<size_t, size_t> ILTracer::getMinRangeAtJunction(int curr_vid, int prev_vid){
  const VertHandleArray& adj_vertices = mesh.getAdjVertices(curr_vid);
  const vector<int>& in_verts = in_vertices[curr_vid];
  const vector<int>& out_verts = out_vertices[curr_vid];
  const pair<size_t, size_t>& max_r = wedge_vec[curr_vid].max_ranges[0];
  const pair<size_t, size_t>& min_r = wedge_vec[curr_vid].min_ranges[0];
  bool visited_flag = false;
  for(size_t k=max_r.first; k!=max_r.second; k=next(curr_vid, k)){
    int adj_vid = adj_vertices[k];
    if(adj_vid == prev_vid) visited_flag = true;
    if(Util::isIn(out_verts, adj_vid)){
      if(adj_vid == prev_vid) continue; //! side have set before
      int side = 0;
      if(visited_flag == true) side = 1;
      else side = -1;
      //! set path side flag
      const vector<size_t>& path_ids = edge_path_mp[make_pair(curr_vid, adj_vid)];
      for(size_t i=0; i<path_ids.size(); ++i){
        path_side_record[path_ids[i]] = side;
      }
    }
  }
  size_t first = min_r.first, second = min_r.second;
  for(size_t k=min_r.first; k!=min_r.second; k=next(curr_vid, k)){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_verts, adj_vid)){
      const vector<size_t>& path_ids = edge_path_mp[make_pair(adj_vid, curr_vid)];
      assert(path_ids.size() !=0);
      map<size_t, int>::const_iterator im = path_side_record.find(path_ids[0]);
      assert(im != path_side_record.end());
      if(im->second == -1) { second = next(curr_vid, k); break;}
      else first = k;
    }
  }
  return make_pair(first, second);
}

pair<size_t, size_t> ILTracer::getMinRangeAtSaddle(int sadd_vid, int prev_vid) {
  const VertHandleArray& adj_vertices = mesh.getAdjVertices(sadd_vid);
  const vector<int>& in_verts = in_vertices[sadd_vid];
  const vector<int>& out_verts = out_vertices[sadd_vid];
  int side = 0;
  int max_range_idx = getRangeIndex(sadd_vid, prev_vid);
  assert(max_range_idx != -1);
  const pair<size_t, size_t>& max_range = wedge_vec[sadd_vid].max_ranges[max_range_idx];
  for(size_t k=max_range.first; k!=max_range.second; k=next(sadd_vid, k)){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(out_verts, adj_vid)){
      if(adj_vid != prev_vid) side = -1;
      else{
        const vector<size_t>& path_ids = edge_path_mp[make_pair(sadd_vid, adj_vid)];
        assert(path_ids.size() > 0);
        side = path_side_record[path_ids[0]];
      }
    }else if(adj_vid == prev_vid){ side=1; }
    if(side !=0) break;
  }
  assert(side != 0);
  int min_range_idx(-1);
  size_t max_range_num = wedge_vec[sadd_vid].max_ranges.size();
  size_t min_range_num = wedge_vec[sadd_vid].min_ranges.size();
  if(side == -1) {
    if(msc.mesh->isBoundaryVertex(sadd_vid) && max_range.second == adj_vertices.size()) return make_pair(-1, -1);
    int _vid = adj_vertices[max_range.second];
    min_range_idx = getRangeIndex(sadd_vid, _vid);
  }else{
    if(max_range.first == 0 && msc.mesh->isBoundaryVertex(sadd_vid)) return make_pair(-1, -1);
    int _vid = adj_vertices[prev(sadd_vid, max_range.first)];
    min_range_idx = getRangeIndex(sadd_vid, _vid);
  }
  assert(min_range_idx != -1);
  const pair<size_t, size_t>& min_range = wedge_vec[sadd_vid].min_ranges[min_range_idx];
  size_t first = min_range.first, second = min_range.second;
  for(size_t k=min_range.first; k!=min_range.second; k=next(sadd_vid, k)){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_verts, adj_vid)){
      if(side == -1){ second = next(sadd_vid, k); break;}
      else if(side == 1) { first = k; }
    }
  }
  return make_pair(first, second);
}

void ILTracer::genCPNeighbor(){
  for(size_t k=0; k<msc.il_vec.size(); ++k){
    const IntegrationLine& il = msc.il_vec[k];
    CriticalPoint& cp1 = msc.cp_vec[il.startIndex];
    CriticalPoint& cp2 = msc.cp_vec[il.endIndex];
    CriticalPointNeighbor cp_nb1, cp_nb2;
    cp_nb1.pointIndex = il.startIndex; cp_nb1.integrationLineIndex = k;
    cp_nb2.pointIndex = il.endIndex; cp_nb2.integrationLineIndex = k;
    cp1.neighbor.push_back(cp_nb2);
    cp2.neighbor.push_back(cp_nb1);
  }
  for(size_t k=0; k<msc.cp_vec.size(); ++k) sortCPNeighbor(msc.cp_vec[k]);
}

void ILTracer::sortCPNeighbor(CriticalPoint& cp) const{
  vector<CriticalPointNeighbor> nb_bak = cp.neighbor;
  if(cp.type == SADDLE){
    cp.neighbor.clear();

    const WEdge& we = wedge_vec[cp.meshIndex];
    const VertHandleArray& adj_vertices = mesh.getAdjVertices(cp.meshIndex);
    if(msc.mesh->isBoundaryVertex(cp.meshIndex)){
      for(size_t k=0; k<adj_vertices.size(); ++k){
        for(size_t i=0; i<nb_bak.size(); ++i){
          const IntegrationLine& il = msc.il_vec[nb_bak[i].integrationLineIndex];
          if(il.path[1] == adj_vertices[k]) cp.neighbor.push_back(nb_bak[i]);
        }
      }
    }else{
      bool flag = true;
      pair<size_t, size_t> range;
      for(size_t i=0; i<we.max_ranges.size()+we.min_ranges.size(); ++i, flag = !flag){
        range = flag ? we.max_ranges[i/2] : we.min_ranges[i/2];
        for(size_t k=range.first; k!=range.second; k=next(cp.meshIndex, k)){
          int adj_vid = adj_vertices[k];
          for(size_t j=0; j<nb_bak.size(); ++j){
            const IntegrationLine& il = msc.il_vec[nb_bak[j].integrationLineIndex];
            if(adj_vid == il.path[1]) cp.neighbor.push_back(nb_bak[j]);
          }
        }
      }
    }
  }else{
    Tree tree;
    vector<int> il_index_vec;
    makeTree(cp, tree);  
    traverseTree(tree, il_index_vec);
    cp.neighbor.clear();
    for(int k=il_index_vec.size()-1; k>=0; --k)
      cp.neighbor.push_back(nb_bak[il_index_vec[k]]);
  }

  if(mesh.isBoundaryVertex(cp.meshIndex)){
    // boundary cp's frist neighbor should also be a boundary
    const VertHandleArray& adj_vertices = mesh.getAdjVertices(cp.meshIndex);
    CriticalPointNeighborArray _nb = cp.neighbor;
    int first_index = -1;
    for(size_t i=0; i<adj_vertices.size(); ++i){
      int vid = adj_vertices[i];
      for(size_t k=0; k<_nb.size(); ++k){
        int first_vid;
        const IntegrationLine& il = msc.il_vec[_nb[k].integrationLineIndex];
        if(msc.cp_vec[il.startIndex].meshIndex == cp.meshIndex) first_vid = il.path[1];
        else first_vid = il.path[il.path.size()-2];
        if(first_vid == vid){ first_index = k; break; }
      }
      if(first_index != -1) break;
    }
    for(size_t k=0; k<cp.neighbor.size(); ++k)
      cp.neighbor[k] = _nb[(first_index+k)%_nb.size()];
  }
  
  if(cp.neighbor.size() != nb_bak.size()){
    cout << cp.neighbor.size() << " " << nb_bak.size() << " ## ";
    cout << cp.meshIndex << " " << msc.vert_cp_index_mp[cp.meshIndex];
    if(cp.type == SADDLE) cout << " SAD" << endl;
    else if(cp.type == MAXIMAL) cout << " MAX" << endl;
    else cout << " MIN"<<endl;
    cout << "Missed integration lines: ";
    for(size_t k=0; k<nb_bak.size(); ++k){
      if(!Util::isIn(cp.neighbor, nb_bak[k])) cout << nb_bak[k].integrationLineIndex << " ";
    }
    cout << endl;
  }
 // assert(cp.neighbor.size() == nb_bak.size());
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    if(!Util::isIn(nb_bak, cp.neighbor[k])) {
      cerr <<"Warning: there are something wrong on sort critial point neighbor" << endl;
    }
  }
//  if(cp.meshIndex == 5089){
//    cout << cp.meshIndex <<": ";
//    cout << "\t CP_Vert: ";
//    for(size_t i=0; i<cp.neighbor.size(); ++i){
//      const CriticalPoint& _cp = msc.cp_vec[cp.neighbor[i].pointIndex];
//      cout << _cp.meshIndex << " ";
//    }
//    cout << endl << "\t IL_Index: ";
//    for(size_t i=0; i<cp.neighbor.size(); ++i){
//      cout << cp.neighbor[i].integrationLineIndex<<" ";
//    }
//    cout << endl;
//  }
}

void ILTracer::splitSaddle(vector<PATH>& path_vec, Tree& t) const{
  for(size_t k=0; k<path_vec.size(); ++k){
    PATH& path = path_vec[k];
    int leaf = path[0];
    if(Util::isIn(t.leaves, leaf)){
      leaf = t.hash++; // split this saddle with a visual node
      t.node_vert_mp[leaf] =  path[0]; // make node-vert mapping
      //! update other paths which pass this vertex
      int curr_vid = path[0], next_vid = path[1];
      for(size_t j=0; j<path_vec.size(); ++j){
        PATH& _path = path_vec[j];
        for(size_t i=1; i<_path.size(); ++i)
          if(_path[i-1] == curr_vid && _path[i] == next_vid){
            _path[i-1]  = leaf;        
          }
      }
    }
    t.leaves.push_back(leaf);
    t.node_path_mp[leaf] = k; // make node-il mapping
  }
}

void ILTracer::replaceSaddle(vector<PATH>& path_vec, Tree& t) const{
  for(size_t k=0; k<path_vec.size(); ++k){
    PATH& path = path_vec[k];
    for(size_t i=1; i<path.size()-1; ++i){
      if(Util::isIn(t.leaves, path[i])){ // a saddle
        int curr_vid = path[i], next_vid = path[i+1];
        if(t.node_vert_mp.find(path[i]) != t.node_vert_mp.end())
          curr_vid = t.node_vert_mp[path[i]];
        if(t.node_vert_mp.find(path[i+1]) != t.node_vert_mp.end())
          next_vid = t.node_vert_mp[path[i+1]];
        assert(msc.getVertexType(curr_vid) == SADDLE);
        int visual_node = t.hash++;
        int mapping_vert(-1);
        //! get the mapping vertex index
        bool ascending = (msc.cmpScalarValue(curr_vid, next_vid) == 1) ? false : true;
        const WEdge& we = wedge_vec[curr_vid];
        const VertHandleArray& adj_vertices = mesh.getAdjVertices(curr_vid);
        int r_index = getRangeIndex(curr_vid, next_vid);
        if(ascending){
          int next_r_index = (r_index+1)%we.max_ranges.size();
          if(next_r_index == r_index){//! at boundary
            //mapping_vert = adj_vertices[we.min_ranges[r_index].second];
            mapping_vert = curr_vid;
          }
          else
            mapping_vert = adj_vertices[we.max_ranges[next_r_index].first];          
        }else{
          int next_r_index = (r_index+1)%we.min_ranges.size();
          if(next_r_index == r_index){
            //mapping_vert = adj_vertices[we.max_ranges[r_index].second];
            mapping_vert = curr_vid;
          }
          else
            mapping_vert = adj_vertices[we.min_ranges[next_r_index].first];
        }
        t.node_vert_mp[visual_node] = mapping_vert;
        t.node_path_mp[visual_node] = t.node_path_mp[path[i]];
        //! remove the real leaf
        size_t idx = distance(t.leaves.begin(),
                              find(t.leaves.begin(), t.leaves.end(), path[i]));
        t.leaves[idx] = visual_node;
        //! update the path
        PATH& _path = path_vec[t.node_path_mp[path[i]]];
        _path.insert(_path.begin(), visual_node);
        t.node_path_mp.erase(t.node_path_mp.find(path[i]));
      }
    }
  }
}

void ILTracer::makeTree(const CriticalPoint& cp, Tree& t) const{
  vector<PATH> path_vec;
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    size_t il_index = cp.neighbor[k].integrationLineIndex;
    path_vec.push_back(msc.il_vec[il_index].path);    
  }
  t.hash = mesh.getVertexNumber();
  //! resolve two special cases
  splitSaddle(path_vec, t);
  replaceSaddle(path_vec, t);

  //! create tree
  t.root = cp.meshIndex;
  t.parent[t.root] = -1;
  for(size_t k=0; k<path_vec.size(); ++k){
    const PATH& path = path_vec[k];
    for(size_t i=1; i<path.size(); ++i){
      int curr_vid = path[i-1];
      int next_vid = path[i];
      assert(t.parent.find(curr_vid) == t.parent.end() ||
             t.parent.find(curr_vid)->second == next_vid);
      t.parent[curr_vid] = next_vid;
      vector<int>& children = t.children[next_vid];
      if(!Util::isIn(children, curr_vid)) children.push_back(curr_vid);
    }
  }
}

void ILTracer::traverseTree(Tree& tree, vector<int>& il_index_vec) const{
  stack<int> st;
  st.push(tree.root);
  map<int, int>::iterator im;
  il_index_vec.clear();
  while(!st.empty()){
    int node = st.top(); st.pop();
    if(Util::isIn(tree.leaves, node)){
      il_index_vec.push_back(tree.node_path_mp[node]);
      continue;
    }
    const vector<int>& subnodes = tree.children[node];
    vector<int> children = subnodes;
    for(size_t k=0; k<children.size(); ++k){
      int child = children[k];
      im = tree.node_vert_mp.find(child);
      if(im != tree.node_vert_mp.end()) children[k] = im->second;
    }
    int parent = tree.parent[node];
    im = tree.node_vert_mp.find(parent);
    if(im != tree.node_vert_mp.end()) parent = im->second;
    int vid = node;
    im = tree.node_vert_mp.find(node);
    if(im != tree.node_vert_mp.end())  vid = im->second;
    const VertHandleArray& adj_vertices = mesh.getAdjVertices(vid);
    size_t out_index;
    if(parent == -1) out_index = 0;
    else out_index = distance(adj_vertices.begin(),
                              find(adj_vertices.begin(), adj_vertices.end(), parent));
    size_t i=out_index, idx;
    for(; i<adj_vertices.size(); ++i){
      idx = distance(children.begin(), find(children.begin(), children.end(), adj_vertices[i]));
      if(idx != children.size()) st.push(subnodes[idx]);
    }
    // speical case for boundary saddle
    idx = distance(children.begin(), find(children.begin(), children.end(), vid));
    if(idx != children.size()) st.push(subnodes[idx]);
    for(i=0; i!=out_index; ++i){
      idx = distance(children.begin(), find(children.begin(), children.end(), adj_vertices[i]));
      if(idx != children.size()) st.push(subnodes[idx]);
    }

  } 
}

void ILTracer::unfoldMultiSaddle(){
  for(size_t k=0; k<msc.cp_vec.size(); ++k){
    if(msc.cp_vec[k].type == SADDLE && !isNormalSaddle(msc.cp_vec[k])){
      cout << "unfold multi-saddle " << msc.cp_vec[k].meshIndex << endl;
      unfoldMultiSaddle(msc.cp_vec[k]);
    }
  }
}

void ILTracer::unfoldMultiSaddle(CriticalPoint& cp){
  bool bd_flag = mesh.isBoundaryVertex(cp.meshIndex);
  while(!isNormalSaddle(cp)){
    CriticalPoint new_cp;
    new_cp.meshIndex = cp.meshIndex; new_cp.type = SADDLE;
    IntegrationLine new_il1, new_il2;
    int new_cp_idx = msc.cp_vec.size() ;
    int new_il1_idx(msc.il_vec.size());
    int new_il2_idx(msc.il_vec.size()+1);
    CriticalPointNeighbor new_nb1, new_nb2;
    
    new_il1 = msc.il_vec[cp.neighbor[0].integrationLineIndex];
    new_il1.startIndex = new_cp_idx;    
    new_nb1.pointIndex = cp.neighbor[0].pointIndex;
    new_nb1.integrationLineIndex = new_il1_idx;    
     
    // set new saddle's neighbor
    new_cp.neighbor.push_back(new_nb1);
    new_cp.neighbor.push_back(cp.neighbor[1]);
    // update two max/min points neighbor
    int nb_idx1, nb_idx2;
    CriticalPoint& m1 = msc.cp_vec[cp.neighbor[0].pointIndex];
    nb_idx1 = getNeighborIndex(m1, cp.neighbor[0].integrationLineIndex);
    assert(nb_idx1 != -1);
    CriticalPointNeighbor new_nb3;
    new_nb3.pointIndex = new_cp_idx;
    new_nb3.integrationLineIndex = new_il1_idx;
    m1.neighbor.insert(m1.neighbor.begin()+nb_idx1, new_nb3);
    msc.il_vec.push_back(new_il1);
    // update il
    IntegrationLine& il1 = msc.il_vec[cp.neighbor[1].integrationLineIndex];
    il1.startIndex = new_cp_idx;
    CriticalPoint& m3 = msc.cp_vec[cp.neighbor[1].pointIndex];
    int nb_idx3 = getNeighborIndex(m3, cp.neighbor[1].integrationLineIndex);
    m3.neighbor[nb_idx3].pointIndex = new_cp_idx;
    
    if(!bd_flag){ // non-boundary saddle
      new_il2 = msc.il_vec[cp.neighbor[3].integrationLineIndex];
      new_il2.startIndex = new_cp_idx;
      new_nb2.pointIndex = cp.neighbor[3].pointIndex;
      new_nb2.integrationLineIndex = new_il2_idx;
      new_cp.neighbor.push_back(cp.neighbor[2]);
      new_cp.neighbor.push_back(new_nb2);
      CriticalPoint& m2 = msc.cp_vec[cp.neighbor[3].pointIndex];
      nb_idx2 = getNeighborIndex(m2, cp.neighbor[3].integrationLineIndex);
      assert(nb_idx2 != -1);
      CriticalPointNeighbor new_nb4;
      new_nb4.integrationLineIndex = new_il2_idx;
      new_nb4.pointIndex = new_cp_idx;
      m2.neighbor.insert(m2.neighbor.begin()+nb_idx2+1, new_nb4);
      msc.il_vec.push_back(new_il2);

      IntegrationLine& il3 = msc.il_vec[cp.neighbor[2].integrationLineIndex];
      il3.startIndex = new_cp_idx;
      CriticalPoint& m4 = msc.cp_vec[cp.neighbor[2].pointIndex];
      int nb_idx4 = getNeighborIndex(m4, cp.neighbor[2].integrationLineIndex);
      m4.neighbor[nb_idx4].pointIndex = new_cp_idx;
    }
    msc.cp_vec.push_back(new_cp);
    if(bd_flag) cp.neighbor.erase(cp.neighbor.begin());
    else{ // remove two neighbor
      cp.neighbor.erase(cp.neighbor.begin()+1);
      cp.neighbor.erase(cp.neighbor.begin()+1);
    }
  }
}

void ILTracer::unfoldBoundaryMultiSaddle(CriticalPoint& cp){
  CriticalPointNeighborArray nb_vec = cp.neighbor;
  const VertHandleArray& adj_vertices = msc.mesh->getAdjVertices(cp.meshIndex);
  for(size_t k=1; k<adj_vertices.size()-1; ++k){
    int adj_vid = adj_vertices[k];
    for(size_t i=0; i<cp.neighbor.size(); ++i){
      const IntegrationLine& il = msc.il_vec[cp.neighbor[i].integrationLineIndex];
      if(il.path[1] == adj_vid){ nb_vec.push_back(cp.neighbor[i]); break; }
    }
  }
  assert(cp.neighbor.size()>4);
  size_t flag = cp.neighbor.size();
  while(nb_vec.size() != 4){
    CriticalPoint new_cp; new_cp.meshIndex = cp.meshIndex; new_cp.type = SADDLE;
    IntegrationLine new_il0 = msc.il_vec[cp.neighbor[0].integrationLineIndex];
    IntegrationLine new_il1 = msc.il_vec[cp.neighbor[3].integrationLineIndex];

  }
}

bool ILTracer::isNormalSaddle(const CriticalPoint& cp) const{
  if(cp.type != SADDLE) return false;
//  CriticalPointNeighborArray nb_vec = cp.neighbor;
//  if(mesh.isBoundaryVertex(cp.meshIndex)){
//    const VertHandleArray& adj_vertices = msc.mesh->getAdjVertices(cp.meshIndex);
//    for(size_t k=1; k<adj_vertices.size()-1; ++k){
//      int adj_vid = adj_vertices[k];
//      for(size_t i=0; i<cp.neighbor.size(); ++i){
//        const IntegrationLine& il = il_vec[cp.neighbor[i].integrationLineIndex];
//        if(il.path[1] == adj_vid){ nb_vec.push_back(i); break; }
//      }
//    }
//  }
//  return nb_vec.size() == 4;
  return cp.neighbor.size() <=4;
}

int ILTracer::getNeighborIndex(const CriticalPoint& cp, int il_index) const{
    for(size_t k=0; k<cp.neighbor.size(); ++k){
        if(cp.neighbor[k].integrationLineIndex == il_index) return k;
    }
    return -1;
}

} // end namespace
