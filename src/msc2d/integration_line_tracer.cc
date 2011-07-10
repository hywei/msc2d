#include "integration_line_tracer.h"
#include "mscomplex.h"
#include "../mesh/Mesh.h"
#include "../util/utility.h"

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
  return true;
}

bool ILTracer::createWEdge(){
  wedge_vec.clear(); wedge_vec.resize(mesh.getVertexNumber());
    
  for(size_t vid = 0; vid<mesh.getVertexNumber(); ++vid){
    WEdge& we = wedge_vec[vid];
    const VertHandleArray& adj_vertices = mesh.getAdjVertices(vid);
    size_t n = adj_vertices.size();
    
    if(msc.getVertexType(vid) == MAXIMAL){
      we.max_ranges.push_back(make_pair(0, n)); continue;
    }else if(msc.getVertexType(vid) == MINIMAL){
      we.min_ranges.push_back(make_pair(0, n)); continue;
    }

    size_t start_maxp = n;    
    //! get the first max-range's start position
    for(size_t i=0; i<n; ++i){
      if(msc.cmpScalarValue(vid, adj_vertices[i]) == -1){
        if( mesh.isBoundaryVertex(adj_vertices[i]) && i==0){
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
      if(mesh.isBoundaryVertex(vid) && i==n-1){ //! handle boundary
        if(msc.cmpScalarValue(vid, adj_vertices[i]) == -1){
          we.max_ranges.push_back(make_pair(start_idx, n)); start_idx = 0;
        }else if(msc.cmpScalarValue(vid, adj_vertices[i]) == 1){
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
          const WEdge& we = wedge_vec[curr_vid];
          if(mesh_path.size()==1) range = max_ranges[k];
          else{
            if(msc.getVertexType(curr_vid) == SADDLE){ // select the previous adjacent maxrange              
              int range_idx = getRangeIndex(curr_vid, prev_vid);              
              if(wedge_vec[curr_vid].max_ranges.size() <= range_idx){//at boundary 
                --range_idx;
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
  vector<pair<size_t, size_t> >  ranges;
  if(msc.cmpScalarValue(vid, adj_vid) == 1) ranges = wedge_vec[vid].min_ranges;
  else ranges = wedge_vec[vid].max_ranges;
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
        msc.il_vec.push_back(IntegrationLine());        
        IntegrationLine& il=msc.il_vec[msc.il_vec.size()-1];
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
          prev_vid = curr_vid;
          curr_vid = getGradDirection(curr_vid, range);
          assert(curr_vid != -1);
          path.push_back(curr_vid);
          assert(path.size() < mesh.getEdgeNumber());
        }
        il.startIndex = it->meshIndex;
        il.endIndex = msc.vert_cp_index_mp[curr_vid];
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
  for(size_t k=min_r.first; k!=min_r.second; k=next(cp.meshIndex, k)){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_vertices[cp.meshIndex], adj_vid)){ range.first = k; last_adj_vid = adj_vid; }
  }
  if(last_adj_vid !=-1){
    const vector<size_t>& paths = edge_path_mp[make_pair(last_adj_vid, cp.meshIndex)];
    for(size_t k=0; k<paths.size(); ++k) path_side_record[paths[k]] = 1; // back direction
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
    }else if(adj_vid == prev_vid) side=1;
  }
  assert(side != 0);
  int min_range_idx(-1);
  size_t max_range_num = wedge_vec[sadd_vid].max_ranges.size();
  size_t min_range_num = wedge_vec[sadd_vid].min_ranges.size();
  if(side == 1) {
    min_range_idx = max_range_idx;
    if(min_range_idx >= min_range_num) min_range_idx = max_range_idx-1;
  }else{
    min_range_idx = (max_range_idx + max_range_num-1)%max_range_num;
    if(min_range_idx >= min_range_num) min_range_idx = max_range_idx;
  }
  const pair<size_t, size_t>& min_range = wedge_vec[sadd_vid].min_ranges[min_range_idx];
  size_t first = min_range.first, second = min_range.second;
  for(size_t k=min_range.first; k!=min_range.second; k=next(sadd_vid, k)){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_verts, adj_vid)){
      if(side == 1){ second = next(sadd_vid, k); break;}
      else if(side == -1) { first = k; }
    }
  }
  return make_pair(first, second);
}

// bool ILTracer::sortNeighborIL(CriticalPoint& cp) const{
//   Tree tree;
//   if(!makeTree(cp, tree)) return false;
//   if(!sortTreeNode(tree)) return false;

//   return true;
// }

bool ILTracer::makeTree(const CriticalPoint& cp, Tree& t) const{
  vector<PATH> path_vec;
  for(size_t k=0; k<cp.neighbor.size(); ++k){
    size_t il_index = cp.neighbor[k].integrationLineIndex;
    path_vec.push_back[il_vec[il_index]].path;
  }

  int hash = mesh.getVertexNumber();
  vector<int> leaves;
  map<int, int> leaf_il_mp;
  for(size_t k=0; k<path_vec.size(); ++k){
    PATH& path = path_vec[k];
    int leaf = path[0];
    if(Util::isIn(leaves, leaf)){ // case 1
      leaf = hash++;
      int next_vid = path[1];
      for(size_t j=0; j<path_vec.size(); ++j){
        if(j==k) continue;
        PATH& _path = path_vec[j];
        for(size_t i=0; i<_path.size()-1; ++i){
          int _next_vid = _path[i+1];
          if(_next_vid == next_vid && _path[i] == path[0]){
            _path[i] == leaf;
          }
        }
      }
      path[0] = leaf;
    }
    leaves.push_back(leaf);
    leaf_il_mp[leaf] = k;
  }

  for(size_t k=0; k<path_vec.size(); ++k){
    PATH& path = path_vec[k];
    for(size_t i=1; i<path.size()-1; ++i){
      int node = path[i];
      if(Util::isIn(leaves, node)){ // case 2
        int new_leaf(-1);
        int curr_vid = leaf_il_mp[node];
        int prev_vid(path[i-1]), next_vid(path[i+1]);
        int range_idx1 = getRangeIndex(curr_vid, next_vid);
        int range_idx2 = getRangeIndex(curr_vid, prev_vid);
        int range_idx = -1;

        //! TODO : get range and new leaf
        assert(new_leaf != -1);
        PATH& _path = il_vec[leaf_il_mp[leaf]].path;
        _path.insert(_path.begin(), new_leaf);
        *find(leaves.begin(), leaves.end(), leaf) = new_leaf;
        leaf_il_mp[new_leaf] = leaf_il_mp[leaf];
        leaf_il_mp.erase(leaf_il_mp.find(leaf));
      }
    }
  }

  //! create tree
  tree.root = cp.meshIndex;
  tree.parent[tree.root] = -1;
  for(size_t k=0; k<path_vec.size(); ++k){
    const PATH& path = path_vec[k];
    for(size_t i=1; i<path.size(); ++i){
      int curr_vid = path[i-1];
      int next_vid = path[i];
      tree.parent[curr_vid] = next_vid;
      vector<int>& children = tree.children[next_vid];
      if(!Util::isIn(children, curr_vid)) children.push_back(curr_vid);
    }
  }

  return true;
}

// bool ILTracer::sortTreeNode(Tree& tree) const{
//   stack<int> st;
//   q.push(tree.root);
//   map<int, int>::iterator im;
//   vector<int> il_index_vec;
//   while(!st.empty()){
//     int node = st.top(); st.pop();
//     if(tree.isLeaf(node)){
//       il_index_vec.push_back(tree.node_il_mp[node]);
//       continue;
//     }
//     vector<int> children = tree.children[node];
//     for(size_t k=0; k<children.size(); ++k){
//       int child = children[k];
//       im = tree.node_vert_mp.find(child);
//       if(im != tree.node_vert_mp.end()) children[k] = im->second;
//     }
//     int parent = tree.parent[node];
//     int vid = node;
//     im = tree.node_vert_mp.find(node);
//     if(im != tree.node_vert_mp.end())  vid = im->second;
//     const VertHandleArray& adj_vertices = mesh.getAdjVertices(vid);

//     size_t out_index;
//     if(parent == -1) out_index = 0;
//     else out_index = distance( find(adj_vertices.begin(), adj_vertices.end(),
//                                     parent), adj_vertices.begin());
//     for(size_t k=0, i=out_index; k<adj_vertices.size(); ++k, i=next(vid,i)){
//       int adj_vid = adj_vertices[i];
//       if(Util::isIn(children, adj_vid)){
//         st.push(adj_vid);
//       }
//     }
//   }
  
//   return true;
// }  
} // end namespace
