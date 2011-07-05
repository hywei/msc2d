#include "intergration_line_tracer.h"

using namespace std;
namespace msc2d{

ILTracer::ILTracer(MSComplex2D& _msc): msc(_msc){}
ILTracer::~ILTracer(){}

#define NEXT(i, n) ((i+1)%(n))
#define PREV(i, n) ((i+(n)-1)%(n))

bool ILTracer::traceIntergrationLine(){
  if(!createWEdge()) return false;
  if(!traceAscendingPath()) return false; 
  setJunctionFlag();
  if(!traceDescendingPath()) return false;
  return true;
}

bool ILTracer::createWEdge(){
  const Mesh& mesh = msc.mesh;  
  wedge_vec.clear(); wedge_vec.resize(mesh.getVertexNumber());
    
  for(size_t vid = 0; vid<mesh.getVertexNumber(); ++vid){
    vector<size_t> adj_vertices = mesh.getAdjVertices(vid);
    WEdge& we = wedge_vec[vid];

    //! create max_ranges;
    size_t n = adj_vertices.size();
    size_t start_maxp = n;
    for(size_t i=0; i<n; ++i){
      if(msc.cmpScalarValue(vid, adj_vertices[i]) == 1){
        if( msc.mesh.isBoundaryVertex(adj_vertices[i]) && i==0){
          start_maxp = 0; break;
        }else{
          if(msc.cmpScalarValue(vid, adj_vertices[PREV(i,n)]) == -1){
            start_maxp = i; break;
          }
        }
      }
    }
    assert(start_maxp != n);

    size_t start_idx(start_maxp), end_idx;
    for(size_t i=NEXT(start_maxp); i!=start_maxp; i=NEXT(i,n)){
      if(msc.mesh.isBoundaryVertex(vid) && i==n-1){ //! handle boundary
        if(msc.cmpScalarValue(vid, adj_vertices[i]) == 1){
          we.max_range.push_back(make_pair(start_idx, n)); start_idx = 0;
        }else if(msc.cmpScalarValue(vid, adj_vertices[i]) == -1){
          we.min_range.push_back(make_pair(start_idx, n)); start_idx = 0;
        }
      }else{ //! general case
        if(msc.cmpScalarValue(vid, adj_vertices[i]) == -1 &&
           msc.cmpScalarValue(vid, adj_vertices[PREV(i,n)]) ==1){
          we.max_range.push_back(make_pair(start_idx, i)); start_idx = i;
        }else if(msc.cmpScalarValue(vid, adj_vertices[i]) == 1 &&
                 msc.cmpScalarValue(vid, adj_vertices[PREV(i,n)]) == -1){
          we.min_range.push_back(make_pair(start_idx, i)); start_idx = i;
        }
      }
    }    
  }
  return true;
}

void ILTracer::setAscendingPathData(){
  //! set junction flag , in/out vertices info and vert-path mapping
  size_t vert_num = msc.mesh.getVertexNumber();
  junction_flag.clear(); junction_flag.resize(vert_num);
  in_vertices.clear(); in_vertices.resize(vert_num);
  out_vertices.clear(); out_vertices.resize(vert_num);
  vert_path_mp.clear(); vert_path_mp.resize(vert_num);
  for(size_t k=0; k<il_vec.size(); ++k){
    const PATH& path = il_vec[k].path;
    for(size_t i=0; i<path.size(); ++i){
      int vid = path[i];
      if(vert_type_vec[vid] == REGULAR) junction_flag[vid] = true;
      vert_path_mp[vid].push_back(k);
      if(i!=0) in_vertices[vid].push_back(path[i-1]);
      if(i!=path.size()-1) out_vertices.push_back(path[i+1]);
    }    
  }
}

bool ILTracer::traceAscendingPath(){
  for(vector<CriticalPoint>::iterator it = msc.cp_vec.begin(); it != msc.cp_vec.end(); ++it){
    if(it->type == SADDLE) {
      CriticalPoint& sad = *it;
      const vector<pair>& max_ranges = wedge_vec[sad.meshIndex].max_ranges;
      for(size_t k=0; k<max_ranges.size(); ++k){
        int prev_vid = -1;
        int curr_vid = sad.meshIndex;
        msc.il_vec.push_back(IntegrationLine());
        
        IntegrationLine& il=msc.il_vec[msc.il_vec.size()-1];
        PATH& mesh_path = il.path;
        mesh_path.push_back(curr_vid);
        do{
          // get right range to trace gradient
          pair<int, int> range;
          if(mesh_path.size()==1) range = max_ranges[k];
          else{
            if(vert_type_vec[curr_vid] == SADDLE){
              // select the previous adjacent maxrange 
              int range_idx = getRangeIndex(curr_vid, prev_vid);              
              if(we_vec[curr_vid].max_ranges.size() <= range_idx){
                // may be happened at boundary 
                --range_idx;
              }
              range = we_vec[curr_vid].max_ranges[range_idx];
            }else range = we_vec[curr_vid].max_ranges[0];
          }
          prev_vid = curr_vid;
          curr_vid = getGradDirection(curr_vid, range);
          assert(curr_vid != -1);
          mesh_path.push_back(curr_vid);
          assert(mesh_path.size() < msc.p_mesh->getEdgeNumber());
        }while(vert_type_vec[curr_vid] != MAXIMUM);
        il.startIndex = distance(it, msc.cp_vec.begin());
        il.endIndex = distance(find(msc.cp_vec.begin(), msc.cp_vec.end(),
                                    curr_vid), msc.cp_vec.begin());
      }
    }
  }
}

bool ILTracer::traceDescendingPath(){
  for(vector<CriticalPoint>::iterator it=msc.cp_vec.begin(); it!=msc.cp_vec.end(); ++it){
    if(it->type == SADDLE) traceDescendingPath(*it);
  }
  return true;
}

bool ILTracer::traceDescendingPath(CriticalPoint& sad){
  const vector<pair>& min_ranges = wedge_vec[sad.meshIndex].min_ranges;
  for(size_t k=0; k<min_ranges.size(); ++k){
    int prev_vid = sad.meshIndex;
    int curr_vid = getDescendingPathSecondVert(sad, k);
    vector<int> mesh_path;
    mesh_path.push_back(sad.meshIndex);
    mesh_path.push_back(curr_vid);
    do{
      pair<int, int> range;
      if(vert_type_vec[curr_vid] == SADDLE){
        
      }else if(vert_type_vec[curr_vid] == REGULAR){
        if(junction_flag[curr_vid] == true){
          
        }else range = we_vec[curr_vid].min_ranges[k];
      }
      prev_vid = curr_vid;
      cur__vid = getGradDirection(curr_vid, range);      
      assert(curr_vid != -1);
      mesh_path.push_back(curr_vid);
      assert(mesh_path.size() < msc.p_mesh->getEdgeNumber());
    }while(vert_type_vec[next_vid] != MINIMUM);
  }
}

int ILTracer::getDescendingPathSecondVert(const CriticalPoint& cp, int range_index) const{  
  const VertHandleArray& adj_vertices = msc.mesh.getAdjVertices(cp.meshIndex);
  const pair<size_t, size_t>& min_r = we_vec[cp.meshIndex].min_ranges[range_index];
  pair<size_t, size_t> range= min_r;
  int last_adj_vid(-1);
  for(size_t k=min_r.first; r!=min_r.second; r=NEXT(r,adj_vertices.size())){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_vertices[cp.meshIndex], adj_vid)){ range.first = k; last_adj_vid = adj_vid; }
  }
  if(last_adj_vid !=-1){
    const vector<int>& paths = vert_path_mp[last_adj_vid];
    for(size_t k=0; k<paths.size(); ++k) path_side_record[paths[k]] = 1; // back direction
  }
  return getGradDirection(cp.meshIndex, range);
}

pair<size_t, size_t> ILTracer::getMinRangeAtJunction(int curr_vid, int prev_vid){
  const VertHandleArray& adj_vertices = msc.mesh.getAdjVertices(curr_vid);
  const vector<int>& in_verts = in_vertices[curr_vid];
  const vector<int>& out_verts = out_vertices[curr_vid];
  const pair<size_t, size_t>& max_r = we_vec[curr_vid].max_ranges[0];
  const pair<size_t, size_t>& min_r = we_vec[curr_vid].min_ranges[0];
  bool visited_flag = false;
  for(size_t k=max_r.first; k!=max_r.second; ++k){
    int adj_vid = adj_vertices[k];
    if(adj_vid == prev_vid) visited_flag = true;
    if(Util::isIn(out_verts, adj_vid)){
      if(adj_vid == prev_vid) continue; //! side have set before
      int side = 0;
      if(visited_flag == true) side = 1;
      else side = -1;
      //! set path side flag
      pair<int, int> e = MAKE(vid, adj_vid);
      const vector<size_t>& path_ids = edge_path_mp[e];
      for(size_t i=0; i<path_ids.size(); ++i){
        path_side_record[path_ids[i]] = side;
      }
    }
  }
  size_t first = min_r.first, second = min_r.second;
  for(size_t k=min_r.first; k!=min_r.second; ++k){
    int adj_vid = adj_vertices[k];
    if(Util::isIn(in_verts., adj_vid)){
      pair<int,int> e = MAKE(adj_vid, vid);
      const vector<size_t>& path_ids = edge_path_mp[e];
      for(size_t i=0; i<path_ids.size(); ++i){
        map<size_t, short>::const_iterator im = path_side_record.find(path_ids[i]);
        if(im!=path_side_record.end()){
          if(im->second == 1) second = next(curr_vid, k);
          else if(im->second == -1) first = k;
        }
      }
    }
  }
  return make_pair(first, second);
}

pair<size_t, size_t> ILTracer::getMinRangeAtSaddle(int sadd_vid, int prev_vid) const{
  const VertHandleArray& adj_vertices = msc.mesh.getAdjVertices(sadd_vid);
  const vector<int>& in_verts = in_vertices[sadd_vid];
  const vector<int>& out_verts = out_vertices[sadd_vid];
  int side = 0;
  if(prev_vid == -1) side = -1;
  else{
    int range_idx = getRangeIndex(sadd_vid, prev_vid);
    assert(range_idx != -1);
    int il_idx = getILIndex(sadd_vid, range_idx);
    
  }
  
}


int ILTracer::getGradDirection(int vid, const pair<size_t, size_t>& range){
  VertHandleArray adj_vertices = msc.mesh.getAdjVertices(vid);
  size_t adj_num = adj_vertices.size();
  double grad = -1.0;
  int grad_dir = -1;
  for(size_t i=range.first; i!=range.second; i=NEXT(i, adj_num)){
    double cur_grad = fabs(msc.calGradient(vid, adj_vertices[i]));
    if(cur_grad > grad) {
      grad = cur_grad; grad_dir = adj_vertices[i];
    }
  }
  return grad_dir; 
}

int ILTracer::getRangeIndex(int vid, int adj_vid) const{
  const vector<int>& adj_vertices = msc.mesh.getAdjVertices(vid);
  if(!Util::isIn(adj_vertices, adj_vid)) return -1;
  vector<pair<size_t, size_t> >  ranges;
  if(cmpScalarValue(vid, adj_vid) == 1) ranges = wedge_vec[vid].min_ranges;
  else ranges = wedge_vec[vid].max_ranges;
  size_t adj_num = adj_vertices.size();
  for(size_t k=0; k<ranges.size(); ++k){
    for(size_t j=ranges[k].first; j!=ranges[k].second; j = NEXT(j, adj_num)){
      if(adj_vertices[j] == adj_vid) return k;
    }
  }
  assert(1);
  return -1;
}


#undef NEXT
#undef PREV
  
} // end namespace
