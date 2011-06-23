#include "intergration_line_tracer.h"

using namespace std;
namespace msc2d{

ILTracer::ILTracer(MSComplex2D& _msc): msc(_msc){}
ILTracer::~ILTracer(){}

#define NEXT(i, n) ((i+1)%(n))
#define PREV(i, n) ((i+(n)-1)%(n))

bool ILTracer::traceIntergrationLine(){
  if(!createWEdge) return false;
  
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


bool ILTracer::traceAscendingPath(){
  for(vector<CriticalPoint>::iterator it = msc.cp_vec.begin(); it != msc.cp_vec.end(); ++it){
    if(it->type == SADDLE) traceAscendingPath(*it);
  }
}

bool ILTracer::traceAscendingPath(CriticalPoint& sad){
  //! trace the  ascending paths for a saddle point
  const vector<pair> max_ranges = wedge_vec[sad.meshIndex].max_ranges;
  for(size_t k=0; k<max_ranges.size(); ++k){
    int next_vid = -1;
    int curr_vid = sad.meshIndex;
    vector<int> mesh_path; mesh_path.push_back(curr_vid);
    do{
      next_vid = getGradientDirection(curr_vid);
      assert(next_vid != -1);
      curr_vid = next_vid;
      mesh_path.push_back(curr_vid);
      assert(mesh_path.size() < msc.p_mesh->getEdgeNumber());
    }while(vert_type_vec[next_vid] != MAXIMUM);
  }

  return true;
}

int ILTracer::getGradientAscendingDirection(int vid, int pre_vid) const{
  int cp_index = cp_index_vec[vid];
  int range_index(-1);
  if(cp_index == -1){
    const WEdge& we = wedge_vec[vid];
    assert(we.max_ranges.size() == 1 && we.min_ranges.size() == 1);
    range_index = 0;
  }else {
    assert(cp_vec[cp_index].type == SADDLE);
    range_index = getMinRangeIndex(vid, pre_vid);
  }
  return getGradientAscendingDirection(vid, we.max_ranges[range_index]);
}

int ILTracer::getGradientAscendingDirection(int vid, pair<size_t, size_t> range) const{
  vector<size_t> adj_vertices = msc.mesh.getAdjVertices(vid);
  size_t adj_num = adj_vertices.size();
  double grad = -1.0;
  int grad_dir = -1;
  for(size_t i=range.first; i!=range.second; i=NEXT(i, adj_num)){
    double cur_grad = msc.calGradient(vid, adj_vertices[i]);
    if(cur_grad > grad) {
      grad = cur_grad; grad_dir = adj_vertices[i];
    }
  }
  return grad_dir;
}

int ILTracer::getMaxRangeIndex(int vid, int adj_vid) const{
  const WEdge& we = wedge_vec[vid];
  const vector<size_t>& adj_vertices = msc.mesh.getAdjVertices(vid);
  size_t adj_num = adj_vertices.size();
  for(size_t k=0; k<we.max_ranges.size(); ++k){
    const pair<size_t, size_t>& mr = we.max_ranges[k];
    for(size_t j=mr.first; j!= NEXT(mr.second, adj_num); j = NEXT(j, adj_num)){
      if(adj_vertices[j] == adj_vid) return k;
    }
  }

  return -1;
}

int ILTracer::getMinRangeIndex(int vid, int adj_vid) const{
  const WEdge& we = wedge_vec[vid];
  const vector<size_t>& adj_vertices = msc.mesh.getAdjVertices(vid);
  size_t adj_num = adj_vertices.size();
  for(size_t k=0; k<we.min_ranges.size(); ++k){
    const pair<size_t, size_t>& mr = we.max_ranges[k];
    for(size_t j=mr.first; j!= NEXT(mr.second, adj_num); j = NEXT(j, adj_num)){
      if(adj_vertices[j] == adj_vid) return k;
    }
  }

  return -1;
}


#undef NEXT
#undef PREV
  
} // end namespace
