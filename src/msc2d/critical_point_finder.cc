#include "critical_point_finder.h"
#include "mscomplex.h"
#include "../mesh/Mesh.h"
#include <queue>

namespace msc2d{
using namespace std;
using namespace meshlib;

CPFinder::CPFinder(MSComplex2D& _msc):
    msc(_msc), mesh(*_msc.mesh), sf(_msc.scalar_field){}
CPFinder::~CPFinder(){}

bool CPFinder::resolveFlatRegion(){
  size_t vert_num = mesh.getVertexNumber();  
  vector<bool> visited_flag(vert_num, false);

  msc.vert_priority_mp.clear();
  for(size_t vid = 0; vid < vert_num; ++vid){
    if(visited_flag[vid] == false){
      queue<int> q;
      q.push(vid);
      visited_flag[vid] = true;

      int priority = -1;
      while(!q.empty()){
        int v = q.front(); q.pop();

        msc.vert_priority_mp[v] = priority++;

        const VertHandleArray& adj_vertices = mesh.getAdjVertices(v);
        for(size_t k=0; k<adj_vertices.size(); ++k){
          size_t adj_vid = adj_vertices[k];
          if(!visited_flag[adj_vid] && fabs(sf[vid]-sf[adj_vid]) < LARGE_ZERO_EPSILON){
            q.push(adj_vid);
            visited_flag[adj_vid] = true;
          } // end if
        } // end for
      }//end while      
    } // end if
  }
  assert(msc.vert_priority_mp.size() == vert_num);
  return true;
}


bool CPFinder::findCriticalPoints(){
  if(!resolveFlatRegion()) return false;
  CriticalPointArray& cp_vec = msc.cp_vec;
  cp_vec.clear();
  msc.vert_type_vec.clear();
  msc.vert_type_vec.resize(mesh.getVertexNumber());
  for(size_t vid=0; vid<mesh.getVertexNumber(); ++vid){
    msc.vert_type_vec[vid] = getPointType(vid);
    if(msc.vert_type_vec != REGULAR){
      CriticalPoint cp;
      cp.meshIndex = vid;
      cp.type = msc.vert_type_vec[vid];
      cp_vec.push_back(cp);
    }
  }
  return true;
}

CriticalPointType CPFinder::getPointType(int vid) const{  
  const VertHandleArray& adj_vertices = mesh.getAdjVertices(vid);
  vector<bool> adj_vflag;

  for(size_t k=0; k<adj_vertices.size(); ++k){
    int _vid = adj_vertices[k];
    if(msc.cmpScalarValue(vid, _vid) == 1) adj_vflag.push_back(false);
    else if(msc.cmpScalarValue(vid, _vid) == -1) adj_vflag.push_back(true);
  }

  if(mesh.isBoundaryVertex(vid)){
    adj_vflag.insert(adj_vflag.end(), adj_vflag.rbegin()+1, adj_vflag.rend()-1);
  }

  int alter_num(0);
  for(size_t k=0; k<adj_vflag.size(); ++k){
    if(adj_vflag[k] != adj_vflag[(k+1)%adj_vflag.size()]) ++alter_num;
  }

  assert(alter_num %2 == 0);
  if(alter_num == 0){
    return sf[adj_vertices[0]] > sf[vid] ? MAXIMAL : MINIMAL;
  }else if(alter_num == 2) return REGULAR;
  else return SADDLE;
}

void CPFinder::printCriticalPointsInfo() const{
  size_t cp_num = msc.cp_vec.size();
  size_t max_num(0), min_num(0), sad_num(0);
  for(size_t k=0; k<cp_num; ++k){
    if(msc.cp_vec[k].type == MAXIMAL) ++max_num;
    if(msc.cp_vec[k].type == MINIMAL) ++min_num;
    if(msc.cp_vec[k].type == SADDLE) ++sad_num;
  }
  cout << "###Total critical points number: " << cp_num << endl;
  cout << "###Maximum points number: " << max_num << endl;
  cout << "###Minimum points number: " << min_num << endl;
  cout << "###Saddle points number: "  << sad_num << endl << endl;
}

}// end namespace
