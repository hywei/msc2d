#include "mscomplex.h"
#include "critical_point_finder.h"
#include "../mesh/Mesh.h"
#include "../common/macro.h"
#include <fstream>
#include <limits>

using namespace std;
using namespace meshlib;

namespace msc2d{

MSComplex2D::MSComplex2D(){}
MSComplex2D::~MSComplex2D(){}

bool MSComplex2D::setMesh(const string& file_name){
  mesh = boost::shared_ptr<Mesh>(new Mesh);
  if(!mesh->attachModel(file_name)){
    cerr << "cannot attach model" << endl;
    return false;
  }else{
    cout << "Vertex Number: " << mesh->getVertexNumber() << endl;
    cout << "Face Number: " << mesh->getFaceNumber() << endl;
    cout << "Edge Number: " << mesh->getEdgeNumber() << endl;
    cout << "Manifold Mesh? : ";
    if(mesh->isManifold()) cout << "YES" << endl;
    else cout << "NO" << endl;

    const VertHandleArray& vh_vec = mesh->getAdjVertices(0);
    for(size_t i=0; i<vh_vec.size(); ++i) cout << vh_vec[i] << " "; cout << endl;
  }
  return true;
}

bool MSComplex2D::setScalarField(const string& file_name){
  ifstream fin(file_name.c_str());
  if(fin.fail()){
    //! TODO:
    cerr << "Cannot load scalar field file " << file_name << endl;
    return false;
  }

  scalar_field.clear(); 
  size_t vert_num;
  fin >> vert_num;

  double scalar;
  while(fin >> scalar){
    scalar_field.push_back(scalar);
  }

  if(scalar_field.size() != vert_num){
    //! TODO:
    cerr << "Warning : " << endl;
  }
  
  fin.close();
  return true;
}

bool MSComplex2D::setScalarField(const vector<double>& _scalar_file){
  scalar_field = _scalar_file;
  return true;
}

bool MSComplex2D::createMSComplex2D(double threshold /*=0.003*/){
  if(!checkMeshAndScalarField()){
    return false;
  }

  CPFinder cp_finder(*this);
  cp_finder.findCriticalPoints();
  cp_finder.printCriticalPointsInfo();
  
}

int MSComplex2D::cmpScalarValue(size_t vid1, size_t vid2) const{
  if( fabs(scalar_field[vid1] - scalar_field[vid2]) < LARGE_ZERO_EPSILON ){
    int pri_1 = vert_priority_mp.find(vid1)->second;
    int pri_2 = vert_priority_mp.find(vid2)->second;
    assert(pri_1 != pri_2);
    return (pri_1 > pri_2) ? 1 : -1;
  }else{
    return scalar_field[vid1] > scalar_field[vid2] ? 1 : -1;
  }
}

double MSComplex2D::calGradient(size_t vid1, size_t vid2) const{
  if(vid1 == vid2) return 0.0;
  const Coord3D& coord1 = mesh->getVertexCoord(vid1);
  const Coord3D& coord2 = mesh->getVertexCoord(vid2);
  double dis = (coord1-coord2).abs();
  if(dis < LARGE_ZERO_EPSILON) return numeric_limits<double>::infinity();
  return (scalar_field[vid1] - scalar_field[vid2])/dis;
}


bool MSComplex2D::checkMeshAndScalarField() const
{
  //! TODO
  if(mesh == NULL) {
    cerr << "Error: please set mesh first!" << endl;
    return false;
  }

  if(mesh->getVertexNumber() != scalar_field.size()){
    cerr << "Error: " << endl;
    return false;
  }  

  if(!mesh->isManifold()) {
    cerr << "non-manifold mesh" <<endl;
  }
  return true;
}


bool MSComplex2D::saveMSComplex(const std::string& file_name) const
{
  ofstream os(file_name.c_str());
  if(!os) {
    cerr << "Cannot open " << file_name << endl;
    return false;
  }

  os << "# Critical Points : CP meshIndex type" << endl;
  for(size_t k=0; k<cp_vec.size(); ++k){
    os << "CP " << cp_vec[k].meshIndex;
    if(cp_vec[k].type == MINIMAL) os << " MINIMAL" << endl;
    else if(cp_vec[k].type == MAXIMAL) os << " MAXNIMAL" << endl;
    else if(cp_vec[k].type == SADDLE) os << " SADDLE" << endl;
  }

  os << "# Integration Lines: cp_index_1 cp_index_2 path" << endl;
  for(size_t k=0; k<il_vec.size(); ++k){
    os << "IL " << il_vec[k].startIndex << " " << il_vec[k].endIndex << " ";
    const PATH& path = il_vec[k].path;
    for(size_t i=0; i<path.size()-1; ++i){
      os << path[i];
      if(i%10 == 0) os << "\\";
      else os << " ";
    }
    os << path[path.size()-1] << endl;
  }
  
  os.close();
  return true;
}

ostream & operator << (std::ostream& os, const MSComplex2D& msc){
  const CriticalPointArray& cp_vec = msc.cp_vec;
  const IntegrationLineArray& il_vec = msc.il_vec;
  
  os << "# Critical Points : CP meshIndex type" << endl;  
  for(size_t k=0; k<cp_vec.size(); ++k){
    os << "CP " << cp_vec[k].meshIndex;
    if(cp_vec[k].type == MINIMAL) os << " MINIMAL" << endl;
    else if(cp_vec[k].type == MAXIMAL) os << " MAXNIMAL" << endl;
    else if(cp_vec[k].type == SADDLE) os << " SADDLE" << endl;
  }

  os << "# Integration Lines: cp_index_1 cp_index_2 path" << endl;
  for(size_t k=0; k<il_vec.size(); ++k){
    os << "IL " << il_vec[k].startIndex << " " << il_vec[k].endIndex << " ";
    const PATH& path = il_vec[k].path;
    for(size_t i=0; i<path.size()-1; ++i){
      os << path[i];
      if(i%10 == 0) os << "\\";
      else os << " ";
    }
    os << path[path.size()-1] << endl;
  }
  return os;
}



}
