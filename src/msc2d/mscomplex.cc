#include "mscomplex.h"
#include "../mesh/Mesh.h"
#include <fstream>

using namespace std;

namespace msc2d{

MSComplex2D::MSComplex2D(){}
MSComplex2D::~MSComplex2D(){}

bool MSComplex2D::setMesh(const string& file_name){
  
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
  //! TODO
  
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


}
