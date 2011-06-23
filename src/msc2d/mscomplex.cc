#include "mscomplex.h"

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

bool MSComplex2D::createMSComple(double threshold /*=0.003*/){
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
  
  return true;
}




}
