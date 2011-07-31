#include <iostream>
#include "../mesh/Mesh.h"
#include "../msc2d/mscomplex.h"
#include <boost/shared_ptr.hpp>
#include <fstream>

using namespace std;
int main(int argc, char** argv)
{
  if(argc <3) {
    cout << "Usage: msc2d mesh-file scalar-field-file"<< endl;
    return -1;
  }
  
  msc2d::MSComplex2D msc;
  msc.setMesh(argv[1]);
  msc.setScalarField(argv[2]);
  msc.createMSComplex2D(0.003);
  string sf_filename = argv[2];
  size_t idx = sf_filename.rfind(".sf");
  if(idx != string::npos){
    string msc_filename = sf_filename;
    msc_filename.replace(idx, ".sf", ".msc");
    //cout << msc_filename << endl;
    msc.saveMSComplex(msc_filename);
    ofstream fout(msc_filename.c_str());
    fout << msc;
    string dual_msc_fn = sf_filename;
    dual_msc_fn.replace(idx, ".sf", ".quad");
    msc.createDualMSComplex2D(dual_msc_fn, 0.003);
  }

  return 0;
}
