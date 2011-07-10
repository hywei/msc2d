#include <iostream>
#include "../mesh/Mesh.h"
#include "../msc2d/mscomplex.h"
#include <boost/shared_ptr.hpp>

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
  msc.createMSComplex2D();
  string sf_filename = argv[2];
  size_t idx = sf_filename.rfind(".sf");
  if(idx != string::npos){
    string msc_filename = sf_filename;
    msc_filename.replace(idx, ".sf", ".msc");
    cout << msc_filename << endl;
    msc.saveMSComplex(msc_filename);
  }

  return 0;
}
