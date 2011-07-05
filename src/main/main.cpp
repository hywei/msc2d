#include <iostream>
#include "../mesh/Mesh.h"

using namespace std;
int main(int argc, char** argv)
{
    if(argc <2) {
        std::cout << "Please select a mesh model!" << std::endl;
        return -1;
    }
    meshlib::Mesh mesh;
    if(mesh.attachModel(argv[1])){
      cout << "Vertex Number: " << mesh.getVertexNumber() << endl;
      cout << "Face Number: " << mesh.getFaceNumber() << endl;
      cout << "Edge Number: " << mesh.getEdgeNumber() << endl;
      cout << "Manifold Mesh? : ";
      if(mesh.isManifold()) cout << "YES" << endl;
      else cout << "NO" << endl;
    }else{
      std::cout << "Can't AttachModel !" << std::endl;
    }
    
    return 0;
}
