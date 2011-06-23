#include <iostream>
#include "../mesh/Mesh.h"

int main(int argc, char** argv)
{
    if(argc <2) {
        std::cout << "Please select a mesh model!" << std::endl;
        return -1;
    }
    MeshLib::Mesh mesh;
    if(mesh.AttachModel(argv[1])){
        std::cout << "Can't AttachModel !" << std::endl;
    }
    
    return 0;
}
