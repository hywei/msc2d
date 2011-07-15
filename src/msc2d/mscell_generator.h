#include "mscomplex.h"

namespace meshlib{
  class Mesh;
}

namespace msc2d{
  class QPGenerator{
 public:
    QPGenerator(MSComplex2D& _msc);
    ~QPGenerator();

    void genQuadPatchArray();

 private:
    void genQuadPatchArray(const CriticalPoint& cp);
    void fillInnerFace(QuadPatch& qp) const;    
    
 private:
    MSComplex2D& msc;
    const Mesh& mesh;

    std::vector< std::vector<int> > formed_patchs;
    
  };
}
