#include "mscomplex.h"

namespace meshlib{
  class Mesh;
}

namespace msc2d{
  class QPGenerator{
 public:
    QPGenerator(MSComplex2D& _msc);
    ~QPGenerator();

    void genQuadPatch();

 private:
    void genQuadPatch(const CriticalPoint& cp);
    bool findPatchInnerFace(QuadPatch& qp) const;
    CriticalPointNeighbor getNextCPNeighbor(const CriticalPointNeighbor&) const;
 private:
    MSComplex2D& msc;
    const meshlib::Mesh& mesh;

    std::vector< std::vector<int> > formed_patchs;
    
  };
}
