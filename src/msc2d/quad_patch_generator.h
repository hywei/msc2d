#include "mscomplex.h"
#include <map>
#include <set>

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
    void genMMSadMapping();
 private:
    MSComplex2D& msc;
    const meshlib::Mesh& mesh;

    std::vector< std::vector<int> > formed_patchs;
    // the map between a pair(max, min) to saddles
    std::map< std::pair<int, int>, std::vector<int> > mm_sad_mp;
    std::vector<int> face_patch_index_mp;
    std::set<int> covered_edge_set;
  };
}
