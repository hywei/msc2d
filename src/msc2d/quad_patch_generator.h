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
    void genQuadPatch(size_t sad_cp_index);
    bool findPatchInnerFace(QuadPatch& qp) const;
    CriticalPointNeighbor getNextCPNeighbor(const CriticalPointNeighbor&) const;
    void genMMSadMapping();

    void genTriPatch();
    bool getInnerFaces(const PATH& loop, std::vector<int>& face_vec) const;
 private:
    MSComplex2D& msc;
    const meshlib::Mesh& mesh;

    std::vector< std::vector<int> > formed_patchs;
    // the map between a pair(max, min) to saddles
    std::map< std::pair<int, int>, std::vector<int> > mm_sad_mp;
    std::vector<int> face_patch_index_mp;
    std::set<int> covered_edge_set;

    std::vector< std::pair<size_t, size_t> > tri_patch_il_index_vec;
    std::vector< size_t > tri_patch_cp_index_vec;
    bool test_flag;
  };
}
