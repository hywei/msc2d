#ifndef DUAL_MSCOMPLEX_GENERATOR_H_
#define DUAL_MSCOMPLEX_GENERATOR_H_

#include "mscomplex.h"
#include <vector>
#include <string>

namespace msc2d{
  class DualGenerator{
 public:
    DualGenerator(MSComplex2D& _msc);
    ~DualGenerator();

    void generateDualPatch();
    bool saveDualMSComplex(const std::string& file_name) const;
    bool saveQuadFile(const std::string& file_name) const;
 private:
    void generateDualCP();
    void generateDualIL();
    void generateDualPatch(int cp_index);
    void generateCPAdjInfo();
    int getPatchIndex(int il_index1, int il_index2) const;
    std::pair<int, int> getMaxMinPair(const QuadPatch& ) const;
    bool checkInnerFaces(const PATH& loop, const std::vector<int>& faces) const;
    std::vector<int> getDualPatchCP(const QuadPatch& dp) const; 
 private:    
    MSComplex2D& msc;
    CriticalPointArray& cp_vec;
    IntegrationLineArray& il_vec;
    QuadPatchArray& qp_vec;

    CriticalPointArray dual_cp_vec;
    IntegrationLineArray dual_il_vec;
    QuadPatchArray dp_vec;

    std::vector<int> cp_mapping;
    std::vector<int> qp_dual_il_map;
    std::vector< std::vector<int> > cp_adj_patch_vec;
    std::vector< int > vert_dual_cp_index_mapping;
  };
}

#endif
