#ifndef MSC2D_SIMPLIFICATION_H_
#define MSC2D_SIMPLIFICATION_H_

#include <vector>
#include <set>
#include <iostream>
#include "mscomplex.h"

namespace msc2d{
  
  class Simplifor{

    /* PersPair: persistence pair
     *  @first: the correspoinding integration line index
     *  @second: the persistence value of these two critical points
     */
    typedef std::pair< size_t, double> PersPair;

    class PersPairCmp{
   public:
      bool operator()(const PersPair& lhs, const PersPair& rhs) const{
        if(lhs.first == rhs.first) return lhs.second < rhs.second;
        return lhs.first < rhs.first;
      }
    };
    
 public:
    Simplifor(MSComplex2D& _msc, bool _remove_deg_sad=false);
    ~Simplifor();

    void simplify(double threshold = 0.003);
 private:
    void calPersistence();
    bool cancel(int cancelIL_index);
    void transferConnection(int cp1_idx, int cp2_idx, int il1_idx, int il2_idx);
    void removeSad(int cp_index);
    bool removePersPair(int il_index);
    void update();
    void refinePath();
    int getILIndexInNeighbor(const CriticalPoint& cp, int il_index) const;
    bool isAscendingIL(const IntegrationLine& il) const;
    void removeDegenerateSaddle();
    bool isDegenerateSaddle(const CriticalPoint& cp) const;
 private:
    MSComplex2D& msc;
    CriticalPointArray& cp_vec;
    IntegrationLineArray& il_vec;
    
    double cancel_threshold;
    std::map<size_t, double> persistence_map;
    std::vector<int> removed_il_flag;
    std::vector<int> removed_cp_flag;
    double sum_persistence;
    bool remove_deg_sad;
  };
}

#endif
