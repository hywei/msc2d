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
        return lhs.second < rhs.second;
      }
    };
    
 public:
    Simplifor(MSComplex2D& _msc);
    ~Simplifor();

    void simplify(double threshold = 0.003);

 private:
    void calPersistence();
    bool cancel(int cancelIL_index);
    void transferConnection(int cp1_idx, int cp2_idx, int il1_idx, int il2_idx);
    void removeSad(int cp_index);
    void update();
    void refinePath();
    int getILIndexInNeighbor(const CriticalPoint& cp, int il_index) const;

    bool cmpPersPair(const PersPair& lhs, const PersPair& rhs) const{
      return lhs.second < rhs.second;
    }
    
 private:
    MSComplex2D& msc;
    CriticalPointArray& cp_vec;
    IntegrationLineArray& il_vec;

    std::set<PersPair, PersPairCmp> persistence_set;
    std::vector<int> removed_il_flag;
    std::vector<int> removed_cp_flag;
    double sum_persistence;
  };
}

#endif
