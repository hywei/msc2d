#ifndef critical_point_finder_h_
#define critical_point_finder_h_

#include "mscomplex.h"

namespace meshlib{
  class Mesh;
}

namespace msc2d{

  class CPFinder{
 public:
    CPFinder(MSComplex2D& _msc);
    ~CPFinder();

    bool findCriticalPoints();

    void printCriticalPointsInfo() const;

 private:
    bool resolveFlatRegion();
    CriticalPointType getPointType(int mesh_point_index) const;
    
 private:
    const meshlib::Mesh& mesh;
    const std::vector<double>& sf;
    MSComplex2D& msc;
  };
} // end namespace
#endif
