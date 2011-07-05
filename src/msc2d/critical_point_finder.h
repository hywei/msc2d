#ifndef critical_point_finder_h_
#define critical_point_finder_h_

#include "msc_wrapper.h"

namespace msc2d{

  class CPFinder{
 public:
    CPFinder(MscWrapper& _msc);
    ~CPFinder();

    bool findCriticalPoints();

 private:
    bool resolveFlatRegion();
    CriticalPointType getPointType(int mesh_point_index) const;
    
 private:
    MscWrapper& msc;
  };
} // end namespace
#endif
