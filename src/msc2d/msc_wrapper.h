#ifndef MSCWRAPPER_H_
#define MSCWRAPPER_H_

#include <iostream>
#include <vector>
#include <map>
#include "mscomplex.h"

namespace meshlib{
  class Mesh;
}

namespace msc2d{
  
  class MscWrapper{
 public:
    MscWrapper(MSComplex2D& msc);
    ~MscWrapper();

    bool CreateMSComplex2D(double threshold=0.003);

 private:
    /*
      Compair two vertices' scalar
      @return 1: v1>v2; -1: v1<v2; 0:v1==v2
     */
    int cmpScalarValue(size_t vert_1, size_t vert_2) const;
    double calGradient(size_t vert_1, size_t vert_2) const;

    
 private:
    const meshlib::Mesh& mesh;
    const std::vector<double>& sf;
    CriticalPointArray& cp_vec;
    IntegrationLineArray& il_vec;

    //! flat region need this mapping
    std::map<size_t, int> vert_priority_mp;

    friend class CPFinder;
    friend class ILTracer;
  };
}// end namespace

#endif
