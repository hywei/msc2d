#ifndef MSCWRAPPER_H_
#define MSCWRAPPER_H_

#include <map>

namespace meshlib{
  class Mesh;
}

namespace msc2d{
  class MSComplex2D;  

  class MscWrapper{
 public:
    MscWrapper(MSComplex2D& msc);
    ~MSComplex2D();

    bool CreateMSComplex2D(double threshold=0.003);

 private:
    /*
      Compair two vertices' scalar
      @return 1: v1>v2; -1: v1<v2; 0:v1==v2
     */
    int cmpScalarValue(size_t vert_1, size_t vert_2) const;

    double calGrandient(size_t vert_1, size_t vert_2) const;

    
 private:
    const meshlib::Mesh& mesh;
    const std::vector<double>& scalar_field;

    //! flat region need this mapping
    std::map<size_t, int> vert_priority_mp;      
  };
}// end namespace

#endif
