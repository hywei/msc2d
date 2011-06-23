#ifndef intergration_line_tracer_h_
#define intergration_line_tracer_h_

#include <vector>
#include <iostream>

namespace msc2d{

  class MscWrapper;
  
  class ILTracer{
    
    class WEdge{
 public:
    std::vector< std::pair<size_t, size_t> > max_ranges;
    std::vector< std::pair<size_t, size_t> > min_ranges;
  };
    
 public:
    ILTracer(MscWrapper& );
    ~ILTracer();

    bool traceIntergrationLine();
 private:
    bool createWedge();
    bool traceAscendingPath();
    bool traceAscendingPath(CriticalPoint&);
    bool traceDescendingPath();
    bool traceDescendingPath(CriticalPoint&);

    size_t getGradientAscendingDirection(size_t vid) const;
 private:
    std::vector<WEdge> wedge_vec;

    MscWrapper& msc;
  }; 
} // end namespace

#endif
