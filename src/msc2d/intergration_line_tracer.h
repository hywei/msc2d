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

    enum GradDir{
      ASCENDING,
      DESCENDING
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
    std::vector<bool> junction_flag;
    std::vector< std::vector<int> > in_vertics; 
    std::vector< std::vector<int> > out_vertics;
    std::map< std::pair<int, int>, std::vector<size_t> > edge_path_mp;
    std::map<size_t, short> path_side_record; //

    MscWrapper& msc;
  }; 
} // end namespace

#endif
