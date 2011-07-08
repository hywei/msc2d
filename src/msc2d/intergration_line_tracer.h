#ifndef intergration_line_tracer_h_
#define intergration_line_tracer_h_

#include <vector>
#include <iostream>

namespace msc2d{
  class MSComplex2D;
  
  class ILTracer{
    
    class WEdge{
   public:
    std::vector< std::pair<size_t, size_t> > max_ranges;
    std::vector< std::pair<size_t, size_t> > min_ranges;
  };

    class Tree{
   public:
      int root_index;
      std::vector<int> leaf_index_vec;
      std::vector<int> node_vec;

      std::map<int, int> parent;
      std::map<int, vector<int> > children;


      std::map<int, int> leaf_il_mp;
    }

    
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

    bool makeTree();
    bool sortNeighbor(CriticalPoint& cp);
    
 private:
    std::vector<WEdge> wedge_vec;
    std::vector<bool> junction_flag;
    std::vector< std::vector<int> > in_vertics; 
    std::vector< std::vector<int> > out_vertics;
    std::map< std::pair<int, int>, std::vector<size_t> > edge_path_mp;
    std::map<size_t, short> path_side_record; //

    MSComplex2D& msc;
  }; 
} // end namespace

#endif
