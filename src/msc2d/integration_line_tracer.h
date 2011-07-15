#ifndef intergration_line_tracer_h_
#define intergration_line_tracer_h_

#include <vector>
#include <map>
#include <iostream>
#include "mscomplex.h"

namespace meshlib{ class Mesh; }

namespace msc2d{
  class CriticalPoint;      
  
  class ILTracer{
    
    class WEdge{
   public:
      std::vector< std::pair<size_t, size_t> > max_ranges;
      std::vector< std::pair<size_t, size_t> > min_ranges;
    };

    class Tree{
   public:
      int hash;
      int root;
      std::vector<int> leaves;
      std::map<int, int> parent;
      std::map<int, std::vector<int> > children;
      std::map<int, int> node_vert_mp;
      std::map<int, int> node_path_mp;
    };

    
 public:
    ILTracer(MSComplex2D& );
    ~ILTracer();

    bool traceIntegrationLine();
 private:
    bool createWEdge();
    
    bool traceAscendingPath();    
    bool traceDescendingPath();
    void setAscendingPathData();    

    void genCPNeighbor();
    void sortCPNeighbor(CriticalPoint&) const;
    void makeTree(const CriticalPoint&, Tree& t) const;
    void traverseTree(Tree& t, std::vector<int>&) const;
    void splitSaddle(std::vector<PATH>& path_vec, Tree& t) const;
    void replaceSaddle(std::vector<PATH>& path_vec, Tree& t) const;

    // help function    
    size_t next(int vid, size_t curr_index) const;
    size_t prev(int vid, size_t curr_index) const;
    int getRangeIndex(int vid, int adj_vid) const;
    int getGradDirection(int vid, const std::pair<size_t, size_t>& range) const;
    int getDescendingPathSecondVert(const CriticalPoint& cp, int range_idx);
    std::pair<size_t, size_t> getMinRangeAtSaddle(int curr_vid, int prev_vid);
    std::pair<size_t, size_t> getMinRangeAtJunction(int curr_vid, int prev_vid);
    
 private:
    std::vector<WEdge> wedge_vec;
    std::vector<bool> junction_flag;
    std::vector< std::vector<int> > in_vertices; 
    std::vector< std::vector<int> > out_vertices;
    std::map< std::pair<int, int>, std::vector<size_t> > edge_path_mp;
    std::map<size_t, int> path_side_record; //

    MSComplex2D& msc;
    meshlib::Mesh& mesh;
  }; 
} // end namespace

#endif
