#ifndef MSCOMPLEX_H_
#define MSCOMPLEX_H_

#include <vector>
#include <string>
#include <map>
#include <fstream>
#include <boost/shared_ptr.hpp>

namespace meshlib{
  class Mesh;
}

namespace msc2d{

  struct CriticalPointNeighbor {
    int pointIndex;  //the index into critical point array
    int integrationLineIndex; //the integration line between this this and neighbor critical point
  };
  typedef std::vector<CriticalPointNeighbor> CriticalPointNeighborArray;
  enum CriticalPointType{
    MINIMAL = 0,
    SADDLE,

    MAXIMAL,
    REGULAR
  };
  struct CriticalPoint{
    CriticalPointType type;        // 0 -- minimal, 1 -- saddle, 2 -- maximal
    int meshIndex;  //index in original mesh
    CriticalPointNeighborArray neighbor; //neighbors are assumed to be counter-clockwise
  };
  typedef std::vector<CriticalPoint> CriticalPointArray;
  typedef std::vector<int> PATH;
  struct IntegrationLine{
    int startIndex, endIndex; 
    std::vector<int> quadPatchIndex; //the size should be either 1 or 2
    PATH path; //the index into original mesh, path always start from a saddle point to a max/min point
  };  
  typedef std::vector<IntegrationLine> IntegrationLineArray;
  typedef std::vector<int> FaceIndexArray;
  struct QuadPatch
  {
    FaceIndexArray face;  //face index into original mesh
    std::vector<int> boundaryIntegrationLineIndex;  //need to be in counter-clock-wise order
  };
  typedef std::vector<QuadPatch> QuadPatchArray;
  struct PatchAdjacent
  {
    int patchIndex;
    int commonIntegrationLineIndex;
  };
  typedef std::vector< std::vector< PatchAdjacent> > PatchAdjacentArray; 
  
  class MSComplex2D{

 public:
    MSComplex2D();
    ~MSComplex2D();
    
    bool setMesh(const std::string& file_name);
    bool setMesh(boost::shared_ptr<meshlib::Mesh> p_mesh);
    bool setScalarField(const std::string& file_name);
    bool setScalarField(const std::vector<double>& sf);

    bool checkMeshAndScalarField() const;
    bool createMSComplex2D(double threshold = 0.003);

    bool loadMSComplex(const std::string& file_name);
    bool saveMSComplex(const std::string& file_name) const;

 private:
    /*
      Compair two vertices' scalar
      @return 1: v1>v2; -1: v1<v2; 0:v1==v2
    */
    int cmpScalarValue(int vert_1, int vert_2) const;
    double calGradient(int vert_1, int vert_2) const;
    double calPersistence(int cp1_index, int cp2_index) const;
    CriticalPointType getVertexType(int vid) const;
    
 private:
    boost::shared_ptr<meshlib::Mesh> mesh;
    std::vector<double> scalar_field;

    CriticalPointArray cp_vec;
    IntegrationLineArray il_vec;
    QuadPatchArray qp_vec;

    // vertex priority for flat region
    std::map<int, int> vert_priority_mp;
    // vertex index -> critical point index mapping
    std::vector<int> vert_cp_index_mp;

    friend class CPFinder;
    friend class ILTracer;
    friend class Simplifor;
    friend class QPGenerator;

    friend std::istream & operator >> (std::istream&, MSComplex2D&);
    friend std::ostream & operator << (std::ostream&, const MSComplex2D&);
  };

  std::istream & operator >> (std::istream&, MSComplex2D&);
  std::ostream & operator << (std::ostream&, const MSComplex2D&);

  bool operator == (const CriticalPointNeighbor& lhs, const CriticalPointNeighbor& rhs);
}// end namespace

#endif
