#ifndef MSCOMPLEX_H_
#define MSCOMPLEX_H_

#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

namespace meshlib{
  class Mesh;
}

namespace msc2d{

  struct CriticalPointNeighbor {
    int pointIndex;  //the index into critical point array
    int integrationLineIndex; //the integration line between this this and neighbor critical point
  };
  enum CriticalPointType{
    MINIMAL = 0,
    SADDLE,
    MAXIMAL,
    REGULAR
  };
  struct CriticalPoint{
    CriticalPointType type;        // 0 -- minimal, 1 -- saddle, 2 -- maximal
    int meshIndex;  //index in original mesh
    std::vector<CriticalPointNeighbor> neighbor; //neighbors are assumed to be counter-clockwise
  };
  typedef std::vector<CriticalPoint> CriticalPointArray;
  typedef std::vector<int> PATH;
  struct IntegrationLine{
    int startIndex, endIndex; 
    std::vector<int> quadPatchIndex; //the size should be either 1 or 2
    PATH path; //the index into original mesh, path always start from a saddle point to a max/min point
  };  
  typedef std::vector<IntegrationLine> IntegrationLineArray;

  
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
    boost::shared_ptr<meshlib::Mesh> mesh;
    std::vector<double> scalar_field;

    CriticalPointArray cp_vec;
    IntegrationLineArray il_vec;

    friend class MscWrapper;
  };
  
}// end namespace

#endif
