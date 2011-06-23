#ifndef MESHLIB_BASICDATATYPE_H_

#include "macro.h"
#include "vec2d.h"
#include "vec3d.h"
#include "color.h"

#include <vector>

namespace MeshLib{
    /* ================== Various Indices ================== */
    typedef int VertexID;    // Vertex index
    typedef int	EdgeID;      // HalfEdge index
    typedef int	FaceID;      // Face index

    typedef int	PointID;     // Point index
    typedef int	LineID;      // Line index
    typedef int	PolygonID;   // Polygon index

    typedef int BdyID;       // Boundary index
    typedef int ComponentID; // Component index

    typedef int	ModelID;     // Model index
    typedef int	SceneID;     // Scene index
    
    /* ================== Properties ================== */
    typedef Vec3D<double> Coord;
    typedef Vec3D<double> Coord3D;
    typedef Vec2D<double> Coord2D;
    
    typedef Coord   Normal;
    typedef Coord2D TexCoord;
    typedef int     Flag;
    typedef int     Index;



    /* ================== Property Arrays ================== */
    typedef std::vector<Coord2D> Coord2DArray;
    typedef std::vector<Coord>   CoordArray;
    typedef std::vector< Color<double> >   ColorArray;

    typedef std::vector<bool>    BoolArray;
    typedef std::vector<int>     IntArray;
    typedef std::vector<double>  DoubleArray;
    typedef std::vector<float>   FloatArray;

    typedef std::vector<Normal>   NormalArray;
    typedef std::vector<TexCoord> TexCoordArray;
    typedef std::vector<Index>    IndexArray;
    typedef std::vector<Flag>     FlagArray;

    typedef std::vector<IndexArray >     PolyIndexArray;
    typedef std::vector<CoordArray >     PolyCoordArray;
    typedef std::vector<TexCoordArray >  PolyTexCoordArray;
    typedef std::vector<DoubleArray >    PolyDataArray;
    
} // namespace MeshLib

#endif
