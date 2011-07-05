#ifndef MESHLIBMARCO_H_
#define MESHLIBMARCO_H_

#include <cmath>

namespace meshlib{
    /* ================== Enumerations ================== */

    typedef enum {SOLID_SMOOTH, SOLID_FLAT, WIREFRAME, SOLID_AND_WIREFRAME, VERTICES, HIGHLIGHT_ONLY, TEXTURE_MAPPING,
                  SELECT_NULL, SELECT_VERTEX, SELECT_EDGE, SELECT_FACE, } RENDER_MODE;
 
    typedef enum {TRANSLATE, SCALE, ROTATE, NONE} TRANSFORM_MODE;

    typedef enum {FACE, EDGE, VERTEX} PICKING_MODE;

    typedef enum {AXIS_X, AXIS_Y, AXIS_Z} AXIS;

    typedef enum {R=0, G, B} COLOR;



    /* ================== Constants ================== */

    const double PI = 3.1415926535897932384626433832795;
    const double PI_12 = 1.5707963267948966192313216916395;
    const double LARGE_ZERO_EPSILON = 1.0E-8;
    const double SMALL_ZERO_EPSILON = 1.0E-16;



    /* ================== Math Macros ================== */

    // Min & Max & ABS
    template <typename T> inline T Max3(const T& a, const T& b, const T& c)
        {
            return (a > b) ? ( a > c ? a : c) : ( b > c ? b : c);
        }
    template <typename T> inline T Min3(const T& a, const T& b, const T& c)
        {
            return (a < b) ? ( a < c ? a : c) : ( b < c ? b : c);
        }


    // Equal within epsilon tolerance
    inline bool ALMOST_EQUAL_LARGE(double x, double y)
    {
        return (fabs(x - y) < LARGE_ZERO_EPSILON);
    }
    inline bool ALMOST_EQUAL_SMALL(double x, double y)
    {
        return (fabs(x - y) < SMALL_ZERO_EPSILON);
    }

    inline bool GreaterEqual(double a, double b, double epsilon=LARGE_ZERO_EPSILON)
    {
        return (a > b) || ( fabs(a - b) < epsilon);
    }
    inline bool LessEqual(double a, double b, double epsilon = LARGE_ZERO_EPSILON)
    {
        return (a < b) || ( fabs(a - b) < epsilon);
    }

        /* ================== Color Macros ================== */
#define RED			Color(1.0,0.0,0.0)
#define GREEN		Color(0.0,1.0,0.0)
#define BLUE		Color(0.0,0.0,1.0)
#define	YELLOW		Color(1.0,1.0,0.0)
#define CYAN		Color(0.0,1.0,1.0)
#define MAGENTA		Color(1.0,0.0,1.0)
#define BLACK		Color(0.0,0.0,0.0)
#define	WHITE		Color(1.0,1.0,1.0)
#define ORANGE		Color(1.0,0.5,0.0)
#define DARK_RED	Color(0.5,0.0,0.0)
#define LIGHT_RED	Color(1.0,0.5,0.5)
#define DARK_GREEN	Color(0.0,0.5,0.0)
#define LIGHT_GREEN	Color(0.5,1.0,0.5)
#define DARK_BLUE	Color(0.0,0.0,0.5)
#define LIGHT_BLUE	Color(0.5,0.5,1.0)

#define LIGHT_GREY  Color(0.3,0.3,0.3)
#define GREY        Color(0.5,0.5,0.5)
#define DARK_GREY   Color(0.8,0.8,0.8)



    /* ================== Coordinate Macros ================== */
#define COORD_AXIS_X Coord(1.0, 0.0, 0.0)
#define COORD_AXIS_Y Coord(0.0, 1.0, 0.0)
#define COORD_AXIS_Z Coord(0.0, 0.0, 1.0)
#define COORD_ORIGIN Coord(0.0, 0.0, 0.0)



} // namespace meshlib

#endif
