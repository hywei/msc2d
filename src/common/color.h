#ifndef MESHLIB_COLOR_H_
#define MESHLIB_COLOR_H_

#include <cstdlib>

namespace meshlib
{
    /* ================== Color (RGB) ================== */
    template <typename T> class Color{

    private:
        T rgb[3];
    
    public:
        // Constructor
        Color();
        Color(T r, T g, T b);
        
        // Destructor
        ~Color();

        // Initializer
        inline void setColor(T r, T g, T b);
        inline void setRandomColor();

        // Basic operator
        inline Color<T> operator +(const Color<T> &) const;
        inline Color<T> operator -(const Color<T> &) const;
        inline Color<T> operator *(T) const;
        inline Color<T> operator /(T) const;
        inline Color<T> operator -() const;

        inline Color<T>&  operator +=(const Color<T> &);
        inline Color<T>&  operator -=(const Color<T> &);
        inline Color<T>&  operator *=(T);
        inline Color<T>&  operator /=(T);
    
        inline Color<T>&  operator =(const Color<T> &color);

        inline bool operator ==(const Color<T> &) const;
        inline bool operator !=(const Color<T> &) const;
    
        inline T& operator[](int i);
        inline T operator[](int i) const;

        // Advanced operator
        inline void Clamp();   // Make sure r,g,b - [0.0, 1.0]

        //
        T r() const {return rgb[0];}
        T g() const {return rgb[1];}
        T b() const {return rgb[2];}
    };

    /* ================== Color<T> (RGB) ================== */

    template < typename T>
        inline Color<T>::Color()
        {
            rgb[0] = rgb[1] = rgb[2] = T();
        }
    template < typename T>
        inline Color<T>::Color( T r, T g, T b)
        {
            setColor(r, g, b);
        }
    template < typename T>
        inline Color<T>::~Color(){}
    
    // Initializer
    template < typename T>
        inline void Color<T>::setColor(T r, T g, T b)
        {
            rgb[0] = r; rgb[1] = g; rgb[2] = b;
            Clamp();
        }

    template <>
        inline void Color<int>::setColor(int r, int g, int b)
        {
            rgb[0] = r/255.0; rgb[1] = g/255.0; rgb[2] = b/255.0;
            Clamp();
        }

    template < typename T>
        inline void Color<T>::setRandomColor()
        {
            for(int i = 0; i < 3; i ++)
                rgb[i] = (T)(rand()%255)/(T)255;
        }

    // Bsaic operator
    template < typename T>
        inline Color<T> Color<T>::operator +(const Color<T>&  color) const
        {
            Color<T> c;
            for(int i = 0; i < 3; i ++)
                c[i] = rgb[i]+color[i];
            return c;
        }

    template < typename T>
        inline Color<T> Color<T>::operator -(const Color<T>&  color) const
        {
            Color<T> c;
            for(int i = 0; i < 3; i ++)
                c[i] = rgb[i]-color[i];
            return c;
        }

    template < typename T>
        inline Color<T> Color<T>::operator *(T v) const
        {
            Color<T> c;
            for(int i = 0; i < 3; i ++)
                c[i] = rgb[i]*v;
            return c;
        }

    template < typename T>
        inline Color<T> Color<T>::operator /(T v) const
        {
            Color<T> c;
            for(int i = 0; i < 3; i ++)
                c[i] = rgb[i]/v;
            return c;
        }

    template < typename T>
        inline Color<T> Color<T>::operator -() const
        {
            Color<T> c;
            for(int i = 0; i < 3; i ++)
                c[i] = -1.0*rgb[i];
            return c;
        }

    //
    template < typename T>
        inline Color<T>&  Color<T>::operator +=(const Color<T> &color)
        {
            for(int i = 0; i < 3; i ++)
                rgb[i] += color[i];
            return *this;
        }

    template < typename T>
        inline Color<T>&  Color<T>::operator -=(const Color<T> &color)
        {
            for(int i = 0; i < 3; i ++)
                rgb[i] -= color[i];
            return *this;
        }

    template < typename T>
        inline Color<T>&  Color<T>::operator *=(T v)
        {
            for(int i = 0; i < 3; i ++)
                rgb[i] *= v;
            return *this;
        }

    template < typename T>
        inline Color<T>&  Color<T>::operator /=(T v)
        {
            for(int i = 0; i < 3; i ++)
                rgb[i] /= v;
            return *this;
        }

    //
    template < typename T>
        inline Color<T>&  Color<T>::operator=(const Color<T> &color)
        {
            for(int i = 0; i < 3; i ++)
                rgb[i] = color[i];
            return *this;
        }

    //
    template < typename T>
        inline bool Color<T>::operator ==(const Color<T> &color) const
        {
            for(int i = 0; i < 3; i ++)
                if(rgb[i] != color[i])
                    return false;
            return true;
        }

    template < typename T>
        inline bool Color<T>::operator !=(const Color<T> &color) const
        {
            for(int i = 0; i < 3; i ++)
                if(rgb[i] != color[i])
                    return true;
            return false;
        }

    template < typename T>
        inline void Color<T>::Clamp()
        {
            for(int i = 0; i < 3; ++ i)
            {
                if(rgb[i] < 0.0)
                    rgb[i] = 0.0;
                else if(rgb[i] > 1.0)
                    rgb[i] = 1.0;
            }
        }

}

#endif
