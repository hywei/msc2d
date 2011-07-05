#ifndef MESHLIB_COORD2D_H_
#define MESHLIB_COORD2D_H_

#include <iostream>

namespace meshlib{

    /* ================== Coordinate (2D) ================== */

    template < typename T> class Vec2D
    { 
    private:
        T pos[2];

    public:
        // Constructor
        Vec2D();
        Vec2D(T v[2]);
        Vec2D(const Vec2D<T> &c);        
        Vec2D(T _x , T _y = T());
    
        // Destructor
        ~Vec2D();

        // Initializer
        inline void setCoords(T _x = T(), T _y = T());

        // Basic operator
        inline Vec2D<T> operator +(const Vec2D<T> &) const;
        inline Vec2D<T> operator -(const Vec2D<T> &) const;
        inline Vec2D<T> operator *(T) const;
        inline Vec2D<T> operator /(T) const;
        inline Vec2D<T> operator -() const;

        inline Vec2D<T>& operator +=(const Vec2D<T> &);
        inline Vec2D<T>& operator -=(const Vec2D<T> &);
        inline Vec2D<T>& operator *=(T);
        inline Vec2D<T>& operator /=(T);

        inline Vec2D<T>& operator =(const Vec2D<T> &);

        inline bool operator ==(const Vec2D<T> &) const;
        inline bool operator !=(const Vec2D<T> &) const;

        inline T& operator [](int);
        inline T operator [](int) const;

        // Advanced operator
        template < typename F>
            inline friend F dot(const Vec2D<F>&, const Vec2D<F>&);
        template < typename F>
            inline friend F angle(const Vec2D<F>&, const Vec2D<F>&);

        inline T abs() const;
        inline T sqrabs() const;
    
        inline Vec2D<T> unit() const;
        inline bool normalize();

        inline T x() const;
        inline T y() const;
    };

    // Constructor and Destructor
    template <typename T> 
        inline Vec2D<T>::Vec2D()
        {
            pos[0] = pos[1] = T();
        }
    template <typename T>
        inline Vec2D<T>::Vec2D(T v[2])
        {
            pos[0] = v[0]; pos[1] = v[1]; 
        }
    template <typename T>
        inline Vec2D<T>::Vec2D(const Vec2D<T> &c)
        {
            pos[0] = c.pos[0]; pos[1] = c.pos[1]; 
        }
    template <typename T>
        inline Vec2D<T>::Vec2D(T _x, T _y /*= T()*/)
        {
            pos[0] = _x; pos[1] = _y; 
        }
    template <typename T> Vec2D<T>::~Vec2D(){}

    
    // Initializer
    template <typename T>
        inline void Vec2D<T>::setCoords(T _x /* = 0.0 */, T _y /* = 0.0 */)
        {
            pos[0] = _x; pos[1] = _y;
        }

    // Basic operator
    template <typename T>
        inline Vec2D<T> Vec2D<T>::operator +(const Vec2D<T> &c) const
        {
            Vec2D<T> coord;
            coord.pos[0] = pos[0]+c.pos[0];
            coord.pos[1] = pos[1]+c.pos[1];
            return coord;
        }
    template <typename T>
        inline Vec2D<T> Vec2D<T>::operator -(const Vec2D<T> &c) const
        {
            Vec2D<T> coord;
            coord.pos[0] = pos[0]-c.pos[0];
            coord.pos[1] = pos[1]-c.pos[1];
            return coord;
        }

    template <typename T>
        inline Vec2D<T> Vec2D<T>::operator *(T v) const
        {
            Vec2D<T> coord;
            coord.pos[0] = pos[0]*v;
            coord.pos[1] = pos[1]*v;
            return coord;
        }

    template <typename T>
        inline Vec2D<T> Vec2D<T>::operator /(T v) const
        {
            Vec2D<T> coord;
            if(ALMOST_EQUAL_SMALL(v,0.0))
                {
                    std::cerr << "Vec2D<T> Error: divided by zero" << std::endl;
                    return coord;
                }
            coord.pos[0] = pos[0]/v;
            coord.pos[1] = pos[1]/v;
            return coord;
        }

    template <typename T>
        inline Vec2D<T> Vec2D<T>::operator -() const
        {
            Vec2D<T> coord;
            coord.pos[0] = -1.0*pos[0];
            coord.pos[1] = -1.0*pos[1];
            return coord;
        }

    template <typename T>
        inline Vec2D<T>& Vec2D<T>::operator +=(const Vec2D<T> &c)
        {
            pos[0] += c.pos[0];
            pos[1] += c.pos[1];
            return *this;
        }

    template <typename T>
        inline Vec2D<T>& Vec2D<T>::operator -=(const Vec2D<T> &c)
        {
            pos[0] -= c.pos[0];
            pos[1] -= c.pos[1];
            return *this;
        }

    template <typename T>
        inline Vec2D<T>& Vec2D<T>::operator *=(T v)
        {
            pos[0] *= v;
            pos[1] *= v;
            return *this;
        }

    template <typename T>
        inline Vec2D<T>& Vec2D<T>::operator /=(T v)
        {
            if(ALMOST_EQUAL_SMALL(v,0.0))
            {
                std::cerr << "Vec2D<T> Error: divided by zero" << std::endl;
                return *this;
            }
            pos[0] /= v;
            pos[1] /= v;
            return *this;
        }

    template <typename T>
        inline Vec2D<T>& Vec2D<T>::operator =(const Vec2D<T> &c)
        {
            pos[0] = c.pos[0];
            pos[1] = c.pos[1];
            return *this;
        }

    //
    template <typename T>
        inline bool Vec2D<T>::operator ==(const Vec2D<T> &c) const
        {
            if(pos[0]!=c.pos[0] || pos[1]!=c.pos[1])
                return false;
            return true;
        }

    template <typename T>
        inline bool Vec2D<T>::operator !=(const Vec2D<T> &c) const
        {
            if(pos[0]!=c.pos[0] || pos[1]!=c.pos[1])
                return true;
            return false;
        }

    //
    template <typename T>
        inline T& Vec2D<T>::operator [](int i)
        {
            return pos[i];
        }

    template <typename T>
        inline T Vec2D<T>::operator [](int i) const
        {
            return pos[i];
        }

    // Advanced operator
    template <typename T>
        inline T dot(const Vec2D<T>& c1, const Vec2D<T>& c2)
        {
            return (c1.pos[0]*c2.pos[0] + c1.pos[1]*c2.pos[1]);
        }
    
    template <typename T>
        inline T angle(const Vec2D<T>& c1, const Vec2D<T>& c2)
        {
            Vec2D<T> tmp_c1(c1), tmp_c2(c2);
            tmp_c1.normalize();
            tmp_c2.normalize();

            T dot_ = dot(tmp_c1, tmp_c2);
            if(dot_ < -1.0)
                dot_ = -1.0;
            else if(dot_ > 1.0)
                dot_ = 1.0;
	
            return acos(dot_);
        }

    //
    template <typename T>
        inline T Vec2D<T>::sqrabs() const
        {
            return (pos[0]*pos[0] + pos[1]*pos[1]);
        }

    template <typename T>
        inline T Vec2D<T>::abs() const
        {
            return sqrt(pos[0]*pos[0] + pos[1]*pos[1]);
        }
    
    template <typename T>
        inline Vec2D<T> Vec2D<T>::unit() const
        {
            Vec2D<T> c;
            T length = this->abs();
            if(ALMOST_EQUAL_SMALL(length, 0.0))
                {
                    std::cerr << "Vec2D<T> Error: divided by zero" << std::endl;
                    return c;
                }
            c.pos[0] = pos[0]/length;
            c.pos[1] = pos[1]/length;
            return c;
        }

    template <typename T>
        inline bool Vec2D<T>::normalize()
        {
            T length = this->abs();
            if(ALMOST_EQUAL_SMALL(length, 0.0))
                {
                    std::cerr << "Vec2D<T> Error: divided by zero" << std::endl;
                    return false;
                }
            pos[0] /= length;
            pos[1] /= length;
            return true;
        }

    template <typename T>
        inline T Vec2D<T>::x() const
        {
            return pos[0];
        }

    template <typename T>
        inline T Vec2D<T>::y() const
        {
            return pos[1];
        }
    
} // namespace meshlib
#endif
