#ifndef MESHLIB_COORD3D_H_
#define MESHLIB_COORD3D_H_

#include <iostream>

namespace meshlib{

    //================== Vec3Dinate (3D) ================== 

    template < typename T> class Vec3D{ 
    private:
        T pos[3];

    public:
        // Constructor
        Vec3D() ;
        Vec3D(T v[3]);
        Vec3D(const Vec3D<T> &c);
        Vec3D(T _x, T _y = T(), T _z = T());

        // Destructor
        ~Vec3D();

        // Initializer
        inline void setVec3Ds(T _x = T(), T _y = T(), T _z = T());

        // Basic operator
        inline Vec3D<T> operator +(const Vec3D<T> &) const;
        inline Vec3D<T> operator -(const Vec3D<T> &) const;
        inline Vec3D<T> operator *(T) const;
        inline Vec3D<T> operator /(T) const;
        inline Vec3D<T> operator -() const;
    
        inline Vec3D<T>& operator +=(const Vec3D<T> &);
        inline Vec3D<T>& operator -=(const Vec3D<T> &);
        inline Vec3D<T>& operator *=(T);
        inline Vec3D<T>& operator /=(T);

        inline Vec3D<T>& operator =(const Vec3D<T> &);

        inline bool operator ==(const Vec3D<T> &) const;
        inline bool operator !=(const Vec3D<T> &) const;

        inline T & operator [](size_t) ;
        inline T operator [](size_t) const;

        // Advanced operator
        template < typename F>
            inline friend F dot(const Vec3D<F>&, const Vec3D<F>&);
        template < typename F>
            inline friend F angle(const Vec3D<F>&, const Vec3D<F>&);
        template < typename F>
            inline friend Vec3D<F> cross(const Vec3D<F>&, const Vec3D<F>&);

        inline T abs() const;
        inline T sqrabs() const;

        inline Vec3D<T> unit() const;
        inline bool normalize();
        
        inline T x() const;
        inline T y() const;
        inline T z() const;        
    };

    //////////////////////////////////////////////////////////////////////////
    // inline implementations

    /* ================== Vec3D<T>inate (3D) ================== */

    // Constructor and Destructor
    template <typename T> 
        inline Vec3D<T>::Vec3D()
        {
            pos[0] = pos[1] = pos[2] = T();
        }
    template <typename T>
        inline Vec3D<T>::Vec3D(T v[3])
        {
            pos[0] = v[0]; pos[1] = v[1]; pos[2] = v[2];
        }
    template <typename T>
        inline Vec3D<T>::Vec3D(const Vec3D<T> &c)
        {
            pos[0] = c.pos[0]; pos[1] = c.pos[1]; pos[2] = c.pos[2];
        }
    template <typename T>
        inline Vec3D<T>::Vec3D(T _x, T _y /*= T()*/, T _z /*= T()*/)
        {
            pos[0] = _x; pos[1] = _y; pos[2] = _z;
        }
    template <typename T> Vec3D<T>::~Vec3D(){}
    
    
    // Initializer
    template <typename T>
        inline void Vec3D<T>::setVec3Ds(T _x, T _y, T _z)
        {
            pos[0] = _x; pos[1] = _y; pos[2] = _z;
        }

    // Basic operator
    template <typename T>
        inline Vec3D<T> Vec3D<T>::operator +(const Vec3D<T> & c) const
        {
            Vec3D<T> coord;
            coord.pos[0] = pos[0]+c.pos[0];
            coord.pos[1] = pos[1]+c.pos[1];
            coord.pos[2] = pos[2]+c.pos[2];
            return coord;
        }

    template <typename T>
        inline Vec3D<T> Vec3D<T>::operator -(const Vec3D<T> & c) const
        {
            Vec3D<T> coord;
            coord.pos[0] = pos[0]-c.pos[0];
            coord.pos[1] = pos[1]-c.pos[1];
            coord.pos[2] = pos[2]-c.pos[2];
            return coord;
        }

    template <typename T>
        inline Vec3D<T> Vec3D<T>::operator *(T v) const
        {
            Vec3D<T> coord;
            coord.pos[0] = pos[0]*v;
            coord.pos[1] = pos[1]*v;
            coord.pos[2] = pos[2]*v;
            return coord;
        }

    template <typename T>
        inline Vec3D<T> Vec3D<T>::operator /(T v) const
        {
            Vec3D<T> coord;
            if(ALMOST_EQUAL_SMALL(v,0.0))
                {
                    std::cerr << "Vec3D<T> Error: Divided by zero!" << std::endl;
                    return coord;
                }
            coord.pos[0] = pos[0]/v;
            coord.pos[1] = pos[1]/v;
            coord.pos[2] = pos[2]/v;
            return coord;
        }

    template <typename T>
        inline Vec3D<T> Vec3D<T>::operator -() const
        {
            Vec3D<T> coord;
            coord.pos[0] = -1.0*pos[0];
            coord.pos[1] = -1.0*pos[1];
            coord.pos[2] = -1.0*pos[2];
            return coord;
        }

    //
    template <typename T>
        inline Vec3D<T>& Vec3D<T>::operator +=(const Vec3D<T> & c)
        {
            pos[0] += c.pos[0];
            pos[1] += c.pos[1];
            pos[2] += c.pos[2];
            return *this;
        }

    template <typename T>
        inline Vec3D<T>& Vec3D<T>::operator -=(const Vec3D<T> & c)
        {
            pos[0] -= c.pos[0];
            pos[1] -= c.pos[1];
            pos[2] -= c.pos[2];
            return *this;
        }

    template <typename T>
        inline Vec3D<T>& Vec3D<T>::operator *=(T v)
        {
            pos[0] *= v;
            pos[1] *= v;
            pos[2] *= v;
            return *this;
        }

    template <typename T>
        inline Vec3D<T>& Vec3D<T>::operator /=(T v)
        {
            if(ALMOST_EQUAL_SMALL(v,0.0))
                {
                    std::cerr << "Vec3D<T> Error: Divided by Zero" << std::endl;
                    return *this;
                }
            pos[0] /= v;
            pos[1] /= v;
            pos[2] /= v;
            return *this;
        }

    //
    template <typename T>
        inline Vec3D<T>& Vec3D<T>::operator =(const Vec3D<T> & c)
        {
            pos[0] = c.pos[0];
            pos[1] = c.pos[1];
            pos[2] = c.pos[2];
            return *this;
        }

    //
    template <typename T>
        inline bool Vec3D<T>::operator ==(const Vec3D<T> &c) const
        {
            if(pos[0]!=c.pos[0] || pos[1]!=c.pos[1] || pos[2]!=c.pos[2])
                return false;
            return true;
        }

    template <typename T>
        inline bool Vec3D<T>::operator !=(const Vec3D<T> &c) const
        {
            if(pos[0]!=c.pos[0] || pos[1]!=c.pos[1] || pos[2]!=c.pos[2])
                return true;
            return false;
        }

    //
    template <typename T>
        inline T& Vec3D<T>::operator [](size_t i)
        {
            return pos[i];
        }

    template <typename T>
        inline T Vec3D<T>::operator [](size_t i) const
        {
            return pos[i];
        }

    // Advanced operator
    template <typename T>
        inline T dot(const Vec3D<T>& c1, const Vec3D<T>& c2)
        {
            return (c1.pos[0]*c2.pos[0] + c1.pos[1]*c2.pos[1] + c1.pos[2]*c2.pos[2]);
        }

    template <typename T>
        inline T angle(const Vec3D<T>& c1, const Vec3D<T>& c2)
        {
            Vec3D<T> tmp_c1(c1), tmp_c2(c2);
            tmp_c1.normalize(); 
            tmp_c2.normalize();

            T dot_ = dot(tmp_c1, tmp_c2);
            if(dot_ < -1.0)
                dot_ = -1.0;
            else if(dot_ > 1.0)
                dot_ = 1.0;
    
            return acos(dot_);
        }

    template <typename T>
        inline Vec3D<T> cross(const Vec3D<T>& c1, const Vec3D<T>& c2)
        {
            Vec3D<T> c;
            c.pos[0] = c1.pos[1]*c2.pos[2] - c2.pos[1]*c1.pos[2];
            c.pos[1] = c2.pos[0]*c1.pos[2] - c1.pos[0]*c2.pos[2];
            c.pos[2] = c1.pos[0]*c2.pos[1] - c2.pos[0]*c1.pos[1];
            return c;
        }
    
    template <typename T>
        inline T Vec3D<T>::abs() const
        {
            return sqrt(pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
        }

    template <typename T>
        inline T Vec3D<T>::sqrabs() const
        {
            return (pos[0]*pos[0] + pos[1]*pos[1] + pos[2]*pos[2]);
        }
    
    template <typename T>
        inline Vec3D<T> Vec3D<T>::unit() const
        {
            Vec3D<T> c;
            T length = this->abs();
            if(ALMOST_EQUAL_SMALL(length,0.0))
                {
                    std::cerr << "Vec3D<T> Error: unit, divided by zero" << std::endl;
                    return *this;
                }
            c.pos[0] = pos[0]/length;
            c.pos[1] = pos[1]/length;
            c.pos[2] = pos[2]/length;
            return c;
        }

    template <typename T>
        inline bool Vec3D<T>::normalize()
        {
            T length = this->abs();
            if(ALMOST_EQUAL_SMALL(length,0.0))
                {
                    std::cerr << "Vec3D<T> Error: unit, divided by zero" << std::endl;
                    return false;
                }
            pos[0] /= length;
            pos[1] /= length;
            pos[2] /= length;
            return true;
        }

    template <typename T>
        inline T Vec3D<T>::x() const
        {
            return pos[0];
        }
    
    template <typename T>
        inline T Vec3D<T>::y() const
        {
            return pos[1];
        }
    
    template <typename T>
        inline T Vec3D<T>::z() const
        {
            return pos[2];
        }

    
} // end namespace
#endif
