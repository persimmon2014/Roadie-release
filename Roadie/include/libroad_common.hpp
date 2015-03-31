#ifndef _LIBROAD_COMMON_HPP_
#define _LIBROAD_COMMON_HPP_

const char *libroad_package_string();

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <vector>
#include <map>
#include <algorithm>
#include <iostream>
#include <fstream>

#ifdef _MSC_VER
#include <functional>
#include <unordered_map>

inline double drand48()
{
	return rand()/(double)RAND_MAX;
}

inline float copysign(const float val, const float sign)
{
	return sign == 0.0f ? val : val*sign/std::abs(sign);
}





#define xunused

#define xisfinite _finite
#define isnan     _isnan

#define M_LN2l static_cast<double>(M_LN2)

#else
#define HAVE_MMAP 1
#define xunused __attribute__ ((unused))
#define xisfinite std::isfinite
#include <tr1/functional>
#include <tr1/unordered_map>

#endif

#include <tvmet/Vector.h>
#include <tvmet/Matrix.h>

#undef None
#include <libxml++/libxml++.h>

using std::tr1::hash;

typedef Glib::ustring str;


namespace std
{
    namespace tr1
    {
        template <>
        struct hash<const str>
        {
            size_t operator()(const Glib::ustring &str) const
            {
                hash<const char*> h;
                return h(str.c_str());
            }
        };
    }
}

template <class T>
struct strhash
{
    typedef std::map<const str, T> type;
};

typedef tvmet::Vector<double,      2>    vec2d;
typedef tvmet::Vector<float,       2>    vec2f;
typedef vec2f                            intervalf;
typedef tvmet::Vector<float,       3>    vec3f;
typedef tvmet::Vector<double,      3>    vec3d;
typedef tvmet::Vector<int,         3>    vec3i;
typedef tvmet::Vector<unsigned int,3>    vec3u;
typedef tvmet::Vector<int,         2>    vec2i;
typedef tvmet::Vector<size_t,      2>    vec2u;
typedef tvmet::Vector<float,       4>    vec4f;
typedef tvmet::Matrix<float,       3, 3> mat3x3f;
typedef tvmet::Matrix<float,       4, 4> mat4x4f;


inline int charStar_to_Int(char* pt) {
    return std::stoi(std::string(pt));
}

inline float charStar_to_Float(char* pt) {
    return std::stof(std::string(pt));
}



template <typename T>
inline float length2(const T& t1)
{
    return tvmet::dot(t1, t1);
}

template <typename T>
inline float length(const T& t1)
{
    return std::sqrt(length2(t1));
}

template <typename T>
inline float distance2(const T& t1, const T& t2)
{
    const T diff(t2-t1);
    return length2(diff);
}

template <typename T>
inline float distance(const T& t1, const T& t2)
{
    return std::sqrt(distance2(t1, t2));
}

inline float planar_distance(const vec3f &pt1, const vec3f &pt2)
{
    const vec2f diff(pt2[0] - pt1[0], pt2[1] - pt1[1]);
    return std::sqrt(tvmet::dot(diff, diff));
}

template <int B, int E>
struct sub
{
    enum { RESDIM = E - B};

    template <typename T>
    static inline tvmet::Vector<typename T::value_type, RESDIM> &vector(T &src)
    {
        assert(E <= T::Size);
        return *reinterpret_cast<tvmet::Vector<typename T::value_type, RESDIM>*>(src.data()+B);
    }

    template <typename T>
    static inline const tvmet::Vector<typename T::value_type, RESDIM> &vector(const T &src)
    {
        assert(E <= T::Size);
        return *reinterpret_cast<const tvmet::Vector<typename T::value_type, RESDIM>*>(src.data()+B);
    }
};

template <typename T, int MAXDIM, int DIM>
inline tvmet::Vector<T, MAXDIM> basis()
{
    assert(DIM < MAXDIM);
    tvmet::Vector<T, MAXDIM> res;
    sub<0, DIM>::vector(res)        = 0;
    res[DIM]                        = 1;
    sub<DIM+1, MAXDIM>::vector(res) = 0;
    return res;
}

template <typename T, int DIM>
inline T basis()
{
    assert(DIM < T::Size);
    T res;
    sub<0, DIM>::vector(res)         = 0;
    res[DIM]                         = 1;
    sub<DIM+1, T::Size>::vector(res) = 0;
    return res;
}

template <typename M, typename V>
inline V transform(const M &mat, const V &v)
{
    return sub<0, 3>::vector(vec4f(mat*vec4f(v[0], v[1], v[2], 1.0)));
}



#endif
