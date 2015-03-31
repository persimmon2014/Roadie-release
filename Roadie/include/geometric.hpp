#ifndef _GEOMETRIC_HPP_
#define _GEOMETRIC_HPP_

#include <cstring>
#include "libroad_common.hpp"

template <typename T>
inline bool rm_invert(T A[16], T Ainv[16])
{
    T swap_space[4];

    for(int i = 0; i < 4; ++i)
    {
        int maxi = i;
        for(int j = i+1; j < 4; ++j)
            if(std::abs(A[4*j + i]) > std::abs(A[4*maxi + i]))
                maxi = j;
        if(A[4*maxi + i] == 0.0f)
            return false;
        if(i != maxi)
        {
            std::memcpy(swap_space,       &(A[4*maxi+ 0]), sizeof(T)*4);
            std::memcpy(&(A[4*maxi + 0]), &(A[4*i   + 0]), sizeof(T)*4);
            std::memcpy(&(A[4*i + 0]),    swap_space,      sizeof(T)*4);

            std::memcpy(swap_space,          &(Ainv[4*maxi + 0]), sizeof(T)*4);
            std::memcpy(&(Ainv[4*maxi + 0]), &(Ainv[4*i    + 0]), sizeof(T)*4);
            std::memcpy(&(Ainv[4*i    + 0]), swap_space,          sizeof(T)*4);
        }

        const T iinv = 1.0f/A[4*i + i];
        A[4*i + i] = 1.0f;
        for(int j = i+1; j < 4; ++j)
            A[4*i + j] *= iinv;
        for(int j = 0; j < 4; ++j)
            Ainv[4*i + j] *= iinv;

        for(int j = 0; j < 4; ++j)
            if(A[4*j + i] != 0.0f && i != j)
            {
                T temp = A[4*j + i];
                A[4*j + i] = 0.0f;
                for(int k = i+1; k < 4; ++k)
                    A[4*j + k] -= A[4*i + k]*temp;
                for(int k = 0; k < 4; ++k)
                    Ainv[4*j + k] -= Ainv[4*i + k]*temp;
            }
    }

    return true;
}

inline float ray_point_project(const vec3f &origin, const vec3f &dir, const vec3f &pt)
{
    // assumes dir is unit, so no divide by tvmet::dot(dir, dir);
    return tvmet::dot(dir, pt-origin);
}

inline vec3f ray_point_nearest(const vec3f &origin, const vec3f &dir, const vec3f &pt)
{
    return vec3f(origin + dir*ray_point_project(origin, dir, pt));
}

inline float ray_point_distance2(const vec3f &origin, const vec3f &dir, const vec3f &pt)
{
    const vec3f diff(pt - ray_point_nearest(origin, dir, pt));
    return tvmet::dot(diff, diff);
}

inline float ray_plane_intersection_param(const vec3f &origin, const vec3f &dir, const vec3f &normal, const float support)
{
    const float denom = tvmet::dot(dir, normal);
    assert(std::abs(denom) > 1e-8);

    return (-tvmet::dot(origin, normal) + support)/denom;
}

inline vec3f ray_plane_intersection(const vec3f &origin, const vec3f &dir, const vec3f &normal, const float support)
{
    return vec3f(origin + dir*ray_plane_intersection_param(origin, dir, normal, support));
}

inline void box_to_cscale(vec2f &center, float &scale,
                          const vec2f &low, const vec2f &high,
                          const vec2i &window)
{
    center = vec2f(low+high)/2;

    const vec2f dv(high-low);
    const float view_aspect  = static_cast<float>(window[0])/window[1];
    const float scene_aspect = dv[0]/dv[1];

    if(view_aspect > scene_aspect)
        scale = dv[0];
    else
        scale = dv[1];
}

inline void cscale_to_box(vec2f &low, vec2f &high,
                          const vec2f &center, const float &scale,
                          const vec2i &window)
{
    vec2f dv;
    if(window[0] > window[1])
    {
        dv[0] =  scale*window[0]/window[1];
        dv[1] =  scale;
    }
    else
    {
        dv[0] =  scale;
        dv[1] =  scale*window[1]/window[0];
    }
    low  = center - dv/2;
    high = center + dv/2;
}

inline vec2f world_point(const vec2i &input, const vec2f &center, const float &scale, const vec2i &window)
{
    vec2f lo, hi;
    cscale_to_box(lo, hi, center, scale, window);
    return vec2f(input*vec2f(1.0f/(window[0]-1),(1.0f/(window[1]-1)))*(hi-lo) + lo);
}

inline vec2f world_point(const vec2i &input, const vec2f &low, const vec2f &high, const vec2i &window)
{
    return vec2f(input*vec2f(1.0f/(window[0]-1), 1.0f/(window[1]-1))*(high-low) + low);
}

inline mat4x4f axis_angle_matrix(const float theta, const vec3f &axis)
{
    const float c = std::cos(theta);
    const float s = std::sin(theta);
    mat4x4f res;
    res(0, 0) = axis[0]*axis[0] + (1.0-axis[0]*axis[0])*c; res(0, 1) = axis[0]*axis[1]*(1.0-c) -  axis[2]*s;      res(0, 2) = axis[0]*axis[2]*(1.0-c) +  axis[1]*s;      res(0, 3) = 0;
    res(1, 0) = axis[0]*axis[1]*(1.0-c) +  axis[2]*s;      res(1, 1) = axis[1]*axis[1] + (1.0-axis[1]*axis[1])*c; res(1, 2) = axis[1]*axis[2]*(1.0-c) -  axis[0]*s;      res(1, 3) = 0;
    res(2, 0) = axis[0]*axis[2]*(1.0-c) -  axis[1]*s;      res(2, 1) = axis[1]*axis[2]*(1.0-c) +  axis[0]*s;      res(2, 2) = axis[2]*axis[2] + (1.0-axis[2]*axis[2])*c; res(2, 3) = 0;
    res(3, 0) = 0;                                         res(3, 1) = 0;                                         res(3, 2) = 0;                                         res(3, 3) = 1;
    return res;
}
#endif
