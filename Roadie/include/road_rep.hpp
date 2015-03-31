#ifndef _ROAD_REP_HPP_
#define _ROAD_REP_HPP_

#include "libroad_common.hpp"

struct road_rep
{
    virtual float length     ()                       const = 0;
    virtual void  point      (float t, vec3f   &pt)   const = 0;
    virtual void  frame      (float t, mat3x3f &fr)   const = 0;
    virtual void  point_frame(float t, mat4x4f &ptfr) const = 0;

    virtual ~road_rep() {};

    // some way to draw
    // some way to serialize
};
#endif
