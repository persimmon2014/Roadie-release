#ifndef __IM_HEIGHTFIELD_HPP__
#define __IM_HEIGHTFIELD_HPP__

#include "functions.hpp"

struct im_heightfield;

#include "hwm_network.hpp"
#include "osm_network.hpp"
#include <fstream>

struct im_heightfield
{
    im_heightfield(float *p, const vec2i &d, float zb, float zs)
        : pix(p), dim(d), zbase(zb), zscale(zs)
    {}

    void set_noise(const vec2f &origin_offs, const float scale, const float z)
    {
        for(int j = 0; j < dim[1]; ++j)
            for(int i = 0; i < dim[0]; ++i)
                pix[i + j * dim[0]] = cog::noise(i*scale+origin_offs[0],
                                                 j*scale+origin_offs[1],
                                                 z);
        normalize();
    }

    void set_turbulence(const vec2f &origin_offs, const float scale, const float z, int N, float pow)
    {
        for(int j = 0; j < dim[1]; ++j)
            for(int i = 0; i < dim[0]; ++i)
                pix[i + j * dim[0]] = cog::turbulence(i*scale+origin_offs[0],
                                                      j*scale+origin_offs[1],
                                                      z, N);
        normalize();
        power(pow);
    }

    bool dump_raw(const std::string &filename) const
    {
        if(dim[0] != dim[1] || ((dim[0] - 1) & (dim[0] - 2) ))
            return false;

        unsigned short       *z   = new unsigned short[dim[0]*dim[1]];
        const unsigned short  max = static_cast<unsigned short>(-1);
        for(int j = 0; j < dim[1]; ++j)
            for(int i = 0; i < dim[0]; ++i)
            {
                if(pix[i + j * dim[0]] > 1.0)
                    z[i + j*dim[0]] = max;
                else if(pix[i + j * dim[0]] < 0.0)
                    z[i + j*dim[0]] = 0;
                else
                    z[i + j*dim[0]] = static_cast<unsigned short>(pix[i + j * dim[0]]*max);
            }
        std::ofstream of(filename.c_str(), std::ios::binary);
        of.write(reinterpret_cast<const char*>(z),
                 static_cast<std::streamsize>(sizeof(unsigned short)*dim[0]*dim[1]));
        return true;
    };

    void normalize()
    {
        float min = pix[0];
        float max = pix[0];
        for(int j = 0; j < dim[1]; ++j)
            for(int i = 0; i < dim[0]; ++i)
            {
                min = std::min(pix[i + j * dim[0]], min);
                max = std::max(pix[i + j * dim[0]], max);
            }
        const float scale = 1.0/(max-min);
        for(int j = 0; j < dim[1]; ++j)
            for(int i = 0; i < dim[0]; ++i)
                pix[i + j * dim[0]] = (pix[i + j * dim[0]]-min)*scale;
    }

    void power(const float pow)
    {
        for(int j = 0; j < dim[1]; ++j)
            for(int i = 0; i < dim[0]; ++i)
            {
                pix[i + j * dim[0]] = std::pow(pix[i + j * dim[0]], pow);
            }
    }

    float lookup(const vec2f &xy) const
    {
        const vec2f local((xy[0]-origin[0])/spacing[0],
                          (xy[1]-origin[1])/spacing[1]);

        vec2i base(static_cast<int>(std::floor(local[0])),
                   static_cast<int>(std::floor(local[1])));

        const vec2f remainder(local-base);

        if(base[0] < 0)         base[0]  = 0;
        if(base[1] < 0)         base[1]  = 0;
        if(base[0] > dim[0]-1)  base[0]  = dim[0]-1;
        if(base[1] > dim[1]-1)  base[1]  = dim[1]-1;

        vec2i top_c(base + 1);
        if(top_c[0] > dim[0]-1) top_c[0] = dim[0]-1;
        if(top_c[1] > dim[1]-1) top_c[1] = dim[1]-1;

        assert(base[0]  +  base[1]*dim[0] < dim[0]*dim[1]);
        assert(top_c[0] +  base[1]*dim[0] < dim[0]*dim[1]);
        assert(top_c[0] +  top_c[1]*dim[0] < dim[0]*dim[1]);
        assert(base[0]  +  top_c[1]*dim[0] < dim[0]*dim[1]);
        const float sw(pix[base[0]  +  base[1]*dim[0]]);
        const float se(pix[top_c[0] +  base[1]*dim[0]]);
        const float ne(pix[top_c[0] + top_c[1]*dim[0]]);
        const float nw(pix[base[0]  + top_c[1]*dim[0]]);

        return zbase + zscale*((sw*(1-remainder[0]) + se*remainder[0])*(1-remainder[1]) +
                               (nw*(1-remainder[0]) + ne*remainder[0])*remainder[1]);
    }

    vec3f vlookup(const vec2f &xy) const
    {
        return vec3f(xy[0], xy[1], lookup(xy));
    }

    vec3f vlookup(const vec3f &xy) const
    {
        return vec3f(xy[0], xy[1], lookup(vec2f(xy[0], xy[1])));
    }

    std::vector<vec3f> displace_polyline(const std::vector<vec3f> &poly, const float min_length, const float fac_limit) const
    {
        std::vector<vec3f>                 res;
        std::vector<vec3f>                 to_add;
        std::vector<vec3f>::const_iterator input(poly.begin());

        res.push_back(vlookup(*input++));
        to_add.push_back(vlookup(*input++));

        vec3f split;
        while(true)
        {
            while(!to_add.empty())
            {
                while(max_diff(split, res.back(), to_add.back(), min_length, fac_limit))
                {
                    to_add.push_back(split);
                }
                res.push_back(to_add.back());
                to_add.pop_back();
            }

            if(input == poly.end())
                return res;

            to_add.push_back(vlookup(*input++));
        }
    }

    void project_vertices(std::vector<vertex> &vrts) const
    {
        for(vertex &v: vrts)
        {
            v.position = vlookup(v.position);
        }
    }

    float project_bb(const aabb3d &box) const
    {
        float min_height = std::numeric_limits<float>::max();
        float delta      = 0;
        for(int i = 0; i < 4; ++i)
        {
            const float f = lookup(vec2f(box.bounds[i&1][0], box.bounds[i%2][1]));
            if(f < min_height)
                delta     = f - box.bounds[0][2];
        }

        return delta;
    }

    void displace_shapes(osm::shape_t &s, const float min_length, const float fac_limit, osm::network &net) const
    {
        osm::shape_t           res;
        osm::shape_t           to_add;
        osm::shape_t::iterator input(s.begin());

        (*input)->xy = vlookup((*input)->xy);
        res.push_back(*input);
        ++input;

        (*input)->xy = vlookup((*input)->xy);
        to_add.push_back(*input);
        ++input;

        vec3f split;
        while(true)
        {
            while(!to_add.empty())
            {
                while(max_diff(split, res.back()->xy, to_add.back()->xy, min_length, fac_limit))
                {
                    to_add.push_back(net.add_node(split, res.back()->is_overpass && to_add.back()->is_overpass));
                }

                res.push_back(to_add.back());
                to_add.pop_back();
            }

            if(input == s.end())
            {
                s.swap(res);
                return;
            }

            (*input)->xy = vlookup((*input)->xy);
            to_add.push_back(*input);
            ++input;
        }
    }

    bool max_diff(vec3f &best, const vec3f &v0, const vec3f &v1, const float min_length, const float fac_limit) const
    {
        const vec3f dvec(v1-v0);
        const float len(length(dvec));
        if(len <= 2*min_length)
            return false;
        const float inv_len = 1.0f/len;
        const vec3f nvec(dvec*inv_len);

        const float dt       = 0.5*std::min(spacing[0], spacing[1]);
        float       best_fac = fac_limit;
        bool        best_set = false;
        for(float t = std::max(dt, min_length); t < len-min_length; t += dt)
        {
            const vec3f mid(v0 + nvec*t);
            const vec3f cand(vlookup(vec2f(mid[0], mid[1])));
            float       cand_fac(std::abs(cand[2] - mid[2])*inv_len);
            if(cand_fac > best_fac)
            {
                best_set = true;
                best     = cand;
                best_fac = cand_fac;
            }
        }
        return best_set;
    };

    void make_mesh(std::vector<vertex> &vrts, std::vector<vec3u> &faces, bool scale=true) const
    {
        for(int j = 0; j < dim[1]; ++j)
        {
            for(int i = 0; i < dim[0]; ++i)
            {
                const vec3f pos(i, j,
                                pix[i + j * dim[0]]);

                const vec2f uv(i/static_cast<float>(dim[0]-1),
                               j/static_cast<float>(dim[1]-1));

                vrts.push_back(vertex(pos, vec3f(0), uv));
            }
        }

        if(scale)
            for(int j = 0; j < dim[1]; ++j)
            {
                for(int i = 0; i < dim[0]; ++i)
                {
                    vec2f &v2d = sub<0,2>::vector(vrts[i + j * dim[0]].position);
                    v2d = v2d*spacing + origin;
                    vrts[i + j * dim[0]].position[2] = zbase + zscale*vrts[i + j * dim[0]].position[2];
                }
            }

        for(int j = 0; j < dim[1]; ++j)
        {
            for(int i = 0; i < dim[0]; ++i)
            {
                const vec3f &current(vrts[i+j*dim[0]].position);
                vec3f xback(0), yback(0), xnext(0), ynext(0);

                if(i > 0)        xback = vec3f(vrts[i-1+ j   *dim[0]].position - current);
                if(j > 0)        yback = vec3f(vrts[i  +(j-1)*dim[0]].position - current);
                if(i < dim[0]-1) xnext = vec3f(vrts[i+1+ j   *dim[0]].position - current);
                if(j < dim[1]-1) ynext = vec3f(vrts[i  +(j+1)*dim[0]].position - current);

                vrts[i+j*dim[1]].normal = tvmet::cross(0.5*(ynext-yback), 0.5*(xnext-xback));
                vrts[i+j*dim[1]].normal /= length(vrts[i+j*dim[0]].normal);
            }
        }

        for(int j = 1; j < dim[1]; ++j)
        {
            for(int i = 1; i < dim[0]; ++i)
            {
                faces.push_back(vec3u(i-1 + j*dim[0],
                                      i   + j*dim[0],
                                      i-1 + (j-1)*dim[0]));
                faces.push_back(vec3u(i   + j    *dim[0],
                                      i   + (j-1)*dim[0],
                                      i-1 + (j-1)*dim[0]));
            }
        }
    }

    float *pix;
    vec2i  dim;

    float zbase;
    float zscale;

    vec2f origin;
    vec2f spacing;
};

#endif
