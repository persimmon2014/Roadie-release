#ifndef _HILBERT_HPP_
#define _HILBERT_HPP_

#include "libroad_common.hpp"

struct hilbert
{
    static size_t quadrant(const size_t x, const size_t y, const size_t w)
    {
        // const bool xbig = x >= w;
        // const bool ybig = y >= w;
        // return xbig*(3 - ybig) + !xbig*ybig;

        if(x >= w)
        {
            if( y >= w )
                return 2;

            return 3;
        }
        if( y >= w )
            return 1;
        return 0;
    }

    static void h_update(size_t &x, size_t &y, const size_t r, const size_t w)
    {
        switch(r)
        {
        case 0:
            std::swap(x, y);
            break;
        case 1:
            y -= w;
            break;
        case 2:
            x -= w;
            y -= w;
            break;
        case 3:
            {
                const size_t in_x = x;
                x = w - y - 1;
                y = 2*w - in_x - 1;
            }
            break;
        };
    }

    static size_t order(const float x, const float y)
    {
        return order_num(static_cast<size_t>(std::floor(x*((1LL << 24)-1))), static_cast<size_t>(std::floor(y*((1LL << 24)-1))), 24);
    }

    static double order_norm(const float x, const float y)
    {
        return order_num(static_cast<size_t>(std::floor(x*((1LL << 24)-1))), static_cast<size_t>(std::floor(y*((1LL << 24)-1))), 24)/static_cast<double>((1LL << (2*24)) -1);
    }

    static size_t order_num(size_t x, size_t y, const size_t n)
    {
        if(x == 0 && y == 0)
            return 0;

        size_t z    = 0;
        size_t rmin = static_cast<size_t>(std::floor(std::log(static_cast<double>(std::max(x, y)))/M_LN2)) + 1;
        size_t w    = 1LL << (rmin - 1);
        if((rmin & 1) != (n & 1))
            std::swap(x, y);

        while(rmin > 0)
        {
            const size_t r = quadrant(x, y, w);
            z = 4*z + r;
            h_update(x, y, r, w);
            --rmin;
            w >>= 1LL;
        }
        return z;
    }

    static void order_str(size_t x, size_t y, const size_t n, size_t *z, size_t &nchars)
    {
        nchars = 0;
        if(x == 0 && y == 0)
        {
            z[nchars++] = 0;
            return;
        }
        size_t rmin = static_cast<size_t>(std::floor(std::log(static_cast<double>(std::max(x, y)))/M_LN2l)) + 1;
        size_t w = 1LL << (rmin - 1);
        if((rmin & 1) != (n & 1))
            std::swap(x, y);

        while(rmin > 0)
        {
            const size_t r = quadrant(x, y, w);
            z[nchars++]    = r;
            h_update(x, y, r, w);
            --rmin;
            w            >>= 1LL;
        }
    }
};

#endif
