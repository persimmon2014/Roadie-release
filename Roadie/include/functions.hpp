#ifndef _FUNCTIONS_HPP_
#define _FUNCTIONS_HPP_

namespace cog
{
    template <class T>
    inline T fade(T t) { return t * t * t * (t * (t * 6 - 15) + 10); }

    template <class T>
    inline T lerp(T t, T a, T b) { return a + t * (b - a); }

    template <class T>
    inline T bilerp(T t0, T t1, T a00, T a10, T a01, T a11) {
	return lerp(t0,
                lerp(t1, a00, a01),
                lerp(t1, a10, a11));
    }

    template <class T>
    inline T trilerp(T t0, T t1, T t2,
                     T a000, T a100, T a010, T a110,
                     T a001, T a101, T a011, T a111) {
        return lerp(t0,
                    lerp(t1,
                         lerp(t2, a000, a001),
                         lerp(t2, a010, a011)),
                    lerp(t1,
                         lerp(t2, a100, a101),
                         lerp(t2, a110, a111)));
    }

    inline float ngrad(int hash, float x, float y, float z)
    {
        int h = hash & 15;                      // CONVERT LO 4 BITS OF HASH CODE
        float u = h<8 ? x : y,                 // INTO 12 NGRADIENT DIRECTIONS.
            v = h<4 ? y : h==12||h==14 ? x : z;
        return ((h&1) == 0 ? u : -u) + ((h&2) == 0 ? v : -v);
    }

    template <class T>
    inline T clamp(T x, T a, T b)
    {
        if(x < a)
            return a;
        else if(x > b)
            return b;
        else
            return x;
    }

    template <class T>
    inline T noise(T x, T y, T z)
    {
        static int p[512], permutation[] = { 151,160,137,91,90,15,
                                             131,13,201,95,96,53,194,233,7,225,140,36,103,30,69,142,8,99,37,240,21,10,23,
                                             190, 6,148,247,120,234,75,0,26,197,62,94,252,219,203,117,35,11,32,57,177,33,
                                             88,237,149,56,87,174,20,125,136,171,168, 68,175,74,165,71,134,139,48,27,166,
                                             77,146,158,231,83,111,229,122,60,211,133,230,220,105,92,41,55,46,245,40,244,
                                             102,143,54, 65,25,63,161, 1,216,80,73,209,76,132,187,208, 89,18,169,200,196,
                                             135,130,116,188,159,86,164,100,109,198,173,186, 3,64,52,217,226,250,124,123,
                                             5,202,38,147,118,126,255,82,85,212,207,206,59,227,47,16,58,17,182,189,28,42,
                                             223,183,170,213,119,248,152, 2,44,154,163, 70,221,153,101,155,167, 43,172,9,
                                             129,22,39,253, 19,98,108,110,79,113,224,232,178,185, 112,104,218,246,97,228,
                                             251,34,242,193,238,210,144,12,191,179,162,241, 81,51,145,235,249,14,239,107,
                                             49,192,214, 31,181,199,106,157,184, 84,204,176,115,121,50,45,127, 4,150,254,
                                             138,236,205,93,222,114,67,29,24,72,243,141,128,195,78,66,215,61,156,180
        };
        static int done = 0;
        if(!done)
        {
            for (int i=0; i < 256 ; i++) p[256+i] = p[i] = permutation[i];
            done = 1;
        }
        int X = (int)std::floor(x) & 255,                  // FIND UNIT CUBE THAT
            Y = (int)std::floor(y) & 255,                  // CONTAINS POINT.
            Z = (int)std::floor(z) & 255;
        x -= std::floor(x);                                // FIND RELATIVE X,Y,Z
        y -= std::floor(y);                                // OF POINT IN CUBE.
        z -= std::floor(z);
        T u = fade(x),                                // COMPUTE FADE CURVES
            v = fade(y),                                // FOR EACH OF X,Y,Z.
            w = fade(z);
        int A = p[X  ]+Y, AA = p[A]+Z, AB = p[A+1]+Z,      // HASH COORDINATES OF
            B = p[X+1]+Y, BA = p[B]+Z, BB = p[B+1]+Z;      // THE 8 CUBE CORNERS,

        return lerp(w, lerp(v, lerp(u, ngrad(p[AA  ], x  , y  , z   ),  // AND ADD
                                    ngrad(p[BA  ], x-1, y  , z   )), // BLENDED
                            lerp(u, ngrad(p[AB  ], x  , y-1, z   ),  // RESULTS
                                 ngrad(p[BB  ], x-1, y-1, z   ))),// FROM  8
                    lerp(v, lerp(u, ngrad(p[AA+1], x  , y  , z-1 ),  // CORNERS
                                 ngrad(p[BA+1], x-1, y  , z-1 )), // OF CUBE
                         lerp(u, ngrad(p[AB+1], x  , y-1, z-1 ),
                              ngrad(p[BB+1], x-1, y-1, z-1 ))));

    }

    template <class T>
    inline T bias(T x, T b)
    {
        static const T INVLOGP5 = 1.0/std::log(0.5);
        return std::pow(x, std::log(b)*INVLOGP5);
    }

    template <class T>
    inline T gain(T x, T b)
    {
        if (x < 0.5)
            return bias(2.0*x, 1.0-b)*0.5;
        else
            return 1.0 - bias(2.0-2.0*x, 1.0-b)*0.5;
    }

    inline int randint(int low, int high)
    {
        return static_cast<int>((high-low)*((float)rand())/(float)RAND_MAX + low);
    }

    inline float randfloat(float low, float high)
    {
        return (high-low)*((float)rand())/(float)RAND_MAX + low;
    }

    inline void linear(const float * in,  float * out)
    {
        out[0] = in[0];
        out[1] = in[1];
    }

    inline void sinusoidal(const float * in,  float * out)
    {
        out[0] = sin(in[0]);
        out[1] = sin(in[1]);
    }

    inline void spherical(const float * in,  float * out)
    {
        float r = 1.0/(in[0]*in[0]+in[1]*in[1]);

        out[0] = in[0]*r;
        out[1] = in[1]*r;
    }

    inline void swirl(const float * in,  float * out)
    {
        float r = 1.0/(in[0]*in[0]+in[1]*in[1]);
        float theta = atan2(in[1],in[0]);

        out[0] = r*cos(theta+r);
        out[1] = r*sin(theta+r);
    }

    inline void horseshoe(const float * in,  float * out)
    {

        float r = 1.0/(in[0]*in[0]+in[1]*in[1]);
        float theta = atan2(in[1],in[0]);

        out[0] = r*cos(2*theta);
        out[1] = r*sin(2*theta);
    }

    template <class T>
    inline T turbulence(T x, T y, T z, int N)
    {
        T res = 0.0;
        T fact = 1;
        for( int i = 0; i < N; i++)
        {
            res += noise(fact*x, fact*y, fact*z)/fact;
            fact *= 2.0;
        }
        return res;
    }
}
#endif
