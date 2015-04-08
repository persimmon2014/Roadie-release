/*
 * arcball.hpp
 * Road Network Library
 *
 * Copyright (c) 2010-2016 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * David Wilkie, Jason Sewall, Weizi Li, Ming C. Lin
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RoadLib/>
 */

#ifndef _ARCBALL_HPP_
#define _ARCBALL_HPP_

#include <cstdlib>
#include <cstring>
#include <cmath>

void matmul(double res[16], const double left[16], const double right[16])
{
    memset(res, 0, 16*sizeof (double));

    for(unsigned char i = 0; i < 4; i++)
        for(unsigned char j = 0; j < 4; j++)
            for(unsigned char k = 0; k < 4; k++)
                res[4*i+j] += left[4*i+k] * right[4*k+j];
}

class arcball
{
public:
    arcball()
    {
        reset();
    }

    void get_click(double x, double y, double scale=1.0, bool update=false)
    {
        memcpy(last_, current_, 3*sizeof (double));

        current_[0] = x;
        current_[1] = y;
        current_[2] = 0.0;

        mat_current_ = !update;
        scale_ = scale;

        double length =
            current_[0]*current_[0]+
            current_[1]*current_[1];

        if(length > 1.0)
        {
            length = 1.0/std::sqrt(length);
            current_[0] *= length;
            current_[1] *= length;
        }
        else
            current_[2] = std::sqrt(1.0-length);
    }

    void translate(const double tr[3])
    {
        double temp[16];
        memset(temp, 0, 16*sizeof (double));
        temp[4*0+0] = 1.0;
        temp[4*1+1] = 1.0;
        temp[4*2+2] = 1.0;
        temp[4*3+3] = 1.0;

        temp[4*3+0] = tr[0];
        temp[4*3+1] = tr[1];
        temp[4*3+2] = tr[2];

        double temp2[16];
        matmul(temp2, world_mat_, temp);
        memcpy(world_mat_, temp2, 16*sizeof (double));
    }

    void reset()
    {
        memset(current_, 0, 3*sizeof (double));
        memset(last_, 0, 3*sizeof (double));

        memset(world_mat_, 0, 16*sizeof (double));
        world_mat_[4*0+0] = 1.0;
        world_mat_[4*1+1] = 1.0;
        world_mat_[4*2+2] = 1.0;
        world_mat_[4*3+3] = 1.0;
        scale_ = 1.0;

        mat_current_ = true;
    }

    const double * world_mat() const
    {
        return world_mat_;
    }

    void get_rotation() const
    {
        if(!mat_current_)
        {
            double q[4] = {
                scale_*(current_[1] * last_[2] - current_[2] * last_[1]),
                scale_*(current_[2] * last_[0] - current_[0] * last_[2]),
                scale_*(current_[0] * last_[1] - current_[1] * last_[0]),
                scale_*(current_[0] * last_[0] +
                        current_[1] * last_[1] +
                        current_[2] * last_[2])
            };

            double temp[16];

            temp[4*0 + 0] = 1.0 - 2.0*q[1]*q[1] - 2.0*q[2]*q[2];
            temp[4*0 + 1] =       2.0*q[0]*q[1] - 2.0*q[3]*q[2];
            temp[4*0 + 2] =       2.0*q[0]*q[2] + 2.0*q[3]*q[1];
            temp[4*0 + 3] = 0.0;

            temp[4*1 + 0] =       2.0*q[0]*q[1] + 2.0*q[3]*q[2];
            temp[4*1 + 1] = 1.0 - 2.0*q[0]*q[0] - 2.0*q[2]*q[2];
            temp[4*1 + 2] =       2.0*q[1]*q[2] - 2.0*q[3]*q[0];
            temp[4*1 + 3] = 0.0;

            temp[4*2 + 0] =       2.0*q[0]*q[2] - 2.0*q[3]*q[1];
            temp[4*2 + 1] =       2.0*q[1]*q[2] + 2.0*q[3]*q[0];
            temp[4*2 + 2] = 1.0 - 2.0*q[0]*q[0] - 2.0*q[1]*q[1];
            temp[4*2 + 3] = 0.0;

            temp[4*3 + 0] = 0.0;
            temp[4*3 + 1] = 0.0;
            temp[4*3 + 2] = 0.0;
            temp[4*3 + 3] = 1.0;

            double temp2[16];
            matmul(temp2, world_mat_, temp);
            memcpy(world_mat_, temp2, 16*sizeof (double));

            mat_current_ = true;
        }
    }

private:
    mutable bool mat_current_;
    double current_[3];
    double last_[3];
    double scale_;
    mutable double world_mat_[16];
};

#endif
