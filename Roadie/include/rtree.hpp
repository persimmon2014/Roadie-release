/*
 * rtree.hpp
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

#ifndef _RTREE_HPP_
#define _RTREE_HPP_

#include <limits>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <string>
#include "hilbert.hpp"

template <typename REAL_T, int D, int MIN, int MAX>
struct rtree
{
    struct node;

    typedef REAL_T real_t;
    typedef size_t idx_t;
    enum { DIMENSION = D };
    enum { m = MIN };
    enum { M = MAX };

    struct aabb
    {
        aabb();
        aabb(real_t xlow, real_t xhigh,
             real_t ylow, real_t yhigh);

        enum {dimension = DIMENSION};

        real_t area() const;
        aabb   funion(const aabb &o) const;
        bool   overlap(const aabb &o) const;
        void   center(real_t c[DIMENSION]) const;
        void   enclose_point(real_t x, real_t y);
        void   enclose_point(real_t x, real_t y, real_t z);

        real_t bounds[2][DIMENSION];
    };

    struct entry
    {
        entry();
        entry(const aabb &r, idx_t i);
        entry(node *c);

        idx_t as_item() const;
        node *as_node();
        const node *as_node() const;

        void recompute_rect();

        aabb rect;
        idx_t item;
    };

    struct node
    {
        static node *new_leaf(node *parent);
        static node *new_interior(node *parent);
        static void  destroy(node *n);

        node();
        node(bool is_leaf, node *p);
        ~node();

        bool leafp() const;
        bool rootp() const;
        int height() const;
        int  depth() const;
        size_t count_nodes(bool do_leaf) const;
        bool check(int depth, int leafdepth) const;

        int add_child(entry e);
        int add_entry(entry e);

        int remove_child(const node *c);
        int remove_entry(idx_t i);

        int child_index(const node *n) const;
        int entry_index(idx_t item) const;

        bool  leaf;
        int   nchildren;
        entry children[M];

        node *parent;
    };

    struct query_result_gen
    {
        query_result_gen(const node *start, const aabb &r);

        bool next(idx_t &res);

        enum {NOT_STARTED, RUNNING, DONE};

        const aabb               &rect;
        std::vector<const node*>  stack;
        const node               *current;
        int                       child;
        int                       state;
    };

    static rtree *hilbert_rtree(const std::vector<entry> &leaves);

    rtree();
    ~rtree();

    bool               check() const;
    int                height() const;
    size_t             count_nodes(bool do_leaf) const;
    std::vector<idx_t> query(const aabb &rect) const;
    query_result_gen   make_query_gen(const aabb &rect) const;
    void               insert(entry &e, bool leafp=true);
    node              *choose_node(const entry &e, const int e_height) const;
    void               adjust_tree(node *l, node *pair);
    node              *find_leaf(const entry &e) const;
    void               remove(const entry &e);
    void               condense_tree(node *l);
    void               split_node(node *n, node *&nn, entry &e) const;

    void dump(const char *filename) const;

    static void quad_split(node *out_n1, node *out_n2, entry in_e[M+1], int &nentries);
    static std::pair<int, int> quad_pick_seeds(const entry in_e[M+1], const int nentries);
    static int quad_pick_next(const aabb &r1, const aabb &r2, const entry in_e[M+1], const int nentries);

    node *root;
};

typedef rtree<float, 2, 85, 170>       rtree2d;
typedef rtree2d::aabb                  aabb2d;
typedef rtree<float, 3, 85, 170>::aabb aabb3d;

template <typename REAL_T, int D, int MIN, int MAX>
struct static_rtree
{
    struct node;

    typedef REAL_T real_t;
    typedef size_t idx_t;
    enum { DIMENSION = D };
    enum { m = MIN };
    enum { M = MAX };

    typedef typename rtree<REAL_T, D, MIN, MAX>::aabb aabb;

    struct entry
    {
        entry();

        idx_t as_item() const;
        const node *as_node(const node *base) const;

        aabb rect;
        idx_t item;
    };

    struct node
    {
        node();
        node(bool is_leaf, size_t p);
        ~node();

        bool leafp() const;
        bool rootp() const;
        int height() const;
        int  depth() const;
        size_t count_nodes(bool do_leaf) const;
        bool check(int depth, int leafdepth) const;

        bool  leaf;
        int   nchildren;
        entry children[M];

        size_t parent;
    };

    static_rtree(const char *filename);
    ~static_rtree();

    bool               check() const;
    int                height() const;
    size_t             count_nodes(bool do_leaf) const;
    std::vector<idx_t> query(const aabb &rect) const;

    node   *root;
    void   *map_root;
    size_t  map_bytes;
};

typedef static_rtree<float, 2, 85, 170> static_rtree2d;

rtree2d::aabb random_rect2d();

#include "rtree_impl.hpp"
#endif
