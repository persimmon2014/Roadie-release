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

#include "rtree-impl.hpp"
#endif
