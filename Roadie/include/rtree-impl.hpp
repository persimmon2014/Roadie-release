#include "libroad_common.hpp"
#include <cstring>
#include <cstdlib>
#include <deque>
#if HAVE_MMAP
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <sys/stat.h>
#endif
#include <iostream>
#include <stdexcept>

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::aabb::aabb()
{
    for(int i = 0; i < DIMENSION; ++i)
    {
        bounds[0][i] =  std::numeric_limits<real_t>::max();
        bounds[1][i] = -std::numeric_limits<real_t>::max();
    }
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::aabb::aabb(const real_t xlow, const real_t xhigh,
                                       const real_t ylow, const real_t yhigh)
{
    assert(D == 2);

    bounds[0][0] = xlow;
    bounds[1][0] = xhigh;
    bounds[0][1] = ylow;
    bounds[1][1] = yhigh;
}

template <typename REAL_T, int D, int MIN, int MAX>
REAL_T rtree<REAL_T, D, MIN, MAX>::aabb::area() const
{
    real_t res(1);
    for(int i = 0; i < DIMENSION; ++i)
        res *= (bounds[1][i] - bounds[0][i]);

    return res;
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::aabb rtree<REAL_T, D, MIN, MAX>::aabb::funion(const typename rtree<REAL_T, D, MIN, MAX>::aabb &o) const
{
    aabb res;
    for(int i = 0; i < DIMENSION; ++i)
    {
        res.bounds[0][i] = std::min(bounds[0][i], o.bounds[0][i]);
        res.bounds[1][i] = std::max(bounds[1][i], o.bounds[1][i]);
    }

    return res;
}

template <typename REAL_T, int D, int MIN, int MAX>
bool rtree<REAL_T, D, MIN, MAX>::aabb::overlap(const typename rtree<REAL_T, D, MIN, MAX>::aabb &o) const
{
    for(int i = 0; i < DIMENSION; ++i)
        if(bounds[1][i] < o.bounds[0][i] ||
           bounds[0][i] > o.bounds[1][i])
            return false;
    return true;
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::aabb::center(real_t c[DIMENSION]) const
{
    for(int i = 0; i < DIMENSION; ++i)
        c[i] = (bounds[0][i] + bounds[1][i])/2;
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::aabb::enclose_point(const real_t x, const real_t y)
{
    //BOOST_STATIC_ASSERT(DIMENSION == 2);
    assert(DIMENSION == 2);
    if(x < bounds[0][0])
        bounds[0][0] = x;
    if(y < bounds[0][1])
        bounds[0][1] = y;
    if(x > bounds[1][0])
        bounds[1][0] = x;
    if(y > bounds[1][1])
        bounds[1][1] = y;
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::aabb::enclose_point(const real_t x, const real_t y, const real_t z)
{
    //BOOST_STATIC_ASSERT(DIMENSION == 3);
    assert(DIMENSION == 3);
    if(x < bounds[0][0])
        bounds[0][0] = x;
    if(y < bounds[0][1])
        bounds[0][1] = y;
    if(z < bounds[0][2])
        bounds[0][2] = z;
    if(x > bounds[1][0])
        bounds[1][0] = x;
    if(y > bounds[1][1])
        bounds[1][1] = y;
    if(z > bounds[1][2])
        bounds[1][2] = z;
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::entry::entry() : item(0)
{
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::entry::entry(const typename rtree<REAL_T, D, MIN, MAX>::aabb &r, const idx_t i) : rect(r), item(i)
{
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::entry::entry(typename rtree<REAL_T, D, MIN, MAX>::node *c) : item(reinterpret_cast<size_t>(c))
{
    recompute_rect();
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::idx_t rtree<REAL_T, D, MIN, MAX>::entry::as_item() const
{
    return item;
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::node *rtree<REAL_T, D, MIN, MAX>::entry::as_node()
{
    return reinterpret_cast<node*>(item);
}

template <typename REAL_T, int D, int MIN, int MAX>
const typename rtree<REAL_T, D, MIN, MAX>::node *rtree<REAL_T, D, MIN, MAX>::entry::as_node() const
{
    return reinterpret_cast<node*>(item);
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::entry::recompute_rect()
{
    const node *child = as_node();
    assert(child);

    assert(child->nchildren);

    rect = child->children[0].rect;
    for(int i = 1; i < child->nchildren; ++i)
        rect = rect.funion(child->children[i].rect);
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::node *rtree<REAL_T, D, MIN, MAX>::node::new_leaf(typename rtree<REAL_T, D, MIN, MAX>::node *p)
{
#ifndef _MSC_VER
    void *v;
    posix_memalign(&v, sysconf(_SC_PAGESIZE),sizeof(node));
    return new(v) node(true, p);
#else
	return new node(true, p);
#endif
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::node *rtree<REAL_T, D, MIN, MAX>::node::new_interior(typename rtree<REAL_T, D, MIN, MAX>::node *p)
{
#ifndef _MSC_VER
    void *v;
    posix_memalign(&v, sysconf(_SC_PAGESIZE),sizeof(node));
    return new(v) node(false, p);
#else
	return new node(false, p);
#endif
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::node::destroy(typename rtree<REAL_T, D, MIN, MAX>::node *c)
{
    c->nchildren = 0;
    c->~node();
    free(c);
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::node::node(const bool is_leaf, typename rtree<REAL_T, D, MIN, MAX>::node *p) : leaf(is_leaf), nchildren(0), parent(p)
{
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::node::~node()
{
    if(!leafp())
        for(int i = 0; i < nchildren; ++i)
        {
            children[i].as_node()->~node();
            free(children[i].as_node());
        }
}

template <typename REAL_T, int D, int MIN, int MAX>
bool rtree<REAL_T, D, MIN, MAX>::node::leafp() const
{
    return leaf;
}

template <typename REAL_T, int D, int MIN, int MAX>
bool rtree<REAL_T, D, MIN, MAX>::node::rootp() const
{
    return parent == 0;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::height() const
{
    if(leafp())
        return 0;

    int h = 0;
    for(int i = 0; i < nchildren; ++i)
        h = std::max(children[i].as_node()->height() + 1, h);
    return h;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::depth() const
{
    int d = 0;
    for(node *p = parent; p; p = p->parent)
        ++d;

    return d;
}

template <typename REAL_T, int D, int MIN, int MAX>
size_t rtree<REAL_T, D, MIN, MAX>::node::count_nodes(bool do_leaf) const
{
    size_t res = 0;
    if(!leafp())
        for(int i = 0; i < nchildren; ++i)
            res += 1 + children[i].as_node()->count_nodes(do_leaf);
    else if(do_leaf)
        res += nchildren;

    return res;
}

template <typename REAL_T, int D, int MIN, int MAX>
bool rtree<REAL_T, D, MIN, MAX>::node::check(const int depth, const int leafdepth) const
{
    assert(nchildren);

    if(!rootp())
    {
        assert(nchildren >= m);
        assert(nchildren <= M);

        if(leafp())
            assert(depth == leafdepth);
        else
        {
            for(int i = 0; i < nchildren; ++i)
                children[i].as_node()->check(depth + 1, leafdepth);
        }
    }
    return true;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::add_child(entry e)
{
    assert(!leafp());
    e.as_node()->parent = this;
    children[nchildren++] = e;
    return nchildren;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::add_entry(entry e)
{
    assert(leafp());
    children[nchildren++] = e;
    return nchildren;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::remove_child(const node *c)
{
    assert(!leafp());
    const int idx = child_index(c);
    assert(idx >= 0);
    children[idx] = children[--nchildren];
    return nchildren;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::remove_entry(const idx_t i)
{
    assert(leafp());
    int idx = entry_index(i);
    assert(idx >= 0);
    children[idx] = children[--nchildren];
    return nchildren;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::child_index(const typename rtree<REAL_T, D, MIN, MAX>::node *n) const
{
    for(int i = 0; i < nchildren; ++i)
        if(children[i].as_node() == n)
            return i;

    return -1;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::node::entry_index(const typename rtree<REAL_T, D, MIN, MAX>::idx_t item) const
{
    for(int i = 0; i < nchildren; ++i)
        if(children[i].item == item)
            return i;

    return -1;
}

template <typename REAL_T, int D, int MIN, int MAX>
 rtree<REAL_T, D, MIN, MAX>::query_result_gen::query_result_gen(const typename rtree<REAL_T, D, MIN, MAX>::node *start, const typename rtree<REAL_T, D, MIN, MAX>::aabb &r) : rect(r), current(0), child(0), state(NOT_STARTED)
{
    stack.push_back(start);
}

template <typename REAL_T, int D, int MIN, int MAX>
bool rtree<REAL_T, D, MIN, MAX>::query_result_gen::next(idx_t &res)
{
    switch(state)
    {
    case NOT_STARTED:
        state = RUNNING;
        break;
    case RUNNING:
        goto next_resume;
    case DONE:
    default:
        return false;
    };

    while(!stack.empty())
    {
        if(!current)
        {
            current = stack.back();
            stack.pop_back();
        }

        if(current->leafp())
        {
            for(child = 0; child < current->nchildren; ++child)
                if(rect.overlap(current->children[child].rect))
                {
                    res = current->children[child].as_item();
                    return true;
                next_resume:
                    ;
                }
        }
        else
        {
            for(int i = 0; i < current->nchildren; ++i)
                if(rect.overlap(current->children[i].rect))
                    stack.push_back(current->children[i].as_node());
        }
        current = 0;
    }

    return false;
}

struct order_perm
{
    order_perm(const std::vector<size_t> &o) : order(o)
    {}

    bool operator()(const size_t i, const size_t j) const
    {
        return order[i] < order[j];
    }

    const std::vector<size_t> &order;
};

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX> *rtree<REAL_T, D, MIN, MAX>::hilbert_rtree(const std::vector<entry> &leaves)
{
    rtree *res = new rtree();

    real_t bounds[D][D];
    for(int j = 0; j < D; ++j)
    {
        bounds[0][j] = std::numeric_limits<real_t>::max();
        bounds[1][j] = -std::numeric_limits<real_t>::max();
    }

    for(size_t i = 0; i < leaves.size(); ++i)
    {
        real_t c[D];
        leaves[i].rect.center(c);
        for(int j = 0; j < D; ++j)
        {
            if(c[j] < bounds[0][j])
                bounds[0][j] = c[j];
            else if(c[j] > bounds[1][j])
                bounds[1][j] = c[j];
        }
    }

    std::vector<size_t> order(leaves.size());
    for(size_t i = 0; i < leaves.size(); ++i)
    {
        real_t c[D];
        leaves[i].rect.center(c);
        for(int j = 0; j < D; ++j)
            c[j] = (c[j]-bounds[0][j])/(bounds[1][j]-bounds[0][j]);

        order[i] = hilbert::order(c[0], c[1]);
    }

    std::vector<size_t> perm(order.size());
    for(size_t i = 0; i < order.size(); ++i)
        perm[i] = i;

    std::sort(perm.begin(), perm.end(), order_perm(order));

    std::vector<node*> node_levels[2];
    node_levels[0].resize(static_cast<size_t>(std::ceil(perm.size()/static_cast<float>(M))));
    size_t             current_node = 0;
    node_levels[0][current_node]    = rtree2d::node::new_leaf(0);
    for(size_t i = 0; i < perm.size(); ++i)
    {
        if(node_levels[0][current_node]->nchildren >= M)
            node_levels[0][++current_node] = rtree2d::node::new_leaf(0);

        node_levels[0][current_node]->add_entry(leaves[perm[i]]);
    }

    node_levels[0].swap(node_levels[1]);
    while(node_levels[1].size() > 1)
    {
        node_levels[0].resize(static_cast<size_t>(std::ceil(node_levels[1].size()/static_cast<float>(M))));
        size_t current_node          = 0;
        node_levels[0][current_node] = rtree2d::node::new_interior(0);
        for(size_t i = 0; i < node_levels[1].size(); ++i)
        {
            if(node_levels[0][current_node]->nchildren >= M)
                node_levels[0][++current_node] = rtree2d::node::new_interior(0);
            entry ie(node_levels[1][i]);
            node_levels[0][current_node]->add_child(ie);
        }
        node_levels[0].swap(node_levels[1]);
    }

    assert(node_levels[1].size() == 1);
    res->root = node_levels[1][0];

    return res;
}


template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::rtree() : root(0)
{
}

template <typename REAL_T, int D, int MIN, int MAX>
rtree<REAL_T, D, MIN, MAX>::~rtree()
{
    if(root)
    {
        root->~node();
        free(root);
    }
}

template <typename REAL_T, int D, int MIN, int MAX>
bool rtree<REAL_T, D, MIN, MAX>::check() const
{
    if(!root)
        return true;

    return root->check(0, root->height());
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::height() const
{
    if(!root)
        return -1;

    return root->height();
}

template <typename REAL_T, int D, int MIN, int MAX>
size_t rtree<REAL_T, D, MIN, MAX>::count_nodes(bool do_leaf) const
{
    if(root)
        return 1 + root->count_nodes(do_leaf);
    else
        return 0;
}

template <typename REAL_T, int D, int MIN, int MAX>
std::vector<typename rtree<REAL_T, D, MIN, MAX>::idx_t> rtree<REAL_T, D, MIN, MAX>::query(const typename rtree<REAL_T, D, MIN, MAX>::aabb &rect) const
{
    std::vector<idx_t> res;
    query_result_gen gen(make_query_gen(rect));
    idx_t r;
    while(gen.next(r))
        res.push_back(r);

    return res;
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::query_result_gen rtree<REAL_T, D, MIN, MAX>::make_query_gen(const typename rtree<REAL_T, D, MIN, MAX>::aabb &rect) const
{
    return query_result_gen(root, rect);
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::insert(typename rtree<REAL_T, D, MIN, MAX>::entry &e, bool leaf)
{
    if(!root)
    {
        root = node::new_leaf(0);
        root->add_entry(e);
        return;
    }

    const int  e_height = leaf ? -1 : e.as_node()->height();
    node      *n        = choose_node(e, e_height);
    assert(n);

    node *nn = 0;
    if(n->nchildren == M)
        split_node(n, nn, e);
    else if(leaf)
        n->add_entry(e);
    else
        n->add_child(e);

    adjust_tree(n, nn);
    assert(check());
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::node* rtree<REAL_T, D, MIN, MAX>::choose_node(const typename rtree<REAL_T, D, MIN, MAX>::entry &e, const int e_height) const
{
    const int height = root->height();
    int       c      = 1;

    node *n = root;
    while(height - c > e_height)
    {
        real_t  min_a      = std::numeric_limits<real_t>::max();
        real_t  min_a_diff = min_a;
        node   *cand       = 0;
        for(int i = 0; i < n->nchildren; ++i)
        {
            const real_t a      = n->children[i].rect.area();
            const real_t a_diff = n->children[i].rect.funion(e.rect).area() - a;
            if(a_diff < min_a_diff || (a_diff == min_a_diff && a < min_a))
            {
                min_a = a;
                min_a_diff = a_diff;
                cand = n->children[i].as_node();
            }
        }
        assert(cand);
        n = cand;

        ++c;
    }
    return n;
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::adjust_tree(typename rtree<REAL_T, D, MIN, MAX>::node *l, typename rtree<REAL_T, D, MIN, MAX>::node *pair)
{
    node *n  = l;
    node *nn = pair;
    while(!n->rootp())
    {
        node *p          = n->parent;
        const int my_idx = p->child_index(n);
        assert(my_idx != -1);
        p->children[my_idx].recompute_rect();

        node *pp = 0;
        if(nn)
        {
            assert(nn->parent == p);
            assert(nn->nchildren);
            entry ie(nn);

            if(p->nchildren == M)
                split_node(p, pp, ie);
            else
                p->add_child(ie);
        }

        n = p;
        nn = pp;
    }

    if(nn)
    {
        assert(n == root);
        root       = node::new_interior(0);
        root->add_child(entry(n));
        root->add_child(entry(nn));
    }
}

template <typename REAL_T, int D, int MIN, int MAX>
typename rtree<REAL_T, D, MIN, MAX>::node *rtree<REAL_T, D, MIN, MAX>::find_leaf(const typename rtree<REAL_T, D, MIN, MAX>::entry &e) const
{
    std::vector<node*> stack;
    stack.push_back(root);

    while(!stack.empty())
    {
        node *c = stack.back();
        stack.pop_back();

        if(c->leafp())
        {
            for(int i = 0; i < c->nchildren; ++i)
                if(c->children[i].as_item() == e.item)
                    return c;
        }
        else
        {
            for(int i = 0; i < c->nchildren; ++i)
                if(e.rect.overlap(c->children[i].rect))
                    stack.push_back(c->children[i].as_node());
        }
    }
    return 0;
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::remove(const typename rtree<REAL_T, D, MIN, MAX>::entry &e)
{
    node *l = find_leaf(e);
    assert(l);

    l->remove_entry(e.item);

    condense_tree(l);

    if(!root->nchildren)
    {
        node::destroy(root);
        root = 0;
    }
    else if(!root->leafp() && root->nchildren == 1)
    {
        node *oldroot = root;
        root          = root->children[0].as_node();
        root->parent  = 0;

        node::destroy(oldroot);
    }

    assert(check());
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::condense_tree(typename rtree<REAL_T, D, MIN, MAX>::node *l)
{
    node *n = l;
    std::vector<node*> q;

    while(!n->rootp())
    {
        node      *p      = n->parent;
        const int  my_idx = p->child_index(n);
        assert(my_idx != -1);
        if(n->nchildren < m)
        {
            p->children[my_idx] = p->children[--p->nchildren];
            q.push_back(n);
        }
        else
        {
            assert(!p->leafp());
            p->children[my_idx].recompute_rect();
        }

        n = p;
    }

    while(!q.empty())
    {
        node *n = q.back();
        for(int i = 0; i < n->nchildren; ++i)
            insert(n->children[i], n->leafp());

        node::destroy(n);
        q.pop_back();
    }
}

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::split_node(typename rtree<REAL_T, D, MIN, MAX>::node *n, typename rtree<REAL_T, D, MIN, MAX>::node *&nn, entry &e) const
{
    entry old_e[M+1];
    memcpy(old_e, n->children, sizeof(entry)*n->nchildren);
    old_e[M]        = e;
    int   nchildren = n->nchildren+1;
    n->nchildren    = 0;

    assert(!nn);
    if(n->leafp())
        nn = node::new_leaf(n->parent);
    else
    {
        nn = node::new_interior(n->parent);
        e.as_node()->parent = n;
    }

    quad_split(n, nn, old_e, nchildren);
    assert(nchildren == 0);
    assert(n->nchildren >= m);
    assert(nn->nchildren >= m);
    assert(n->nchildren + nn->nchildren == M+1);

    if(!nn->leafp())
        for(int i = 0; i < nn->nchildren; ++i)
            nn->children[i].as_node()->parent = nn;

#ifndef DNDEBUG
    if(!n->leafp())
        for(int i = 0; i < n->nchildren; ++i)
            assert(n->children[i].as_node()->parent == n);
#endif
}

#if HAVE_MMAP

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::dump(const char *filename) const
{
    assert(root);
    const size_t nnodes(count_nodes(false));
    int fo = open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRUSR | S_IWUSR);
    lseek(fo, (1+nnodes)*sizeof(node)-1, SEEK_SET);
    char z = 0;
    write(fo, &z, 1);
    assert(fo >= 0);
    void *res = mmap(0, (1+nnodes)*sizeof(node), PROT_WRITE | PROT_READ, MAP_SHARED, fo, 0);
    assert(res);
    close(fo);

    node   *wpos   = reinterpret_cast<node*>(res) + 1;
    size_t  ncount = 0;

    std::deque<std::pair<const node*, size_t> > stack;
    stack.push_back(std::make_pair(root, ncount));
    ++ncount;
    while(!stack.empty())
    {
        std::pair<const node*, size_t> top = stack.front();
        stack.pop_front();

        node *dest = wpos + top.second;
        memcpy(dest, top.first, sizeof(node));
        if(!dest->leafp())
            for(int i = 0; i < dest->nchildren; ++i)
            {
                stack.push_back(std::make_pair(dest->children[i].as_node(), ncount));
                dest->children[i].item = ncount;
                ++ncount;
            }
    }

    munmap(res, (1+nnodes)*sizeof(node));
}

#endif

template <typename REAL_T, int D, int MIN, int MAX>
void rtree<REAL_T, D, MIN, MAX>::quad_split(typename rtree<REAL_T, D, MIN, MAX>::node *out_n1, typename rtree<REAL_T, D, MIN, MAX>::node *out_n2, entry in_e[M+1], int &nentries)
{
    assert(!out_n1->nchildren);
    assert(!out_n2->nchildren);
    std::pair<int, int> p(quad_pick_seeds(in_e, nentries));
    assert(p.first >= 0);
    assert(p.second >= 0);
    if(p.first > p.second)
        std::swap(p.first, p.second);

    out_n1->children[out_n1->nchildren++] = in_e[p.first];
    out_n2->children[out_n2->nchildren++] = in_e[p.second];

    aabb rect1(in_e[p.first ].rect);
    aabb rect2(in_e[p.second].rect);

    in_e[p.second] = in_e[--nentries];
    in_e[p.first]  = in_e[--nentries];

    while(true)
    {
        if(!nentries)
            return;

        if(out_n1->nchildren + nentries == m)
        {
            memcpy(out_n1->children + out_n1->nchildren,
                   in_e,
                   sizeof(entry)*nentries);
            out_n1->nchildren += nentries;
            nentries = 0;
            return;
        }
        if(out_n2->nchildren + nentries == m)
        {
            memcpy(out_n2->children + out_n2->nchildren,
                   in_e,
                   sizeof(entry)*nentries);
            out_n2->nchildren += nentries;
            nentries = 0;
            return;
        }

        const int i(quad_pick_next(rect1, rect2, in_e, nentries));
        assert(i >= 0);
        assert(i < nentries);

        const aabb   cand_rect1(rect1.funion(in_e[i].rect));
        const aabb   cand_rect2(rect2.funion(in_e[i].rect));
        const real_t adiff1(cand_rect1.area() - rect1.area());
        const real_t adiff2(cand_rect2.area() - rect2.area());
        if(adiff1 < adiff2 ||
           (adiff1 == adiff2 &&
            (rect1.area() < rect2.area() ||
             (rect1.area() == rect2.area()
              && out_n1->nchildren < out_n2->nchildren))))
        {
            out_n1->children[out_n1->nchildren++] = in_e[i];
            rect1 = cand_rect1;
        }
        else
        {
            out_n2->children[out_n2->nchildren++] = in_e[i];
            rect2 = cand_rect2;
        }

        in_e[i] = in_e[--nentries];
    }
}

template <typename REAL_T, int D, int MIN, int MAX>
std::pair<int, int> rtree<REAL_T, D, MIN, MAX>::quad_pick_seeds(const typename rtree<REAL_T, D, MIN, MAX>::entry ev[M+1], const int nentries)
{
    std::pair<int, int> best      = std::make_pair(-1,-1);
    real_t              largest_d = -std::numeric_limits<real_t>::max();

    for(int i = 0; i < nentries; ++i)
    {
        real_t ai = ev[i].rect.area();
        for(int j = i+1; j < nentries; ++j)
        {
            real_t aj   = ev[j].rect.area();
            real_t comp = ev[i].rect.funion(ev[j].rect).area();
            if(comp - aj - ai > largest_d)
            {
                largest_d = comp - aj - ai;
                best = std::make_pair(i, j);
            }
        }
    }
    return best;
}

template <typename REAL_T, int D, int MIN, int MAX>
int rtree<REAL_T, D, MIN, MAX>::quad_pick_next(const aabb &r1, const aabb &r2, const typename rtree<REAL_T, D, MIN, MAX>::entry ev_remainder[M+1], const int nev)
{
    int    best      = -1;
    real_t largest_d = -std::numeric_limits<real_t>::max();

    const real_t r1_a = r1.area();
    const real_t r2_a = r2.area();

    for(int i = 0; i < nev; ++i)
    {
        const real_t r1_grow = r1.funion(ev_remainder[i].rect).area() - r1_a;
        const real_t r2_grow = r2.funion(ev_remainder[i].rect).area() - r2_a;
        if(std::abs(r1_grow - r2_grow) > largest_d)
        {
            largest_d = std::abs(r1_grow - r2_grow);
            best = i;
        }
    }
    return best;
}

#if HAVE_MMAP

template <typename REAL_T, int D, int MIN, int MAX>
typename static_rtree<REAL_T, D, MIN, MAX>::idx_t static_rtree<REAL_T, D, MIN, MAX>::entry::as_item() const
{
    return item;
}

template <typename REAL_T, int D, int MIN, int MAX>
const typename static_rtree<REAL_T, D, MIN, MAX>::node *static_rtree<REAL_T, D, MIN, MAX>::entry::as_node(const typename static_rtree<REAL_T, D, MIN, MAX>::node *base) const
{
    return base + item;
}

template <typename REAL_T, int D, int MIN, int MAX>
bool static_rtree<REAL_T, D, MIN, MAX>::node::leafp() const
{
    return leaf;
}

template <typename REAL_T, int D, int MIN, int MAX>
static_rtree<REAL_T, D, MIN, MAX>::static_rtree(const char *filename)
{
    int         fi    = open(filename, O_RDONLY);
    assert(fi >= 0);
    struct stat fs;
    int         s_res = fstat(fi, &fs);
    if(s_res == -1)
        throw std::runtime_error("Stat of file failed!");
    map_bytes         = fs.st_size;
    map_root          = mmap(0, map_bytes, PROT_READ, MAP_SHARED, fi, 0);
    assert(map_root);
    close(fi);

    root = static_cast<node*>(map_root) + 1;
}

template <typename REAL_T, int D, int MIN, int MAX>
static_rtree<REAL_T, D, MIN, MAX>::~static_rtree()
{
    if(map_root)
        munmap(map_root, map_bytes);
}

template <typename REAL_T, int D, int MIN, int MAX>
std::vector<typename static_rtree<REAL_T, D, MIN, MAX>::idx_t> static_rtree<REAL_T, D, MIN, MAX>::query(const typename static_rtree<REAL_T, D, MIN, MAX>::aabb &rect) const
{
    std::vector<idx_t> res;
    std::vector<const node*> stack;

    stack.push_back(root);

    while(!stack.empty())
    {
        const node *top = stack.back();
        stack.pop_back();

        if(top->leafp())
        {
            for(int i = 0; i < top->nchildren; ++i)
                if(rect.overlap(top->children[i].rect))
                    res.push_back(top->children[i].as_item());
        }
        else
        {
            for(int i = 0; i < top->nchildren; ++i)
                if(rect.overlap(top->children[i].rect))
                    stack.push_back(top->children[i].as_node(root));
        }
    }
    return res;
}

#endif

inline rtree2d::aabb random_rect2d()
{
    rtree2d::aabb res;
    for(int i = 0; i < 2; ++i)
    {
        res.bounds[0][i] = 40*drand48() - 20;
        do
        {
            res.bounds[1][i] = drand48() + res.bounds[0][i];
        }
        while(res.bounds[1][i] <= res.bounds[0][i]);
    }
    return res;
}
