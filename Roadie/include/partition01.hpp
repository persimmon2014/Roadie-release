#ifndef _PARTITION01_HPP_
#define _PARTITION01_HPP_

#include "libroad_common.hpp"

template <class T>
struct partition01 : public std::map<float, T>
{
    typedef std::map<float, T>                                  base;
    typedef typename std::map<float, T>::iterator               iterator;
    typedef typename std::map<float, T>::reverse_iterator       reverse_iterator;
    typedef typename std::map<float, T>::const_reverse_iterator const_reverse_iterator;
    typedef typename std::map<float, T>::const_iterator         const_iterator;
    typedef std::pair<const float, T>                           entry;
    typedef intervalf                                           interval_t; // intervalf is a type of vec2f

    partition01() : base() {}

    template <class C>
    void xml_read (C &n, xmlpp::TextReader &reader, const str &name);
    
    void xml_write(xmlpp::Element *elt, const str &name) const;

    // return the iterator of inserted element
    iterator insert(float x, const T &val)
    {
        return base::insert(std::make_pair(x, val)).first; 
    }

    iterator split_interval(iterator c, const interval_t &iv, const T &val)
    {
        const interval_t ci(containing_interval(c));
        // three cases:
        // iv is wholly contained in ci
        // iv abuts start of ci
        // iv abuts end of ci

        assert(ci[0] <= iv[0]);
        assert(iv[1] <= ci[1]);
	
        if(iv[0] == ci[0])
        {
            if(iv[1] < ci[1])
                insert(iv[1], c->second); 
            c->second = val;
            return c;
        }
        else
        {
            if(iv[1] < ci[1])
                insert(iv[1], c->second);
            return insert(iv[0], val);
        }
    }

    interval_t containing_interval(const_iterator c_this_itr) const
    {
        const_iterator c_next_itr(std::next(c_this_itr));

        return interval_t(c_this_itr->first,
                          (c_next_itr == this->end()) ? 1.0f : c_next_itr->first);
    }

    interval_t containing_interval(const_reverse_iterator c_this_itr) const
    {
        const_reverse_iterator c_next_itr(std::prev(c_this_itr));

        return interval_t(c_this_itr->first,
                          (c_this_itr == this->rbegin()) ? 1.0f : c_next_itr->first);
    }

    float interval_length(const_iterator c_this_itr) const
    {
        interval_t in(containing_interval(c_this_itr));
        return in[1] - in[0];
    }

    iterator find(float x)
    {
        if(empty())
            return end();

        iterator itr(this->upper_bound(x)); // upper_bound points to one element after the element contains key x
        return (itr == this->begin()) ? itr : --itr;
    }

    const_iterator find(float x) const
    {
        if(empty())
            return end();

        const_iterator itr(this->upper_bound(x));
        return (itr == this->begin()) ? itr : --itr;
    }

    iterator begin()
    {
        return base::begin();
    }

    const_iterator begin() const
    {
        return base::begin();
    }

    iterator end()
    {
        return base::end();
    }

    const_iterator end() const
    {
        return base::end();
    }

    bool empty() const
    {
        return base::empty();
    }

    iterator operator[](float x)
    {
        return find(x);
    }

    const_iterator operator[](float x) const
    {
        return find(x);
    }

    iterator find_rescale(float x, float &scale)
    {
        if(empty())
            return end();

        iterator itr = find(x);
        scale = (x-itr->first)/interval_length(itr);
        return itr;
    }

    const_iterator find_rescale(float x, float &scale) const
    {
        if(empty())
            return end();

        const_iterator itr = find(x);
        scale = (x - itr->first)/interval_length(itr);
        return itr;
    }
};
#endif
