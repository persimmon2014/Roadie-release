#ifndef _XML_UTIL_HPP_
#define _XML_UTIL_HPP_

#include "libroad_common.hpp"
#include "sumo_network.hpp"
#include <libxml++/libxml++.h>
#include <libxml++/parsers/textreader.h>

inline int xml_line(const xmlpp::Node *n)
{
    if(n)
        return n->get_line();
    else
        return -1;
}

struct xml_error : public std::runtime_error
{
    xml_error(const xmlpp::TextReader &reader, const str &e) : std::runtime_error(e), line(xml_line(reader.get_current_node()))
    {
    }

    virtual const char *what() const throw()
    {
	return std::string("Line " + std::to_string(line) + ": " + std::runtime_error::what()).c_str();
    }

    const int line;
};

struct xml_eof : public std::exception
{
    xml_eof(const int l) : std::exception(), line(l)
    {
    }

    virtual const char *what() const throw()
    {
	return std::string("Reached unexpected EOF (started search at line: " + std::to_string(line) + ")").c_str();
    }

    const int line;
};

inline void read_skip_comment(xmlpp::TextReader &reader)
{
    const int line = xml_line(reader.get_current_node());

    do
    {
        if(!reader.read())
            throw xml_eof(line);
    }
    while(reader.get_node_type() == xmlpp::TextReader::Comment);
}

struct missing_attribute : std::exception
{
    missing_attribute(const xmlpp::TextReader &r, const str &en) : std::exception(), reader(r), eltname(en)
    {}

    virtual const char *what() const throw()
    {
	return std::string("Line " + std::to_string(xml_line(reader.get_current_node())) + ": no attribute " + eltname).c_str();
    }

    const xmlpp::TextReader &reader;
    const str &eltname;
};

template <typename T>
inline bool get_attribute(T &res,
                          xmlpp::TextReader &reader,
                          const str &eltname)
{
    str val(reader.get_attribute(eltname));


    if(!val.empty()) {
       std::istringstream str_in(val);
       str_in >> res;

       return true;
    }
    else
       return false;
}

template <>
inline bool get_attribute(bool &res,
                          xmlpp::TextReader &reader,
                          const str &eltname)
{
   str val(reader.get_attribute(eltname));

   if(!val.empty()) {
      if (val == "true")
         res = true;
      else
         res = false;

      return true;
   }
   else
      return false;
}


template <>
inline bool get_attribute(str &res, xmlpp::TextReader &reader, const str &eltname)
{
    res = reader.get_attribute(eltname);

    if(res.empty())
       return false;

    return true;
}

inline bool is_opening_element(const xmlpp::TextReader &reader, const str &name)
{

    return (reader.get_node_type() == xmlpp::TextReader::Element
            && reader.get_name() == name);
}

inline bool is_closing_element(const xmlpp::TextReader &reader, const str &name)
{
    return ((reader.get_node_type() == xmlpp::TextReader::EndElement ||
             reader.is_empty_element()) &&
            reader.get_name() == name);
}

struct xml_eof_opening : public std::exception
{
    xml_eof_opening(const int l, const str &o) : std::exception(), line(l), opening(o)
    {
    }

    virtual const char *what() const throw()
    {
	return std::string("Reached EOF while looking for opening " + opening + " element! (Started search at line: " + std::to_string(line) + ")").c_str();
    }

    const int line;
    const str &opening;
};

inline void read_to_open(xmlpp::TextReader &reader, const str &opentag)
{
    const int line = xml_line(reader.get_current_node());

    try
    {
        while(!is_opening_element(reader, opentag))
            read_skip_comment(reader);
    }
    catch(xml_eof &e)
    {
        throw xml_eof_opening(line, opentag);
    }
}

struct xml_eof_closing : public std::exception
{
    xml_eof_closing(const int l, const str &c) : std::exception(), line(l), closing(c)
    {
    }

    virtual const char *what() const throw()
    {
	return std::string("Reached EOF while looking for opening " + closing + " element! (Started search at line: " + std::to_string(line) + ")").c_str();
    }

    const int  line;
    const str &closing;
};

inline void read_to_close(xmlpp::TextReader &reader, const str &endtag)
{
    const int line = xml_line(reader.get_current_node());

    try
    {
        while(!is_closing_element(reader, endtag))
            read_skip_comment(reader);
    }
    catch(xml_eof &e)
    {
        throw xml_eof_closing(line, endtag);
    }
}

template <class closure, typename T>
inline void read_map_no_container(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name)
{
    while(1)
    {
        try
        {
            read_skip_comment(reader);
        }
        catch(xml_eof &e)
        {
            return;
        }

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
	  
	    // store nodes and ways into n.nodes and n.edge_hash
            if(reader.get_name() == item_name)
            {
                const str id(reader.get_attribute("id"));

                typename T::iterator vp(themap.find(id));
                if(vp == themap.end())
                    vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
                vp->second.id = vp->first;
		

                xml_read(c, themap[id], reader); // refers to a static function in osm_xml_read.cpp
             }
        }
    }

}

template <class closure, typename T>
inline void read_map(closure &c, T &themap, xmlpp::TextReader &reader, const str &item_name, const str &container_name)
{
    while(!is_closing_element(reader, container_name))
    {
        read_skip_comment(reader);

        if(reader.get_node_type() == xmlpp::TextReader::Element)
        {
           if(reader.get_name() != item_name) {
              // std::cerr << boost::str(boost::format("Found stray %s in %s container search (expected %s)") % reader.get_name() % container_name % item_name) << std::endl;
	      std::cerr<<"Error in read_map in xml_util.hpp"<<std::endl;
              continue;
           }

           const str id(reader.get_attribute("id"));
           assert(id != str());
           typename T::iterator vp(themap.find(id));
           if(vp == themap.end())
              vp = themap.insert(vp, std::make_pair(id, typename T::value_type::second_type()));
           vp->second.id = vp->first;

           themap[id].xml_read(c, reader); // in hwm_xml_read.cpp
        }
    }
}

template <class closure, typename T>
inline void sumo_read_map(
                          closure &c,
                          T &themap,
                          xmlpp::TextReader &reader,
                          const str &item_name,
                          const str &container_name) {
   do
   {
      read_skip_comment(reader);

      if(reader.get_node_type() == xmlpp::TextReader::Element)
      {
         // Checks to see if this node is the type being looked for.
         if(reader.get_name() != item_name) {
            // std::cerr << boost::str(boost::format("Found stray %s in %s container search (expected %s)") % reader.get_name() % container_name % item_name)  << std::endl;
	   std::cerr<<"Error in sumo_read_map in xml_util.hpp"<<std::endl;
         }
         else {

            // Gets the elements id.
            const str id(reader.get_attribute("id"));

            // Retrieves the element from the map or creates it if needed.
            typename T::iterator vp(themap.find(id));
            if(vp == themap.end()) {
               auto new_pair =
               std::make_pair(id, typename T::value_type::second_type());
               vp = themap.insert(vp, new_pair);
            }

            // Sets the id of the new element.
            vp->second.id = vp->first;

            // In sumo_xml_read.cpp
            // Reads the attributes and buildes the element in the map.
            // Tests to see if the attribute was fully read. If not, removes it
            // from the map.
            if (!xml_read(c, themap[id], reader)) {
               themap.erase(vp);
            }
         }
      }
   }
   while(!is_closing_element(reader, container_name));
}

template <class network, typename T>
inline void sumo_read_vector(
                          network &c,
                          T &a_vector,
                          xmlpp::TextReader &reader,
                          const str &item_name,
                          const str &container_name) {
   do
   {
      read_skip_comment(reader);

      if(reader.get_node_type() == xmlpp::TextReader::Element)
      {
         // Checks to see if this node is the type being looked for.
         if(reader.get_name() != item_name) {
            // std::cerr << boost::str(boost::format("Found stray %s in %s container search (expected %s)") % reader.get_name() % container_name % item_name)  << std::endl;
	   std::cerr<<"Error in sumo_read_vector in xml_util.hpp"<<std::endl;
         }
         else {
            // Increase the vector size.
            a_vector.resize(a_vector.size() + 1);

            // In sumo_xml_read.cpp
            // Reads the attributes and buildes the element in the map.
            // Tests to see if the attribute was fully read. If not, throws an
            // exception.
            if (!xml_read(c, a_vector[a_vector.size() - 1], reader)) {
               a_vector.pop_back();
            }
         }
      }
   }
   while(!is_closing_element(reader, container_name));
}

#endif
