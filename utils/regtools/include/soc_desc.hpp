/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2014 by Amaury Pouly
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/  
#ifndef __SOC_DESC__
#define __SOC_DESC__

#include <stdint.h>
#include <vector>
#include <list>
#include <string>
#include <map>

namespace soc_desc
{

const size_t MAJOR_VERSION = 2;
const size_t MINOR_VERSION = 0;
const size_t REVISION_VERSION = 0;

/** Typedef for SoC types: word, address and flags */
typedef uint32_t soc_addr_t;
typedef uint32_t soc_word_t;
typedef int soc_id_t;

/** Error class */
class error_t
{
public:
    enum level_t
    {
        INFO,
        WARNING,
        FATAL
    };
    error_t(level_t lvl, const std::string& loc, const std::string& msg)
        :m_level(lvl), m_loc(loc), m_msg(msg) {}
    level_t level() const { return m_level; }
    std::string location() const { return m_loc; }
    std::string message() const { return m_msg; }
protected:
    level_t m_level;
    std::string m_loc, m_msg;
};

/** Error context to log errors */
class error_context_t
{
public:
    void add(const error_t& err) { m_list.push_back(err); }
    size_t count() const { return m_list.size(); }
    error_t get(size_t i) const { return m_list[i]; }
protected:
    std::vector< error_t > m_list;
};

/**
 * Bare representation of the format
 */

/** Enumerated value (aka named value), represents a special value for a field */
struct enum_t
{
    soc_id_t id; /** ID (must be unique among field enums) */
    std::string name; /** Name (must be unique among field enums) */
    std::string desc; /** Optional description of the meaning of this value */
    soc_word_t value; /** Value of the field */
};

/** Register field information */
struct field_t
{
    soc_id_t id; /** ID (must be unique among register fields) */
    std::string name; /** Name (must be unique among register fields) */
    std::string desc; /** Optional description of the field */
    size_t pos; /** Position of the least significant bit */
    size_t width; /** Width of the field in bits  */
    std::vector< enum_t > enum_; /** List of special values */

    /** Returns the bit mask of the field within the register */
    soc_word_t bitmask() const
    {
        // WARNING beware of the case where width is 32
        if(width == 32)
            return 0xffffffff;
        else
            return ((1 << width) - 1) << pos;
    }

    /** Extract field value from register value */
    soc_word_t extract(soc_word_t reg_val) const
    {
        return (reg_val & bitmask()) >> pos;
    }

    /** Replace the field value in a register value */
    soc_word_t replace(soc_word_t reg_val, soc_word_t field_val) const
    {
        return (reg_val & ~bitmask()) | ((field_val << pos) & bitmask());
    }

    /** Return field value index, or -1 if none */
    int find_value(soc_word_t v) const
    {
        for(size_t i = 0; i < enum_.size(); i++)
            if(enum_[i].value == v)
                return i;
        return -1;
    }
};

/** Register information */
struct register_t
{
    size_t width; /** Size in bits */
    std::vector< field_t > field; /** List of fields */
};

/** Node address range information */
struct range_t
{
    enum type_t
    {
        STRIDE, /** Addresses are given by a base address and a stride */
        FORMULA /** Addresses are given by a formula */
    };

    type_t type; /** Range type */
    size_t first; /** First index in the range */
    size_t count; /** Number of indexes in the range */
    soc_word_t base; /** Base address (for STRIDE) */
    soc_word_t stride; /** Stride value (for STRIDE) */
    std::string formula; /** Formula (for FORMULA) */
    std::string variable; /** Formula variable name (for FORMULA) */
};

/** Node instance information */
struct instance_t
{
    enum type_t
    {
        SINGLE, /** There is a single instance at a specified address */
        RANGE /** There are multiple addresses forming a range */
    };

    soc_id_t id; /** ID (must be unique among node instances) */
    std::string name; /** Name (must be unique among node instances) */
    std::string title; /** Optional instance human name */
    std::string desc; /** Optional description of the instance */
    type_t type; /** Instance type */
    soc_word_t addr; /** Address (for SINGLE) */
    range_t range; /** Range (for RANGE) */
};

/** Node information */
struct node_t
{
    soc_id_t id; /** ID (must be unique among nodes) */
    std::string name; /** Name (must be unique for the among nodes) */
    std::string title; /** Optional node human name */
    std::string desc; /** Optional description of the node */
    std::vector< register_t> register_; /** Optional register */
    std::vector< instance_t> instance; /** List of instances */
    std::vector< node_t > node; /** List of sub-nodes */
};

/** System-on-chip information */
struct soc_t
{
    std::string name; /** Codename of the SoC */
    std::string title; /** Human name of the SoC */
    std::string desc; /** Optional description of the SoC */
    std::string isa; /** Instruction Set Assembly */
    std::string version; /** Description version */
    std::vector< std::string > author; /** List of authors of the description */
    std::vector< node_t > node; /** List of nodes */
};

/** Parse a SoC description from a XML file, put it into <soc>. */
bool parse_xml(const std::string& filename, soc_t& soc, error_context_t& error_ctx);
/** Write a SoC description to a XML file, overwriting it. A file can contain
 * multiple Soc descriptions */
bool produce_xml(const std::string& filename, const soc_t& soc, error_context_t& error_ctx);
/** Formula parser: try to parse and evaluate a formula with some variables */
bool evaluate_formula(const std::string& formula,
    const std::map< std::string, soc_word_t>& var, soc_word_t& result,
    const std::string& loc, error_context_t& error_ctx);

/**
 * Convenience API to manipulate the format
 *
 * The idea is that *_ref_t objects are stable pointers: they stay valid even
 * when the underlying soc changes. In particular:
 * - modifying any structure data (except id fields) preserves all references
 * - removing a structure invalidates all references pointing to this structure
 *   and its children
 * - adding any structure preserves all references
 * These references can be used to get pointers to the actual data
 * of the representation when it needs to be read or write.
 */

class soc_ref_t;
class node_ref_t;
class register_ref_t;
class field_ref_t;
class node_inst_t;

/** SoC reference */
class soc_ref_t
{
    soc_t *m_soc; /* pointer to the soc */
public:
     /** Builds an invalid reference */
    soc_ref_t();
     /** Builds a reference to a soc */
    soc_ref_t(soc_t *soc);
    /** Checks whether this reference is valid */
    bool valid() const;
    /** Returns a pointer to the soc */
    soc_t *get() const;
    /** Returns a reference to the root node */
    node_ref_t root() const;
    /** Returns a reference to the root node instance */
    node_inst_t root_inst() const;
    /** Compare this reference to another */
    bool operator==(const soc_ref_t& r) const;
    inline bool operator!=(const soc_ref_t& r) const { return !operator==(r); }
};

/** SoC node reference
 * NOTE: the root soc node is presented as a node with empty path */
class node_ref_t
{
    friend class soc_ref_t;
    friend class node_inst_t;
    soc_ref_t m_soc; /* reference to the soc */
    std::vector< soc_id_t > m_path; /* path from the root */

    node_ref_t(soc_ref_t soc);
    node_ref_t(soc_ref_t soc, const std::vector< soc_id_t >& path);
public:
    /** Builds an invalid reference */
    node_ref_t();
    /** Check whether this reference is valid */
    bool valid() const;
    /** Check whether this reference is the root node */
    bool is_root() const;
    /** Returns a pointer to the node, or 0 if invalid or root */
    node_t *get() const;
    /** Returns a reference to the soc */
    soc_ref_t soc() const;
    /** Returns a reference to the parent node */
    node_ref_t parent() const;
    /** Returns a reference to the register (which may be on a parent node) */
    register_ref_t reg() const;
    /** Returns a list of references to the sub-nodes */
    std::vector< node_ref_t > children() const;
    /** Returns a reference to a specific child */
    node_ref_t child(const std::string& name) const;
    /** Returns the path of the node, as the list of node names from the root */
    std::vector< std::string > path() const;
    /** Returns the name of the node */
    std::string name() const;
    /** Compare this reference to another */
    bool operator==(const node_ref_t& r) const;
    inline bool operator!=(const node_ref_t& r) const { return !operator==(r); }
};

/** SoC register reference */
class register_ref_t
{
    friend class node_ref_t;
    node_ref_t m_node; /* reference to the node owning the register */

    register_ref_t(node_ref_t node);
public:
    /** Builds an invalid reference */
    register_ref_t();
    /** Check whether this reference is valid/exists */
    bool valid() const;
    /** Returns a pointer to the register, or 0 */
    register_t *get() const;
    /** Returns a reference to the node containing the register */
    node_ref_t node() const;
    /** Returns a list of references to the fields of the register */
    std::vector< field_ref_t > fields() const;
    /** Returns a reference to a particular field */
    field_ref_t field(const std::string& name) const;
    /** Compare this reference to another */
    bool operator==(const register_ref_t& r) const;
    inline bool operator!=(const register_ref_t& r) const { return !operator==(r); }
};

/** SoC register field reference */
class field_ref_t
{
    friend class register_ref_t;
    register_ref_t m_reg; /* reference to the register */
    soc_id_t m_id; /* field name */

    field_ref_t(register_ref_t reg, soc_id_t id);
public:
    /** Builds an invalid reference */
    field_ref_t();
    /** Check whether this reference is valid/exists */
    bool valid() const;
    /** Returns a pointer to the field, or 0 */
    field_t *get() const;
    /** Returns a reference to the register containing the field */
    register_ref_t reg() const;
    /** Compare this reference to another */
    bool operator==(const field_ref_t& r) const;
    inline bool operator!=(const field_ref_t& r) const { return !operator==(r); }
};

/** SoC node instance
 * NOTE: the root soc node is presented as a node with a single instance at 0 */
class node_inst_t
{
    friend class node_ref_t;
    friend class soc_ref_t;
    node_ref_t m_node; /* reference to the node */
    std::vector< soc_id_t > m_id_path; /* list of instance IDs */
    std::vector< size_t > m_index_path; /* list of instance indexes */

    node_inst_t(soc_ref_t soc);
    node_inst_t(node_ref_t soc, const std::vector< soc_id_t >& path,
        const std::vector< size_t >& indexes);
public:
    /** Builds an invalid reference */
    node_inst_t();
    /** Check whether this instance is valid/exists */
    bool valid() const;
    /** Returns a reference to the soc */
    soc_ref_t soc() const;
    /** Returns a reference to the node */
    node_ref_t node() const;
    /** Check whether this reference is the root node instance */
    bool is_root() const;
    /** Returns a reference to the parent instance */
    node_inst_t parent() const;
     /** Returns a pointer to the instance of the node, or 0 */
    instance_t *get() const;
    /** Returns the address of this instance */
    soc_addr_t addr() const;
    /** Returns an instance to a child of this node's instance. If the subnode
     * instance is a range, the returned reference is invalid */
    node_inst_t child(const std::string& name) const;
    /** Returns an instance to a child of this node's instance with a range index.
     * If the subnode is not not a range or if the index is out of bounds,
     * the returned reference is invalid */
    node_inst_t child(const std::string& name, size_t index) const;
    /** Returns a list of all instances of subnodes of this node's instance */
    std::vector< node_inst_t > children() const;
    /** Returns the name of the instance */
    std::string name() const;
    /** Checks whether this instance is indexed */
    bool is_indexed() const;
    /** Returns the index of the instance */
    size_t index() const;
    /** Compare this reference to another */
    bool operator==(const node_inst_t& r) const;
    inline bool operator!=(const node_inst_t& r) const { return !operator==(r); }
};

} // soc_desc

#endif /* __SOC_DESC__ */
