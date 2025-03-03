#pragma once

#include "config.h"

#include <string>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <vector>
#include <functional>
#include <bitset>


// #include "simulation.hpp"
typedef uint64_t simulation_time_t;

namespace Tracing::VCD{

  class Module;
  class Variable;
  class VCDFile;/*  : public std::ofstream; */

  class Item{
  public:
    enum class Type{
      Module,
      Variable,
    };
    Item(const std::string &name, Type type)
    : m_name{name}
    , m_type{type}
    {}

    Module &as_module(); /* { return static_cast<Module&>(*this); } */
    Variable &as_variable(); /* { return static_cast<Variable&>(*this); } */

    void set_id(const std::string &id) { m_id = id; id_updated(); }
    const std::string &get_id() const { return m_id; }
    const std::string &get_name() const { return m_name; }
    Type get_type() const { return m_type; }
    void set_file(VCDFile *file) { 
      if(m_file || !m_file_internal)
        m_file = file;
      m_file_internal = file; 
      file_updated();
    }
    void enable() {
      m_file = m_file_internal;
    }
    virtual void disable() {
      m_file = nullptr;
    }
    virtual void dump() = 0;
    Item &operator[](const std::string &key);
  protected:
    virtual void id_updated(){};
    virtual void file_updated(){};
    VCDFile *file() { return m_file; }
  private:
    const std::string m_name;
    std::string m_id;
    const Type m_type;
    VCDFile *m_file;
    VCDFile *m_file_internal;
  };    

  class Module : public Item{
  public:
    Module(const std::string &name)
    : Item{name, Type::Module}
    {}
    void add_item(Item &item) { 
      m_items.push_back(item); 
      item.set_id(get_id() + generate_id(m_items.size()-1));
      item.set_file(file());
    }
    const std::vector<std::reference_wrapper<Item>> &items() const { return m_items; }
    void dump() override {
      for(auto &i : m_items){
        i.get().dump();
      }
    }
    Item &get(const std::string &key);
    void disable() override {
      Item::disable();
      for(auto &i : m_items){
        i.get().disable();
      }
    }
  protected:
    void id_updated() override {
      int cnt = 0;
      for(auto &i : m_items){
        i.get().set_id(get_id() + generate_id(cnt++));
      }
    }
    void file_updated() override {
      for(auto &i : m_items){
        i.get().set_file(file());
      }
    }
  private:
    static std::string generate_id(int n) {
      char start =  '!';
      char end = '~';
      int num_chars = end - start;
      std::string id{};
      while(n){
        id.push_back(start + n % num_chars);
        n /= num_chars;
      }
      return id + end;
    }
    std::vector<std::reference_wrapper<Item>> m_items;
  };
  
  class Variable : public Item{
  public:
    enum class Type{
      WIRE,
      REG,
      INTEGER,
      PARAMETER,
      REAL,
      EVENT,
      TIME,
      STRING
    };
    virtual std::ostream &print_state(std::ostream &os) const = 0;
    uint32_t width() const { return m_width; }
    void dump() override { updated(); }
    Type type() const { return m_type; }
    std::string type_str() const {
      switch(m_type){
        case Type::WIRE: return "wire";
        case Type::REG: return "reg";
        case Type::INTEGER: return "integer";
        case Type::PARAMETER: return "parameter";
        case Type::REAL: return "real";
        case Type::EVENT: return "event";
        case Type::TIME: return "time";
        case Type::STRING: return "string";
      }
      return "unknown";
    }
  protected:
    Variable(const std::string &name, Type type, uint32_t width)
    : Item{name, Item::Type::Variable}
    , m_type{type}
    , m_width{width}
    {}
    friend std::ostream &operator<<(std::ostream &os, const Variable &v) {
      return v.print_state(os);
    }
    #if config_VCD_TRACE_ENABLED
    void updated();
    #else
    void updated(){}
    #endif
  private:
    const Type m_type;
    uint32_t m_width;
  };

  template<class T>
  class Register final : public Variable{
  public:
    Register(const std::string &name, uint32_t width=sizeof(T)*8)
    : Variable{name, Type::REG, width}
    {}
    std::ostream &print_state(std::ostream &os) const override {
      os << "b" << std::bitset<sizeof(T)*8>(m_state) << " " << get_id() << std::endl;
      return os;
    }
    operator T() const { return m_state; }
    T operator=(T state) { 
      bool updated = m_state != state;
      m_state = state; 
      if(updated)
        this->updated();
      return state;
    }
    T operator=(const Register &state) {
      bool updated = m_state != state;
      m_state = state; 
      if(updated)
        this->updated();
      return state;
    }
    T operator |= (T state) { 
      bool updated = m_state & ~state;
      m_state |= state; 
      if(updated)
        this->updated();
      return state;
    }
    T operator &= (T state) { 
      bool updated = m_state & ~state;
      m_state &= state; 
      if(updated)
        this->updated();
      return state;
    }
    T operator--(){
      T ret = m_state--;
      this->updated();
      return ret;
    }
    T operator--(int){
      T ret = --m_state;
      this->updated();
      return ret;
    }
    T operator++(){
      T ret = m_state++;
      this->updated();
      return ret;
    }
    T operator++(int){
      T ret = ++m_state;
      this->updated();
      return ret;
    }
    void set_state(uint32_t state) { m_state = state; updated(); }
    T get_state() const { return m_state; }
  protected:
  private:
    T m_state;
    uint32_t m_width;
  };
  template<class T>
  class Time final : public Variable{
  public:
    Time(const std::string &name, uint32_t width)
    : Variable{name, Type::TIME, width}
    {}
    std::ostream &print_state(std::ostream &os) const override {
      os << "r" << m_state << " " << get_id() << std::endl;
      return os;
    }
    operator T() const { return m_state; }
    T operator=(T state) { 
      bool updated = m_state != state;
      m_state = state; 
      if(updated)
        this->updated();
      return state;
    }
    void set_state(uint32_t state) { m_state = state; updated(); }
    T get_state() const { return m_state; }
  protected:
  private:
    T m_state;
  };

  class VCDFile : public std::ofstream{
  public:
    VCDFile()
    {
      top().set_file(this);
    }
    ~VCDFile() {
      flush();
      close();
    }
    void check_time();
    VCDFile &operator<<(const Variable &v) {
      check_time();
      v.print_state(*this);
      return *this;
    }
    void write_module_header(const Module &m, size_t indent=0) {
      std::string indent_str{};
      for(size_t i=0; i<indent; i++)
        indent_str += "  ";
      *this << indent_str << "$scope module " << m.get_name() << " $end" << std::endl;
      for(auto &i : m.items()){
        auto &item = i.get();
        if(item.get_type() == Item::Type::Module)
          write_module_header(item.as_module(), indent+1);
        else{
          auto &v = item.as_variable();
          *this << indent_str << "  $var " << v.type_str() << " " << v.width() << " " << v.get_id() << " " << item.get_name() << " $end" << std::endl;
        }
      }
      *this << indent_str << "$upscope $end" << std::endl;
    }
    void write_header() {
      *this << "$version" << std::endl;
      *this << "  PicoSim VCD Generator" << (config_VCD_TRACE_ENABLED?"":" (Trace disabled)") << std::endl;
      *this << "$end" << std::endl;
      *this << "$timescale 100ps $end" << std::endl;
      write_module_header(m_top);
      *this << "$enddefinitions $end" << std::endl;
      *this << "$dumpvars" << std::endl;
      m_top.dump();
    }
    Module &top() { return m_top; }
  protected:
  private:
    simulation_time_t m_time;
    Module m_top{"TOP"};
  };

}