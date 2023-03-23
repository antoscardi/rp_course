#include "factor_graph.h"
#include <srrg_boss/deserializer.h>
#include <srrg_boss/serializer.h>
// included here since I instantiate all methods
namespace srrg2_solver {

  using namespace std;
  using namespace srrg2_core;

  FactorGraphInterface::~FactorGraphInterface() {
  }

  void FactorGraphInterface::clear() {
    variables().clear();
    factors().clear();
    _var_to_factor_map.clear();
  }

  FactorGraphInterface::VariableFactorMap& FactorGraphInterface::_v2f(VariableBase::Id id, bool insert) {
    auto it=_var_to_factor_map.find(id);
    if (it!=_var_to_factor_map.end()) {
      return it->second;
    } else {
      if (! insert)
        std::runtime_error("FactorGraphInrerface::_v2f, no var in");
    }
    auto ret=_var_to_factor_map.insert(std::make_pair(id, VariableFactorMap()));
    return ret.first->second;
  }

  void FactorGraphInterface::_write(srrg2_core::Serializer& ser, std::set<FactorBase*>& selected_factors) {
    cerr << endl;
    std::set<VariableBase*> selected_variables;
    for (auto f:  selected_factors) {
      if (! factor(f->graphId())) {
        throw std::runtime_error("no factor in serialization");
      }
      int num_var = f->numVariables();
      for (int v_idx = 0; v_idx < num_var; ++v_idx) {
        VariableBase* v=f->variable(v_idx);
        if (v)
        selected_variables.insert(v);
      }
    }
    int object_count=0;
    for (VariableBase* v : selected_variables) {
      ++object_count;
      if (! (object_count%1000) )
        cerr << "\robjects written" << object_count;
      ser.writeObject(*v);
    }
    for (FactorBase* f : selected_factors) {
      ++object_count;
      if (! (object_count%1000) )
        cerr << "\robjects written" << object_count;
      ser.writeObject(*f);
    }
  }

  void FactorGraphInterface::write(const std::string& filename) {
    Serializer ser;
    ser.setFilePath(filename);
    cerr << "writing view on file [ " << filename << "]" << endl;
    std::set<FactorBase*> selected_factors;
    for (auto f_it: factors()) {
      selected_factors.insert(f_it.second);
    }
    _write(ser, selected_factors);
    cerr<< "done" << endl;
  }

  void FactorGraphInterface::printVariables() {
    for (auto it = variables().begin(); it != variables().end(); ++it) {
      const VariableBase* const v = it.value();
      cerr << "id: " << v->graphId() << " " << v->status() << endl;
    }
  }

  VariableBase* FactorGraphInterface::variable(VariableBase::Id id) {
    auto it = variables().find(id);
    if (it == variables().end()) {
      return 0;
    }
    VariableBase* v = it.value();
    return v;
  }

  FactorBase* FactorGraphInterface::factor(FactorBase::Id id) {
    auto it = factors().find(id);
    if (it == factors().end()) {
      return 0;
    }
    FactorBase* f = it.value();
    return f;
  }

  int FactorGraphInterface::bindFactor(FactorBase* f) {
    int num_vars = 0;
    num_vars     = f->bind(variables());
    if (num_vars>0) {
      f->setEnabled(false);
      return num_vars;
    }
    for (int var_idx = 0; var_idx < f->numVariables(); ++var_idx) {
      VariableBase* v = f->variable(var_idx);
      if (v) {
        _v2f(v->graphId()).insert(VariableFactorPair(v->graphId(), f));
      }
    }
    if (num_vars)
      throw std::runtime_error(
                               "error in bind, you are trying to add a factor"
                               "that connects variables you did not put in the container");
    // we now populate the graph from scratch
    return num_vars;
  }

  void FactorGraphInterface::unbindFactor(FactorBase* f) {
    //cerr << __PRETTY_FUNCTION__ << "| " << this << endl;
    for (int var_idx = 0; var_idx < f->numVariables(); ++var_idx) {
      VariableBase* v = f->variable(var_idx);
      if (v) {
        VariableFactorPair key(v->graphId(), f);
        _v2f(v->graphId()).erase(key);
      }
      f->setVariable(var_idx, 0);
    }
  }

  int FactorGraphInterface::bindFactors() {
    //    std::cerr << "BindFactors " << this << std::endl;
    _var_to_factor_map.clear();
    int unbinded_vars = 0;
    int suppressed_factors = 0;
    for (auto it = factors().begin(); it != factors().end(); ++it) {
      FactorBase* f = it.value();
      int ub=bindFactor(f);
      if (ub) {
        f->setEnabled(false);
      }
      unbinded_vars += ub;
      ++suppressed_factors;
    }
    if (unbinded_vars > 0) {
      cerr << "there are up to " << unbinded_vars << " dangling variables\n" << endl;
      cerr << "disabling " << suppressed_factors << " factors\n" << endl;
      cerr << "optimize at your own risk\n" << endl;
    }
    // we now populate the graph from scratch
    return unbinded_vars;
  }

  FactorGraphInterface::VariableFactorMap& FactorGraphInterface::factors(const VariableBase* v){
    return _v2f(v->graphId());
  }

  bool FactorGraphInterface::removeFactor(FactorBase* f) {
    //cerr << __PRETTY_FUNCTION__ << "| " << this << " " << f << endl;
    auto it = factors().find(f->graphId());
    if (it == factors().end()) {
      assert("factor not in graph");
      return false;
    }
    assert (f==it.value() && "factor ptr mismatch");
    unbindFactor(f);
    //cerr << "fac_remove, erasing factor" << f << endl;
    return true;
  }

  bool FactorGraphInterface::removeVariable(VariableBase* v) {
    //cerr << __PRETTY_FUNCTION__ << "| " << this << " " << v << endl;
    auto it = variables().find(v->graphId());
    if (it == variables().end()) {
      return false;
    }
    assert (v==it.value() && "variables ptr mismatch");
    //cerr << "Var id: "<< v->graphId()<< endl;
    _var_to_factor_map.erase(v->graphId());
    return true;
  }

}
