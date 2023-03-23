#include "factor_graph_initializer.h"
#include "srrg_solver/variables_and_factors/types_2d/all_types.h"
#include "srrg_solver/variables_and_factors/types_3d/all_types.h"
#include "srrg_solver/variables_and_factors/types_projective/all_types.h"
#include <Eigen/Eigenvalues>

namespace srrg2_solver {

  void FactorGraphInitializer::setGraph(FactorGraphInterface& graph_) {
    _graph = &graph_;
    _entries.clear();
    _initialized_variables.clear();
    _queue = FactorGraphVisitEntryQueue();
    // populate the entry structure to hold the visit
    // populate the queue with all fixed variables
    for (auto it = _graph->variables().begin(); it != _graph->variables().end(); ++it) {
      VariableBase* variable    = it.value();
      if (variable->status() == VariableBase::NonActive)
        continue;
      VariableVisitEntry* entry = _entries.add(VariableVisitEntry(variable));
      if (variable->status() == VariableBase::Fixed) {
        entry->cost = 0;
        if (! _parameter_ids.count(variable->graphId()))
          _queue.push(*entry);
        _initialized_variables.insert(variable->graphId());
      } else {
        entry->cost = -1;
      }
    }
    if (verbose)
      std::cerr << "queue.size()" << _queue.size() << std::endl;
  }

  float FactorGraphInitializer::getCost(VariableBase* v) {
    VariableVisitEntry* ne = _entries.at(v->graphId());
    if (! ne)
      throw std::runtime_error("getCost on non visited variable");
    std::set<VariableBase*> neighbors;
    for (auto f_it: _graph->factors(ne->variable)) {
      FactorBase* f=f_it.second;
      if (! f->enabled())
        continue;
      if (param_level.value()!=-1 && f->level()!=param_level.value())
        continue;
      for (int i=0; i<f->numVariables(); ++i) {
        int v_id=f->variableId(i);
        if (_parameter_ids.count(v_id))
          continue;
        VariableBase* v_other=_graph->variable(v_id);
        neighbors.insert(v_other);
      }
    }
    float min_cost=std::numeric_limits<float>::max();
    for (auto v_other: neighbors) {
      VariableVisitEntry* e_other=_entries.at(v_other->graphId());
      if (! e_other)
        continue;
      if (e_other->cost<0)
        continue;
      min_cost=std::min(e_other->cost, min_cost);
    }
    return min_cost;
  }

  void FactorGraphInitializer::updateGraph() {
    // populate the queue with all variables that are fixed
    // and all the ones that were not in the graph before
    _queue = FactorGraphVisitEntryQueue();
    for (auto it = _graph->variables().begin(); it != _graph->variables().end(); ++it) {
      VariableBase* variable    = it.value();
      if (variable->status() == VariableBase::NonActive) 
        continue;
      VariableVisitEntry* entry = _entries.at(variable->graphId());
      if (! entry) {
        entry = _entries.add(VariableVisitEntry(variable));
        if (variable->status() == VariableBase::Fixed) {
          entry->cost = 0;
          _initialized_variables.insert(variable->graphId());
          if (! _parameter_ids.count(variable->graphId()))
              _queue.push(*entry);
        } else {
          entry->cost=-1;
        }
      } else {
        _queue.push(*entry);
      }
        
    }
    //std::cerr << "queue.size()" << _queue.size() << std::endl;
  }
  using namespace std;
  
  void FactorGraphInitializer::compute() {
    if (verbose) 
      cerr << "init [ ";
    while (!_queue.empty()) {
      VariableVisitEntry e = _queue.top();
      VariableVisitEntry* e_other=_entries.at(e.variable->graphId());
      if (verbose ) std::cerr << "pop: " << e.variable->graphId() << " " << e.cost << std::endl;
      _queue.pop();
      if (e_other->cost<e.cost)
        continue;
      if (param_max_cost.value()>-1 && e.cost>param_max_cost.value())
        return;
      VariableBase* v = e.variable;
      if (! _graph->variable(v->graphId()))
          throw std::runtime_error("FactorGraphInitializer, bookkeping error");
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        int v_pos          = -1;
        FactorBase* f = f_it.second;
        if (! f->enabled()) {
          continue;
        }
        for (int i = 0; i < f->numVariables(); ++i) {
          if (f->variable(i) == v) {
            e.var_pos = i;
            v_pos      = i;
            break;
          }
        }

        // scan all variables
        for (int nv_pos = 0; nv_pos < f->numVariables(); ++nv_pos) {
          if (nv_pos == v_pos)
            continue;
          VariableBase* nv       = f->variable(nv_pos);
          if (_parameter_ids.count(nv->graphId()))
            continue;

          VariableVisitEntry* ne = _entries.at(nv->graphId());
          //std::cerr << "\t nv:" << nv->graphId() << std::endl;
          //std::cerr << "ne: " << ne << std::endl;
          if (! ne)
            continue;
          
          //assert(ne && "bookkeeping error 2");
          // if variable already initialized
          if (ne->cost >= 0)
            continue;
          if (initVariable(nv)) {
            if (verbose) {
              cerr << "(" << v->graphId() << "-" << nv->graphId() << ") " ;
            }
            _initialized_variables.insert(nv->graphId());
            //ne->cost = e.cost + 1;
            ne->cost = getCost(nv) + 1;
            _queue.push(*ne);
            //std::cerr << "push: " << ne->variable->graphId() << " " << ne->cost << std::endl;
          }
        }
      }
    }
    if (verbose) 
      cerr << "]";
  }

  bool FactorGraphInitializer::initVariable(VariableBase* variable) {
    for (auto it = _rules.begin(); it != _rules.end(); ++it) {
      if ((*it)->init(variable))
        return true;
    }
    return false;
  }

  bool FactorGraphInitializer::isInit(VariableBase* v) {
    VariableVisitEntry* ne = _entries.at(v->graphId());
    if (! ne)
      return false;
    return ne->cost >= 0;
  }

  float FactorGraphInitializer::maxCost() {
    float max_cost=-1;
    for (auto v_id: _initialized_variables) {
      auto entry=_entries.at(v_id);
      if (! entry)
        continue;
      max_cost=std::max(entry->cost, max_cost);
    }
    return max_cost;
  }

  struct SE2toSE2InitializerRule : public FactorGraphInitializerRule_<VariableSE2Base> {
    SE2toSE2InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariableSE2Base>(initializer_) {
    }

    bool doInit(VariableSE2Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;

      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        
        SE2PosePoseGeodesicErrorFactor* this_factor =
          dynamic_cast<SE2PosePoseGeodesicErrorFactor*>(f);
        if (!this_factor)
          continue;
        bool direct             = true;
        VariableSE2Base* root_v = this_factor->variables().at<0>();
        if (root_v == v) {
          root_v = this_factor->variables().at<1>();
          direct = false;
        }
        if (!_initializer->isInit(root_v))
          continue;

        if (direct) {
          v->setEstimate(root_v->estimate() * this_factor->measurement());
        } else {
          v->setEstimate(root_v->estimate() * this_factor->measurement().inverse());
        }
        //std::cerr << "se2 pose_pose init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE2toPoint2InitializerRule : public FactorGraphInitializerRule_<VariablePoint2> {
    SE2toPoint2InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint2>(initializer_) {
    }

    bool doInit(VariablePoint2* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        SE2PosePointErrorFactor* this_factor =
          dynamic_cast<SE2PosePointErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariableSE2Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        v->setEstimate(root_v->estimate() * this_factor->measurement());
        //std::cerr << "se2 pose_point init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE2toPoint2BearingInitializerRule : public FactorGraphInitializerRule_<VariablePoint2> {
    SE2toPoint2BearingInitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint2>(initializer_) {
    }

    bool doInit(VariablePoint2* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      std::list<SE2PosePointBearingErrorFactor*> active_factors;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        SE2PosePointBearingErrorFactor* this_factor =
          dynamic_cast<SE2PosePointBearingErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariableSE2Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        active_factors.push_back(this_factor);
      }
      if (active_factors.size() < 2)
        return false;
      // do a triangulation with the factors
      Matrix2f H = Matrix2f::Zero();
      Vector2f b = Vector2f::Zero();
      Vector2f x = v->estimate();
      for (auto it = active_factors.begin(); it != active_factors.end(); ++it) {
        SE2PosePointBearingErrorFactor* f = *it;
        VariableSE2Base* pose                    = f->variables().at<0>();
        float theta                              = f->measurement()(0);
        Vector2f n        = pose->estimate().linear() * Vector2f(cos(theta), sin(theta));
        n                 = Vector2f(n.y(), -n.x());
        const Vector2f& p = pose->estimate().translation();
        float e           = n.dot(x - p);
        H += n * n.transpose();
        b += n * e;
      }
      x -= (H).ldlt().solve(b);
      v->setEstimate(x);
      // todo: check for consistency of solution, otherwise disable variable
      //std::cerr << "se2 pose point bearinf :" << x.transpose() << std::endl;
      return true;
    }
  };

  struct SE3toSE3InitializerRule : public FactorGraphInitializerRule_<VariableSE3Base> {
    SE3toSE3InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariableSE3Base>(initializer_) {
    }

    bool doInit(VariableSE3Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        SE3PosePoseGeodesicErrorFactor* this_factor =
          dynamic_cast<SE3PosePoseGeodesicErrorFactor*>(f);
        if (!this_factor)
          continue;
        bool direct             = true;
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (root_v == v) {
          root_v = this_factor->variables().at<1>();
          direct = false;
        }
        if (!_initializer->isInit(root_v))
          continue;

        if (direct) {
          v->setEstimate(root_v->estimate() * this_factor->measurement());
        } else {
          v->setEstimate(root_v->estimate() * this_factor->measurement().inverse());
        }
        //std::cerr << "se3 pose_pose init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct Sim3toSim3InitializerRule : public FactorGraphInitializerRule_<VariableSim3Base> {
    Sim3toSim3InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariableSim3Base>(initializer_) {
    }

    bool doInit(VariableSim3Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        Sim3PosePoseErrorFactorAD* this_factor =
          dynamic_cast<Sim3PosePoseErrorFactorAD*>(f);
        if (!this_factor)
          continue;
        bool direct             = true;
        VariableSim3Base* root_v = this_factor->variables().at<0>();
        if (root_v == v) {
          root_v = this_factor->variables().at<1>();
          direct = false;
        }
        if (!_initializer->isInit(root_v))
          continue;

        if (direct) {
          v->setEstimate(root_v->estimate() * this_factor->measurement());
        } else {
          v->setEstimate(root_v->estimate() * this_factor->measurement().inverse());
        }
        //std::cerr << "se3 pose_pose init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE3toPoint3InitializerRule : public FactorGraphInitializerRule_<VariablePoint3> {
    SE3toPoint3InitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint3>(initializer_) {
    }

    bool doInit(VariablePoint3* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
        for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;

        if (level!=-1 && f->level()!=level)
          continue;
        SE3PosePointErrorFactor* this_factor =
          dynamic_cast<SE3PosePointErrorFactor*>(f);
        if (!this_factor)
          continue;
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        v->setEstimate(root_v->estimate() * this_factor->measurement());
        //std::cerr << "se3 pose_point init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };

  struct SE3toPoint3OffsetInitializerRule : public FactorGraphInitializerRule_<VariablePoint3> {
    SE3toPoint3OffsetInitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint3>(initializer_) {
    }

    bool doInit(VariablePoint3* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        SE3PosePointOffsetErrorFactor* this_factor =
          dynamic_cast<SE3PosePointOffsetErrorFactor*>(f);
        if (!this_factor)
          continue;
        VariableSE3Base* offset_v = this_factor->variables().at<2>();
        if (!_initializer->isInit(offset_v))
          continue;
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;

        v->setEstimate(root_v->estimate() * offset_v->estimate() * this_factor->measurement());
        //std::cerr << "se3 pose_point_offset init" << root_v->graphId() << " " << v->graphId() << std::endl;
        return true;
      }
      return false;
    }
  };


  struct SE3toPoint3OmniBAInitializerRule : public FactorGraphInitializerRule_<VariablePoint3> {
    SE3toPoint3OmniBAInitializerRule(FactorGraphInitializer* initializer_) :
      FactorGraphInitializerRule_<VariablePoint3>(initializer_) {
    }

    bool doInit(VariablePoint3* v) override {
      std::list<SE3PosePointOmniBAErrorFactor*> active_factors;
      FactorGraphInterface* _graph = _initializer->_graph;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        SE3PosePointOmniBAErrorFactor* this_factor =
          dynamic_cast<SE3PosePointOmniBAErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariableSE3Base* root_v = this_factor->variables().at<0>();
        if (!_initializer->isInit(root_v))
          continue;
        active_factors.push_back(this_factor);
      }
      if (active_factors.size() < 3)
        return false;
      // do a triangulation with the factors
      Matrix3f H = Matrix3f::Zero();
      Vector3f b = Vector3f::Zero();
      Vector3f x = v->estimate();
      for (auto it = active_factors.begin(); it != active_factors.end(); ++it) {
        SE3PosePointOmniBAErrorFactor* f = *it;
        VariableSE3Base* pose                    = f->variables().at<0>();
        VariableSE3Base* offset                  = f->variables().at<2>();
        Vector3f d        = pose->estimate().linear()*offset->estimate().linear()*f->measurement();
        Matrix3f J=geometry3d::skew(d);
        const Vector3f& p = pose->estimate()*offset->estimate().translation();
        Vector3f e        = J*(x - p);
        H += J.transpose() * J;
        b += J.transpose() * e;
      }
      x -= (H).ldlt().solve(b);
      v->setEstimate(x);
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f>  es;
      es.compute(H);
      float lambda_min=es.eigenvalues()(0);
      float lambda_max=es.eigenvalues()(2);
      if (lambda_min/lambda_max<1e-1)
        return false;
      // todo: check for consistency of solution, otherwise disable variable
      //std::cerr << "se3 pose point omni ba:" << x.transpose() << std::endl;
      return true;
    }
  };

    struct Point3toSE3InitializerRule : public FactorGraphInitializerRule_<VariableSE3Base> {

      Point3toSE3InitializerRule(FactorGraphInitializer* initializer_)
        : FactorGraphInitializerRule_<VariableSE3Base>(initializer_) {
      }

    bool doInit(VariableSE3Base* v) override {
      FactorGraphInterface* _graph = _initializer->_graph;
      std::list<SE3PosePointErrorFactor*> active_factors;
      for (auto f_it: _graph->FactorGraphInterface::factors(v)) {
        FactorBase* f = f_it.second;
        if (level!=-1 && f->level()!=level)
          continue;
        SE3PosePointErrorFactor* this_factor =
          dynamic_cast<SE3PosePointErrorFactor*>(f);
        if (!this_factor) {
          continue;
        }
        VariablePoint3* root_v = this_factor->variables().at<1>();
        if (!_initializer->isInit(root_v))
          continue;
        active_factors.push_back(this_factor);
      }
      if (active_factors.size() < 4)
        return false;
      const int iterations=10;
      //tiny ICP;
      Matrix6f H;
      Vector6f b;
      Vector6f dx;
      Isometry3f X=v->estimate().inverse();
      float last_chi =0;
      //cerr << "init : [";
      for (int i=0; i<iterations; ++i) {
        last_chi=0;
        H.setZero();
        b.setZero();
        dx.setZero();
        Matrix3_6f J;
        J.setZero();
        J.block<3,3>(0,0).setIdentity();
        // do a triangulation with the factors
        for (auto it = active_factors.begin(); it != active_factors.end(); ++it) {
          auto f = *it;
          auto p = f->variables().at<1>()->estimate();
          auto z = f->measurement();
          auto z_hat=X*p;
          J.block<3,3>(0,3)=-geometry3d::skew(z_hat);
          auto e = z_hat-z;
          H += J.transpose() * f->informationMatrix() * J;
          b += J.transpose() * f->informationMatrix() * e;
          last_chi+=e.transpose()*f->informationMatrix()*e;
        }
        //cerr << last_chi/active_factors.size() << " ";
        dx -= (H).ldlt().solve(b);
        X=geometry3d::ta2t(dx)*X;
      }
      //cerr << "] " << endl;
      
      Eigen::SelfAdjointEigenSolver<Matrix6f>  es;
      es.compute(H);
      float lambda_min=es.eigenvalues()(0);
      //float lambda_max=es.eigenvalues()(5);
      if (lambda_min<1e-1) { //gg: sucks big times. more times. can't stop sucking
        return false;
      }
      v->setEstimate(X.inverse());
      // todo: check for consistency of solution, otherwise disable variable
      //std::cerr << "se3 pose point omni ba:" << x.transpose() << std::endl;
      //cerr << "init!";
      return true;
    }
  };

  FactorGraphInitializer::FactorGraphInitializer() {
    _rules.push_back(FactorGraphInitializerRulePtr(new SE2toSE2InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toSE3InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new Sim3toSim3InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE2toPoint2InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toPoint3InitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toPoint3OffsetInitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE2toPoint2BearingInitializerRule(this)));
    _rules.push_back(FactorGraphInitializerRulePtr(new SE3toPoint3OmniBAInitializerRule(this))); 
    _rules.push_back(FactorGraphInitializerRulePtr(new Point3toSE3InitializerRule(this)));
 }

} // namespace srrg2_solver
