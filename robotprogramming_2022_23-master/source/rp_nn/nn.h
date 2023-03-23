#pragma once
#include <cmath>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>
#include <map>
#include <memory>
#include "activation_function.h"
#include "neuron.h"
#include <iostream>
using namespace std;

class NeuralNet {
  friend class NeuralNetOptimizer;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // helper function
  // initializes the network by creating layers
  // all intermediate neurons are of IntermediateNeuronType_
  // the outputs are of type OutputNeuronType_
  
  template <typename IntermediateNeuronType_, typename OutputNeuronType_>
  void init(std::vector<int>& neurons_per_layer);
  
  inline NeuronPtrVector& inputLayer() {return neurons.front();}
  inline NeuronPtrVector& outputLayer() {return neurons.back();}
  
  // one forward step, updates outputs
  void forward();

  int numWeights();
  int numNeurons();

  
  void addNeuron(int level, int level_pos, NeuronBase* n);

  // throws if no link
  void checkLink(int parent_level, int parent_pos,
                 int child_level, int child_pos);

  // returns the link at pos. throws if not there
  NeuronBase::Link& link(int parent_level, int parent_pos,
                         int child_level, int child_pos);
  
  // adds a link between two neurons
  void addLink(int parent_level, int parent_pos,
               int child_level, int child_pos, float weight=0.5);

  void setInputs(const std::vector<float>& inputs);

  NeuronPtrVectorVector neurons;
  void write(std::ostream& is);
  void read(std::istream& is);
  
protected:
  struct PairCompare {
    inline bool operator()(const std::pair<const NeuronBase*, const NeuronBase*>& a,
                           const std::pair<const NeuronBase*, const NeuronBase*>& b){
      return a.first<b.first || (a.first==b.first && a.second <b.second);
    }
  };
  using LinkMapType=std::map<std::pair<const NeuronBase*, const NeuronBase*>, int, PairCompare>;

  // assigns the vector index to the weights
  void enumerate();

  NeuronBase::Link* linkPtr(int parent_level, int parent_pos,
                            int child_level, int child_pos);

// map parent->child -> level in child
  LinkMapType _link_map;

  int _num_weights=-1;
  int _num_neurons=-1;

  
};

#include "nn_impl.h"

