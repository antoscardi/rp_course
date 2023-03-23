#pragma once
#include <vector>
#include <memory>
#include "activation_function.h"

// mother of all neurons
class NeuronBase {
public:
  // link: from parent to this
  struct Link {
    Link(NeuronBase *p, float w=0):
      parent(p),
      weight(w){
    };
    
    NeuronBase * parent;
    float weight;
    int index; // unique index in weight vector
  };

  inline float dot() const;
  
  virtual void forward(){}
  virtual const char* name() const {return "Input";}
  float output;
  float d_output=1;
  std::vector<Link> links;
  int level=0;
  int level_index; // index in level
  int index;       // unique index in net
};

//specialization for intermediate neuron, on an activation function
//takes an activation function as template parameter
template <typename ActivationFunctionType_=SigmoidActivationFunction>
class Neuron_: public NeuronBase {
public:
  using ActivationFunction = ActivationFunctionType_;
  void forward() override{
    ActivationFunction::compute(output, d_output, dot());
  }
  const char* name() const override {return ActivationFunction::NAME;}
};

using NeuronPtr=std::unique_ptr<NeuronBase>;
using NeuronPtrVector=std::vector<NeuronPtr>;
using NeuronPtrVectorVector=std::vector<NeuronPtrVector>;

#include "neuron_impl.h"
