#include "nn_gd.h"

void NNGradientDescentOptimizer::computeGradients() {
    if (_weight_gradients.rows()!=_network.numWeights())
      _weight_gradients.resize(_network.numWeights());
    if (_forward_gradients.rows()!=_network.numNeurons())
      _forward_gradients.resize(_network.numNeurons());
    _weight_gradients.setZero();
    _forward_gradients.setZero();
    for (size_t num_layer=_network.neurons.size()-1; num_layer>0; --num_layer) {
      auto& layer=_network.neurons[num_layer];
      size_t k=0;
      for (auto& ptr : layer) {
        const int& neuron_index=ptr->index;
        float& forward_gradient = _forward_gradients(ptr->index);
        if (num_layer==_network.neurons.size()-1)
          forward_gradient = 2*(ptr->output-_desired_outputs[k]);
        else
          forward_gradient = 0;
        ++k;
      }
      for (auto& ptr : layer) {
        const float& forward_gradient = _forward_gradients(ptr->index);
        const float output_gradient=forward_gradient*ptr->d_output;
        for (auto& l: ptr->links) {
          const auto& parent=l.parent;
          auto& weight_gradient=_weight_gradients(l.index);
          weight_gradient += output_gradient * parent->output;
          _forward_gradients(parent->index) += output_gradient*l.weight;
        }
      }
    }
  }
  
  void NNGradientDescentOptimizer::applyGradients() {
    for (size_t num_layer=_network.neurons.size()-1; num_layer>0; --num_layer) {
      auto& layer=_network.neurons[num_layer];
      for (auto& ptr : layer) {
        for (auto& l: ptr->links) {
          l.weight += _weight_gradients(l.index)*learning_rate;
        }
      }
    }
  }
  
  float NNGradientDescentOptimizer::trainOnce(const NeuralNetOptimizer::TrainSet& train_set) {
    float chi_sum=0;
    cerr << "gradients: ";
    for (const auto& example: train_set) {
      _network.setInputs(example.first);
      setDesiredOutputs(example.second);
      _network.forward();
      float chi = eval();
      computeGradients();
      chi_sum += chi;
      cerr << ".";
    }
    cerr << endl << "apply gradients" << endl;
    applyGradients();
    return chi_sum;
  }
