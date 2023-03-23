#include <Eigen/Cholesky>
#include "nn_gn.h"
#include <iostream>

using namespace std;

void NNGaussNewtonOptimizer::setupBuffers(){
  // compute the max number of weights in a layet
  size_t max_weights_in_layer=0;
  for (auto& layer : _network.neurons) {
    size_t weights_in_layer=0;
    for (auto& ptr: layer) {
      weights_in_layer+=ptr->links.size();
    }
    max_weights_in_layer=std::max(max_weights_in_layer, weights_in_layer);
  }
  _gX_buffer[0].resize(max_weights_in_layer*_network.outputLayer().size());
  _gX_buffer[1].resize(max_weights_in_layer*_network.outputLayer().size());
}
  
void NNGaussNewtonOptimizer::computeFullGradients() {
  _gX_buffer_ptr[0]=&_gX_buffer[0];
  _gX_buffer_ptr[1]=&_gX_buffer[1];
  _gX_rows=_network.outputLayer().size();
  _gX_cols=_network.outputLayer().size();
  Eigen::Map<Eigen::MatrixXf, Eigen::Aligned16> gX_init(_gX_buffer_ptr[0]->data(), _gX_rows, _gX_cols);
  gX_init.setZero();
  for (int i=0; i<gX_init.rows(); ++i)
    gX_init(i,i)=1;
  for (size_t num_layer=_network.neurons.size()-1; num_layer>0; --num_layer) {
    Eigen::Map<Eigen::MatrixXf, Eigen::Aligned16> gX(_gX_buffer_ptr[0]->data(), _gX_rows, _gX_cols);
    auto& previous_layer=_network.neurons[num_layer-1];
    auto& layer=_network.neurons[num_layer];
    _gX_rows=_network.outputLayer().size();
    _gX_cols=previous_layer.size();
    Eigen::Map<Eigen::MatrixXf, Eigen::Aligned16> gX_new(_gX_buffer_ptr[1]->data(), _gX_rows, _gX_cols);
    gX_new.setZero();
    for (auto& ptr : layer) {
      for (auto& link: ptr->links){
        //cerr << "current_weight_index: " << link.index << endl;
        _weight_gradients.col(link.index)=-gX.col(ptr->level_index)*ptr->d_output*link.parent->output;
        //cerr << " grad: " << _weight_gradients.col(current_weight_index).transpose() << endl;
        gX_new.col(link.parent->level_index)-=gX.col(ptr->level_index)*ptr->d_output*link.weight;
      }
    }
    std::swap(_gX_buffer_ptr[0], _gX_buffer_ptr[1]);
  }
}

void NNGaussNewtonOptimizer::computeFullNumericGradients(float epsilon) {
  _numeric_weight_gradients=Eigen::MatrixXf(_network.outputLayer().size(), _network.numWeights());
  for (size_t num_layer=_network.neurons.size()-1; num_layer>0; --num_layer) {
    auto& layer=_network.neurons[num_layer];
    for (auto& ptr : layer) {
      for (auto& l: ptr->links){
        Eigen::VectorXf out_plus(_network.outputLayer().size()), out_minus(_network.outputLayer().size());
        float w=l.weight;
        l.weight=w+epsilon;
        _network.forward();
        for (size_t o=0; o<_network.outputLayer().size(); ++o)
          out_plus[o]=_network.outputLayer()[o]->output;
    
        l.weight=w-epsilon;
        _network.forward();
        for (size_t o=0; o<_network.outputLayer().size(); ++o)
          out_minus[o]=_network.outputLayer()[o]->output;
        //cerr << out_plus-out_minus;
        _numeric_weight_gradients.col(l.index)=(out_plus-out_minus)/(2*epsilon);
        l.weight=w;
      }
    }
  }
}

  
float NNGaussNewtonOptimizer::trainOnce(const NeuralNetOptimizer::TrainSet& train_set) {
  int n=_network.numWeights();
  _H.resize(n,n);
  _b.resize(n);
  _H.setZero();
  _b.setZero();
  float chi_sum=0;
  cerr << "gradients: ";
  for (const auto& example: train_set) {
    _network.setInputs(example.first);
    setDesiredOutputs(example.second);
    _network.forward();
    float chi = eval();
    computeFullGradients();
    cerr << ".";
      
    _H+=_weight_gradients.transpose()*_weight_gradients;
    Eigen::VectorXf error(_network.outputLayer().size());
    for (int i=0; i< _network.outputLayer().size(); ++i) {
      error(i)=_network.outputLayer()[i]->output-_desired_outputs[i];
    }
    _b-=_weight_gradients.transpose()*error;
    chi_sum += chi;
  }
  cerr << endl << "solve";
  for (int i=0; i<n; ++i)
    _H(i,i)+=damping;
  Eigen::VectorXf dw = _H.ldlt().solve(_b);
    
  for (size_t num_layer=1; num_layer<_network.neurons.size(); ++num_layer) {
    auto& layer=_network.neurons[num_layer];
    for (auto& ptr : layer) {
      for (auto& l: ptr->links) {
        l.weight+=dw(l.index);
      }
    }
  }
  cerr << endl;
  return chi_sum;
}
