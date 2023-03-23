#pragma once
#include "nn_optimizer.h"

class NNGradientDescentOptimizer: public NeuralNetOptimizer {
public:
  NNGradientDescentOptimizer(NeuralNet& nn): NeuralNetOptimizer(nn) {}
  
  void computeGradients();  
  void applyGradients();  
  float trainOnce(const NeuralNetOptimizer::TrainSet& train_set) override;
  const Eigen::VectorXf& weightGradients() const {return _weight_gradients;}
  float learning_rate=0.01;
protected:
  Eigen::VectorXf _weight_gradients;
  Eigen::VectorXf _forward_gradients;
};
