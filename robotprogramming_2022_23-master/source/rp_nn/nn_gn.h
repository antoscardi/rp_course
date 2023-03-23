#pragma once
#include "nn_optimizer.h"

class NNGaussNewtonOptimizer: public NeuralNetOptimizer {
public:
  NNGaussNewtonOptimizer(NeuralNet& nn): NeuralNetOptimizer(nn){
    setupBuffers();
    _weight_gradients=Eigen::MatrixXf(_network.outputLayer().size(), _network.numWeights());

  }
  
  void computeFullGradients();

  float trainOnce(const NeuralNetOptimizer::TrainSet& train_set);
    
  // debug only
  void computeFullNumericGradients(float epsilon);
  Eigen::MatrixXf _weight_gradients;
  Eigen::MatrixXf _numeric_weight_gradients;

  float damping =1;
protected:
  void setupBuffers();
  Eigen::MatrixXf _H;
  Eigen::VectorXf _b;
  // double buffer for projection matrix
  int _gX_rows, _gX_cols;
  std::vector<float > _gX_buffer[2];
  std::vector<float >* _gX_buffer_ptr[2];
};
