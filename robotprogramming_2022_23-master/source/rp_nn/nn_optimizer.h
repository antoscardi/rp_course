#pragma once
#include "nn.h"

class NeuralNetOptimizer {
public:
  using TrainSet = std::vector<std::pair<std::vector<float>, std::vector<float> > >;
  NeuralNetOptimizer(NeuralNet& network);
  void setDesiredOutputs(const std::vector<float>& des_out);
  float eval();
  virtual float trainOnce(const NeuralNetOptimizer::TrainSet& train_set)=0;
  void writeTrainSet(const TrainSet& train_set, ostream& os);
  void readTrainSet(TrainSet& train_set, istream& is);
  
  std::vector<float> _desired_outputs;
protected:
  NeuralNet& _network;
};
