#include "nn_optimizer.h"


NeuralNetOptimizer::NeuralNetOptimizer(NeuralNet& network_):
  _network(network_){
  _network.enumerate();
  _desired_outputs.resize(_network.outputLayer().size());
}

  
float NeuralNetOptimizer::eval() {
  auto& output_layer=_network.neurons.back();
  float chi2=0;
  for (size_t i=0; i<output_layer.size(); ++i) {
    auto& ptr = output_layer[i];
    float e=ptr->output-_desired_outputs[i];
    chi2 += e*e;
  }
  return chi2;
}


void NeuralNetOptimizer::setDesiredOutputs(const std::vector<float>& des_out) {
  if (des_out.size()!=_desired_outputs.size())
    throw std::runtime_error("train output size does not match net size");
  _desired_outputs=des_out;
}

void NeuralNetOptimizer::writeTrainSet(const TrainSet& train_set, ostream& os) {
  os << _network.inputLayer().size() << " "
     << _network.outputLayer().size() << " "
     << train_set.size() << endl;
  for (const auto& t: train_set) {
    for (const auto& v: t.first) {
      os << v << " ";
    }
    os << endl;
    for (const auto& v: t.second) {
      os << v << " ";
    }
    os << endl;
  };
};

void NeuralNetOptimizer::readTrainSet(TrainSet& train_set, istream& is) {
  int input_size, output_size, num_samples;
  char buf[1024];
  is.getline(buf, 1024);
  istringstream input_h(buf);
  input_h >> input_size >> output_size >> num_samples;
  if (input_size!=_network.inputLayer().size()) {
    throw std::runtime_error("input size mismatch");
  }
  if (output_size!=_network.outputLayer().size()) {
    throw std::runtime_error("output size mismatch");
  }
  cerr << "preparing to load " << num_samples << " samples" << endl;
  train_set.resize(num_samples);
  for (int i=0; i<num_samples && is.good(); ++i) {
    char buf[1024];
    is.getline(buf, 1024);
    istringstream input_s(buf);
    train_set[i].first.resize(input_size);
    train_set[i].second.resize(output_size);
    int k=0;
    while (input_s.good() && k<input_size) {
      input_s >> train_set[i].first[k];
      ++k;
    }
    is.getline(buf, 1024);
    istringstream output_s(buf);
    k=0;
    while (output_s.good() && k<output_size) {
      output_s >> train_set[i].second[k];
      ++k;
    }
  }
}
