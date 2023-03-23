#include "nn_gn.h"
#include <iostream>
#include <fstream>
#include "neuron_factory.h"

using namespace std;

using SigmoidNeuron=Neuron_<SigmoidActivationFunction>;


int main(int argc, char**argv) {
  NeuralNet nn;
  
  int num_iterations=100;
  float damping=1;
  int train_set_size = 300;
  int num_neurons=10;

  // 3 layers the first is input only
  std::vector<int> layers={num_neurons, num_neurons, num_neurons, num_neurons};
  nn.init<SigmoidNeuron, SigmoidNeuron>(layers);

  // fully connect each layer with the next and initialize
  // random weights
  int levels=nn.neurons.size();
  for (int i=1; i<levels; ++i) {
    int num_nodes_previous=nn.neurons[i-1].size();
    int num_nodes_current=nn.neurons[i].size();
    for (int p=0; p<num_nodes_previous; ++p)
      for (int c=0; c<num_nodes_current; ++c)
        nn.addLink(i-1, p, i, c, drand48()-.5);
  }
  std::cerr << "num weights: " << nn.numWeights() << std::endl;
  ofstream os ("net.dat");
  nn.write(os);
  os.close();
  ifstream is("net.dat");
  nn.read(is);
  exit (0);
 
  /*
  // set a random input. The input layer's output is the input
  std::vector<float> inputs(nn.inputLayer().size());
  for (int i=0; i<nn.inputLayer().size(); ++i)
    inputs[i]=drand48()-.5;
  nn.setInputs(inputs);
  
  // set the net desired outputs
  std::vector<float> desired_outputs(nn.outputLayer().size());
  for (int i=0; i<nn.outputLayer().size(); ++i)
    desired_outputs[i]=drand48()-.5;
  nn.setDesiredOutputs(desired_outputs);
                       
  // do one round to check if the calculation of the gradients is correct
  nn.forward();
  nn.computeFullGradients();
  cerr << endl << endl;
  nn.computeFullNumericGradients(1e-3);
  cerr << (nn._numeric_weight_gradients-nn._weight_gradients).transpose() << endl;
  */
  NNGaussNewtonOptimizer gn_opt(nn);
  gn_opt.damping=damping;
  
  NeuralNetOptimizer::TrainSet  train_set;
  train_set.resize(train_set_size);
  
  for (auto& example: train_set) {
    example.first.resize(nn.inputLayer().size());
    for (size_t i=0; i<example.first.size(); ++i) {
      example.first[i]=drand48()-.5;
    }
    example.second.resize(nn.outputLayer().size());
    for (size_t i=0; i<example.second.size(); ++i) {
      example.second[i]=drand48()-.5;
    }
  }
  for (int i=0; i<num_iterations; ++i) {
    float chi2 = gn_opt.trainOnce(train_set);
    //float chi2 = nn.trainOnce(train_set, learning_rate);
    cout << i << " " << chi2/train_set.size() << endl;
  }
}
