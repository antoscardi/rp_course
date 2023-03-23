#include "nn.h"
#include "nn_gd.h"

#include <iostream>

using namespace std;

using SigmoidNeuron=Neuron_<SigmoidActivationFunction>;


int main(int argc, char**argv) {
  NeuralNet nn;

  // parameters
  int num_iterations=1000;
  float learning_rate=0.01;
  int train_set_size = 5;
  int num_neurons=100;

  // 3 layers the first is input only
  std::vector<int> layers={num_neurons, num_neurons, num_neurons, num_neurons};
  nn.init<SigmoidNeuron, SigmoidNeuron>(layers);

  cerr << "building network..." << endl;
  // fully connect each layer with the next and initialize
  // random weights
  int levels=nn.neurons.size();
  for (int i=1; i<levels; ++i) {
    cerr << "linking layer" << i << " " << i+1 << endl;
    int num_nodes_previous=nn.neurons[i-1].size();
    int num_nodes_current=nn.neurons[i].size();
    for (int p=0; p<num_nodes_previous; ++p)
      for (int c=0; c<num_nodes_current; ++c)
        nn.addLink(i-1, p, i, c, drand48()-.5);
  }
  cerr << "done" << endl;
  cerr << "num weights: " << nn.numWeights() << endl;

  // set a random input. The input layer's output is the input
  for (int i=0; i<nn.inputLayer().size(); ++i)
    nn.inputLayer()[i]->output=drand48()-.5;

  NNGradientDescentOptimizer gd_opt(nn);
  gd_opt.learning_rate=learning_rate;

  /* to debug gradients*/
  // // set the net desired outputs
  // std::vector<float> desired_outputs(nn.outputLayer().size());
  // for (int i=0; i<nn.outputLayer().size(); ++i)
  //   desired_outputs[i]=drand48()-.5;
  // gd_opt.setDesiredOutputs(desired_outputs);
                       
  // // do one round to check if the calculation of the gradients is correct
  // nn.forward();
  // gd_opt.computeGradients();
  
  // // get the link between layer 2 and 3, neurons 2 and 2.
  // // print the weight and the gradient, and  compute the
  // // numerical derivative of the gradient to check if done well
  // auto& l=nn.link(2,1,3,2);
  // cerr << "w: " << l.weight << " " << gd_opt.weightGradients()(l.index) << endl;
  // float epsilon=1e-2;
  // float w_zero=l.weight;
  // l.weight=w_zero+epsilon;
  // nn.forward();
  // float chi_plus=gd_opt.eval();
  // cerr << "chi_plus: " << chi_plus << endl;
  // l.weight=w_zero-epsilon;
  // nn.forward();
  // float chi_minus=gd_opt.eval();
  // cerr << "chi_plus: " << chi_minus << endl;
  // cerr << "numeric_gradient: " << (chi_plus-chi_minus)/(2*epsilon);
  
  // create training set
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
  // train
  for (int i=0; i<num_iterations; ++i) {
    float chi2 = gd_opt.trainOnce(train_set);
    cout << i << " " << chi2/train_set.size() << endl;
  }
}
