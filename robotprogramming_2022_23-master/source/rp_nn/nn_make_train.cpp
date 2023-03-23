#include <iostream>
#include <fstream>
#include <string>
#include "nn.h"
#include "neuron_factory.h"
#include "nn_gd.h"
using namespace std;

const char *banner[] = {
  "nn_make_train",
  "creates a batch of train set from a network",
  "  nn_make_train -o <train_file> -n num_samples <mode> <input file>"
  "  where mode is either r(andom) or (z)ero error",
  0
};

void printBanner (const char** b) {
  while (*b) {
    cerr << *b << endl;
    ++b;
  }
}

void listActivations() {
  auto& factory=NeuronFactory::instance();
  cerr << "Registered Activations" << endl;
  for (auto& it :factory->_creator_map ) {
    cerr << it.first << endl;
  }
  cerr << endl;
    
}

int main(int argc, char** argv) {
  if (argc<2) {
    printBanner(banner);
    return 0;
  }
  std::string output_name;
  std::string input_name;
  std::string mode="r";
  int num_samples;
  for (int c=1; c<argc; ++c) {
    std::string arg(argv[c]);
    if (arg == std::string("-z")) {
      mode="z";
    } else if (arg == std::string("-r")) {
      mode="r";
    } else if (arg == std::string("-o")) {
      ++c;
      if (c>=argc) {
        cerr << "no output file provided" << endl;
        return 0;
      }
      output_name = argv[c];
    } else if (arg == std::string("-n")) {
      ++c;
      if (c>=argc) {
        cerr << "no output file provided" << endl;
        return 0;
      }
      num_samples=atoi(argv[c]);
    } else {
      input_name=argv[c];
      break;
    }
  }

  cerr << "Reading input from file [" << input_name << "]" << endl;
  cerr << "Writing output to  file [" << output_name << "]" <<  endl;
  cerr << "mode: " << mode <<  endl;
  cerr << "num_samples: " << num_samples <<  endl;
  NeuralNet nn;
  ifstream is(input_name);
  if (! is.good()) {
    cerr << "cannot read input file" << endl;
    return 0;
  }
  nn.read(is);
  NeuralNetOptimizer::TrainSet  train_set;
  train_set.resize(num_samples);
  for (auto& example: train_set) {
    example.first.resize(nn.inputLayer().size());
    for (size_t i=0; i<example.first.size(); ++i) {
      example.first[i]=drand48()-.5;
    }
    example.second.resize(nn.outputLayer().size());
    for (size_t i=0; i<example.second.size(); ++i) {
      example.second[i]=drand48()-.5;
    }
    if (mode == "z") {
      nn.setInputs(example.first);
      nn.forward();
      for (size_t i=0; i<example.second.size(); ++i) {
        example.second[i]=nn.outputLayer()[i]->output;
      }
    }
  }
  NNGradientDescentOptimizer gd_opt(nn);

  if (!output_name.length()) {
    cerr << "no output to write" << endl;
    return 0;
  }
  
  cerr << "writing output ...";
  ofstream os (output_name);
  gd_opt.writeTrainSet(train_set, os);
  cerr << "done" << endl;

}
