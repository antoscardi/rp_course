#include <iostream>
#include <fstream>
#include <string>
#include "nn.h"
#include "neuron_factory.h"
#include "nn_gd.h"
#include "nn_gn.h"
#include "nn.h"

const char *banner[] = {
  "nn_optimize",
  "optimizes a net in a file, based on a set of examples",
  "the examples are stored in a file",
  " each odd line of the files contains the inputs",
  " each even line of the files contains the outputs corresponding to the inputs above",
  " they can be generated with nn_make_train",
  "",
  "",
  " usage: ",
  " nn_optimize -o <output_file> -i <iterations> -t <train_file> [-gn|-gd] -d <damping/learning rate> input_file ",
  "   -gn  or -gd select between gauss newton and gradient descent",
  "   gn is more powerful bur works only on small nets",
  0
};

void printBanner (const char** b) {
  while (*b) {
    cerr << *b << endl;
    ++b;
  }
}

int main(int argc, char** argv) {
  if (argc<2) {
    printBanner(banner);
    return 0;
  }
  std::string output_name;
  std::string input_name;
  std::string train_name;
  std::string mode="gd";
  int iterations=100;
  float float_param;
  for (int c=1; c<argc; ++c) {
    std::string arg(argv[c]);
    if (arg == std::string("-gn")) {
      mode="gn";
    } else if (arg == std::string("-gd")) {
      mode="gd";
    } else if (arg == std::string("-o")) {
      ++c;
      if (c>=argc) {
        cerr << "no output file provided" << endl;
        return 0;
      }
      output_name = argv[c];
    } else if (arg == std::string("-t")) {
      ++c;
      if (c>=argc) {
        cerr << "no train file provided" << endl;
        return 0;
      }
      train_name = argv[c];
    } else if (arg == std::string("-i")) {
      ++c;
      if (c>=argc) {
        cerr << "no iterations set" << endl;
        return 0;
      }
      iterations=atoi(argv[c]);
    } else if (arg == std::string("-d")) {
      ++c;
      if (c>=argc) {
        cerr << "no damping/learning rate" << endl;
        return 0;
      }
      float_param=atof(argv[c]);
    } else {
      input_name=argv[c];
      break;
    }
  }

  NeuralNet nn;
  ifstream is(input_name);
  if (! is.good()) {
    cerr << "cannot read input file" << endl;
    return 0;
  }
  nn.read(is);

  NeuralNetOptimizer* opt=0;
  if (mode == "gn") {
    NNGaussNewtonOptimizer* gn=new NNGaussNewtonOptimizer(nn);
    gn->damping= float_param;
    opt=gn;
    
  } else if (mode == "gd") {
    NNGradientDescentOptimizer* gd=new NNGradientDescentOptimizer(nn);
    gd->learning_rate=float_param;
    opt=gd;
  }
  if (! opt) {
    cerr << "no optimizer selected" << endl;
    return 0;
  }

  ifstream ts(train_name);
  if (! ts.good()) {
    cerr << "cannot read train file" << endl;
    return 0;
  }

  NeuralNetOptimizer::TrainSet train_set;
  opt->readTrainSet(train_set, ts);

  for (int i=0; i<iterations; ++i) {
    float chi2 = opt->trainOnce(train_set);
    cout << i << " " << chi2/train_set.size() << endl;
  }

  ofstream os(output_name);
  if (os.good()) {
    nn.write(os);
  }
  
  
}
