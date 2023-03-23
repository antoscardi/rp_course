#include <iostream>
#include <fstream>
#include <string>
#include "nn.h"
#include "neuron_factory.h"

using namespace std;

const char *banner[] = {
  "nn_make_net",
  "creates a file with the network topology as specified in the input file",
  "sintax of the input file (for each line)"
  " <type of activation> <number of neurons in 1st layer>"
  " <type of activation> <number of neurons in 2nd layer>"
  " ... "
  " <type of activation> <number of neurons in last layer>"
  "",
  "The first layer needs to be of type Input, while the others ",
  "can be of all activations supported",
  "To list the activations supported type",
  "  nn_make_net -l"
  "",
  " usage: ",
  " nn_make_net -o <output_file> input_file ",
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
  
  for (int c=1; c<argc; ++c) {
    std::string arg(argv[c]);
    if (arg == std::string("-l")) {
      listActivations();
      return 0;
    } else if (arg == std::string("-o")) {
      ++c;
      if (c>=argc) {
        cerr << "no output file provided" << endl;
        return 0;
      }
      output_name = argv[c];
    } else {
      input_name=argv[c];
      break;
    }
  }

  cerr << "Reading input from file [" << input_name << "]" << endl;
  cerr << "Writing output to  file [" << output_name << "]" <<  endl;
  NeuralNet nn;
  ifstream is(input_name);
  if (! is.good()) {
    cerr << "cannot read input file" << endl;
    return 0;
  }
  int num_layer = 0;
  auto& factory=NeuronFactory::instance();
  while (is.good()) {
    char buf[1024];
    memset(buf,0, 1024);
    is.getline(buf, 1024);
    istringstream ls(buf);
    std::string tag;
    int layer_size;
    ls >> tag >> layer_size;
    if (num_layer==0 && tag != "Input") {
      cerr << "error, first layer should be an Input, not " << tag <<"]" << endl;
      return 0;
    }
    NeuronBase* n=factory->create(tag);
    if (!n) {
      cerr << "unknown neuron of type " << tag << " aborting" << endl;
      continue;
    }
    delete n;
    nn.neurons.resize(nn.neurons.size()+1);
    auto& layer=nn.neurons.back();
    layer.resize(layer_size);
    for (int i=0; i<layer_size; ++i) {
      NeuronBase* n=factory->create(tag);
      nn.addNeuron(num_layer, i, n);
    }
    ++num_layer;
  }

  //done with the layers
  // now fully connect a layer with the next
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
  if (output_name.length()) {
    cerr << "writing output ...";
  }
  ofstream os (output_name);
  nn.write(os);
  cerr << "done" << endl;
  os.close();
  {
    ifstream is(output_name);
    nn.read(is);
    ofstream os2("nn_test");
    nn.write(os2);
  }
}
