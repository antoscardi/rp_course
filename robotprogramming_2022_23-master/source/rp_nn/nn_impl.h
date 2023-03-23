
template <typename IntermediateNeuronType_, typename OutputNeuronType_>
void NeuralNet::init(std::vector<int>& neurons_per_layer) {
  neurons.clear();
  neurons.resize(neurons_per_layer.size());
  for(size_t num_layer=0; num_layer < neurons_per_layer.size(); ++num_layer) {
    int num_neurons_in_layer=neurons_per_layer[num_layer];
    auto& layer=neurons[num_layer];
    layer.resize(num_neurons_in_layer);
    for (int num_neuron=0; num_neuron<num_neurons_in_layer; ++num_neuron) {
      auto& ptr=layer[num_neuron];
      if (num_layer==0) 
        ptr.reset(new NeuronBase);
      else if (num_layer<neurons_per_layer.size()-1)
        ptr.reset(new IntermediateNeuronType_);
      else
        ptr.reset(new OutputNeuronType_);
      ptr->level=num_layer;
      ptr->level_index=num_neuron;
    }
  }
}
