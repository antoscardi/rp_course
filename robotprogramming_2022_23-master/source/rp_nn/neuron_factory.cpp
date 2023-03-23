#include "neuron_factory.h"

std::unique_ptr<NeuronFactory> NeuronFactory::_instance;

void NeuronFactory::init() {
  _instance.reset(new NeuronFactory);
  _instance->registerNeuron<NeuronBase>();
  _instance->registerNeuron< Neuron_<SigmoidActivationFunction> >();
}

void _neuronFactoryInit() {
  NeuronFactory::init();
}
