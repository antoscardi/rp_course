#include <iostream>
#include <cmath>
#include <list>
#include <set>
#include <cmath>
#include <vector>
#include <memory>
#include "nn.h"
#include <sstream>
#include "neuron_factory.h"
  
  
void NeuralNet::forward() {
  for (size_t num_layer=1; num_layer<neurons.size(); ++num_layer) {
    auto& layer=neurons[num_layer];
    for (auto& ptr : layer)
      ptr->forward();
  }
}

void NeuralNet::checkLink(int parent_level, int parent_pos,
                          int child_level, int child_pos){
  if (parent_level>=child_level)
    throw std::runtime_error("parent has a higer or equal level than child. this is wrong");
  if (parent_level<0 || parent_level>=neurons.size())
    throw std::runtime_error("parent level out of bounds");
  if (child_level<0 || child_level>=neurons.size())
    throw std::runtime_error("child level out of bounds");
  if (parent_pos<0 || parent_pos>= neurons[parent_level].size())
    throw std::runtime_error("parent out of bounds in level");
  if (child_pos<0 || child_pos>= neurons[child_level].size())
    throw std::runtime_error("child out of bounds in level");

}

void NeuralNet::addNeuron(int level, int level_pos, NeuronBase* n) {
  if (level<0 || level >= neurons.size())
    throw std::runtime_error("num levels overflow");
  if (level_pos<0 || level_pos >= neurons[level].size())
    throw std::runtime_error("in levels overflow");
  n->level=level;
  n->level_index=level_pos;
  neurons[level][level_pos].reset(n);
}

NeuronBase::Link* NeuralNet::linkPtr(int parent_level, int parent_pos,
                                     int child_level, int child_pos) {
  checkLink(parent_level, parent_pos, child_level, child_pos);
  auto parent_ptr=neurons[parent_level][parent_pos].get();
  auto child_ptr=neurons[child_level][child_pos].get();
   auto it=_link_map.find(std::make_pair(parent_ptr, child_ptr));
  if (it==_link_map.end())
    return 0;
  return &child_ptr->links[it->second];
}
  
NeuronBase::Link& NeuralNet::link(int parent_level, int parent_pos,
                                  int child_level, int child_pos) {
  auto l=linkPtr(parent_level, parent_pos, child_level, child_pos);
  if (! l)
    throw std::runtime_error("no link");
  return *l;
}
  
void NeuralNet::addLink(int parent_level, int parent_pos,
                        int child_level, int child_pos, float weight){
  auto l=linkPtr(parent_level, parent_pos, child_level, child_pos);
  auto parent_ptr=neurons[parent_level][parent_pos].get();
  auto child_ptr=neurons[child_level][child_pos].get();
  if (!l) {
    _num_weights=-1;
    _link_map.insert(std::make_pair(std::make_pair(parent_ptr, child_ptr), child_ptr->links.size()));
    child_ptr->links.push_back(NeuronBase::Link(parent_ptr, weight));
  } else {
    l->weight=weight;
  }
}

void NeuralNet::setInputs(const std::vector<float>& inputs) {
  if (inputs.size()!=inputLayer().size()) {
    cerr << "inputs.size: " << inputs.size() << " layer:" << inputLayer().size() << endl;
    throw std::runtime_error("train input size does not match net size");
  }
  for (size_t i=0; i<inputLayer().size(); ++i)
    inputLayer()[i]->output=inputs[i];
}

int NeuralNet::numWeights() {
  if (_num_weights<0)
    enumerate();
  return _num_weights;
}

int NeuralNet::numNeurons() {
  if (_num_weights<0)
    enumerate();
  return _num_neurons;
}

void NeuralNet::enumerate() {
  int current_weight_index=0;
  int current_neuron_index=0;
  for (size_t num_layer=neurons.size()-1; num_layer>0; --num_layer) {
    auto& layer=neurons[num_layer];
    for (auto& ptr : layer) {
      ptr->index=current_neuron_index;
      ptr->level=num_layer;
      for (auto& l: ptr->links){
        l.index=current_weight_index;
        ++ current_weight_index;
      }
      ++current_neuron_index;
    }
  }
  _num_weights=current_weight_index;
  _num_neurons=current_neuron_index;
}

void NeuralNet::write(std::ostream& os) {
  // write number of neurons
  enumerate();
  os << neurons.size() <<" ";
  for (auto & l: neurons)
    os << l.size() << " ";
  os << endl;
  
  // write each neuron, tag, level, position
  for (auto& level : neurons) {
    for (auto& ptr: level)
      os << ptr->name() << " " << ptr->level << " " << ptr->level_index << endl;
  }
  os << numWeights() << endl;
  for (size_t i=1; i<neurons.size(); ++i) {
    auto& level=neurons[i];
    for (size_t j=0; j<level.size(); ++j) {
      auto& ptr=level[j];
      for (auto& l: ptr->links) {
        os << l.parent->level << " "<< l.parent->level_index << " "
           << ptr->level << " " << ptr->level_index << " " << l.weight << endl;
      }
    }
  }
}

void NeuralNet::read(std::istream& is) {
  auto& factory=NeuronFactory::instance();
  neurons.clear();
  _link_map.clear();
  int levels;
  is >> levels;
  neurons.resize(levels);
  int n_to_read=0;
  cerr << "num levels: " << levels;
  for (int i=0; i<levels; ++i) {
    int level_size;
    is >> level_size;
    neurons[i].resize(level_size);
    n_to_read+=level_size;
    cerr << " neurons at level " << i << ": " << level_size << endl;
  }
  cerr << "reading the neurons proper" << n_to_read << endl;
  while (n_to_read && is.good()) {
    char buf[1024];
    is.getline(buf, 1024);
    istringstream ss(buf);
    std::string tag;
    int level, level_index;
    ss >> tag;
    ss >> level;
    ss >> level_index;
    auto n = factory->create(tag);
    if (! n){
      cerr << "no neuron of type [" << tag << "] skipping" << endl;
      continue;
    }
    //cerr << "type: " << tag << " level: " << level << " level_index: " << level_index << endl;
    n->level=level;
    n->level_index=level_index;
    neurons[level][level_index].reset(n);
    --n_to_read;
  }
  cerr << "to read: " << n_to_read << endl; 
  int num_weights;
  is >> num_weights;
  cerr << "reading " << num_weights << " weights" << endl;
  while (num_weights && is.good()) {
    char buf[1024];
    memset(buf, 0, 1024);
    is.getline(buf, 1024);
    if (strlen(buf)==0)
      continue;
    istringstream ss(buf);
    int parent_level, parent_index, level, level_index;
    float weight;
    if (ss.good()) {
      ss >> parent_level >> parent_index >> level >> level_index >> weight;
      addLink(parent_level, parent_index, level, level_index, weight);
      auto& l=link(parent_level, parent_index, level, level_index);
      auto& ptr=neurons[level][level_index];
      --num_weights;
    }
  }
}

