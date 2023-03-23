#pragma once
#include "neuron.h"
#include <map>
struct NeuronFactory {
  static std::unique_ptr<NeuronFactory>& instance() {return _instance;}
  
  struct NeuronCreatorBase {
    virtual NeuronBase* create() const = 0;
  };

  template <typename NeuronType>
  struct NeuronCreator_: public NeuronCreatorBase {
    NeuronBase* create() const override {return new NeuronType;}
  };

  template <typename NeuronType>
  void registerNeuron() {
    NeuronType new_item;
    std::string name=new_item.name();
    _creator_map[name]=std::unique_ptr<NeuronCreatorBase>(new NeuronCreator_<NeuronType>);
  }

  std::map<std::string, std::unique_ptr<NeuronCreatorBase> > _creator_map;
  
  NeuronBase* create(const std::string& tag ) {
    auto it=_creator_map.find(tag);
    if (it==_creator_map.end())
      return 0;
    return it->second->create();
  }
  static void init();
protected:
  static std::unique_ptr<NeuronFactory> _instance;
};

// called when so loaded
void _neuronFactoryInit() __attribute__((constructor));
