
// inline dot product
float NeuronBase::dot() const {
  float input_sum=0;
  for(auto& l : links) {
    input_sum+=l.parent->output*l.weight;
  }
  return input_sum;
}
