#include <iostream>
struct VecF {
  int size; float* v;
  float get(int i) { return v[i];}
  void  set(int i,float f) {v[i]=f;}
  VecF() { size=0; v=nullptr;}
  VecF(int size){
    this->size=size; 
    v=new float[size];
  }

  VecF(const VecF& other) {
    if (! other.size) return;
    size=other.size; v=new float[size];
    for (int i=0; i<size; ++i)
      v[i]=other.v[i];
  }
  ~VecF() {if (size) delete [] v;}

  VecF& operator =(const VecF& other) {
    if (size) delete[] v; size=0; v=0;
    if (! other.size) return *this;
    size=other.size; v=new float[size];
    for (int i=0; i<size; ++i) v[i]=other.v[i];
  }
};

int main() {
  VecF v5(5);
  v5.set(0,0.1);
  v5.set(1,0.2);
  VecF v7(v5); //(copy ctor)
  VecF v8=v5;  //(copy ctor)
  v8=v7;       // op=   
}
