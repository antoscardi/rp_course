#pragma once

#include "object_writer.h"

namespace srrg2_core {

class JSONObjectWriter: public ObjectWriter {
public:
  virtual void writeObject(std::ostream& os, const std::string& type, ObjectData& object);
protected:
  int indentation_level=0;
};

}
