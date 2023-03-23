#include <string>
#include <map>

void initEnvMap();
  
// returns the environment variables
const std::map<std::string, std::string>& envMap();

  
// replaces in a string the tags "<VAR_KEY>" with "VAR_VALUE"
// according to the environment map;
void replaceEnvTags(std::string& str);
