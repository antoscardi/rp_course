#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/joystick.h>
#include <cstdio>
#include <string>
#include <iostream>
 
using namespace std;

int main(int argc, char** argv) {
  if (argc<2) {
    cerr << "usage: " << argv[0] << " <joy_device>" << endl;
    return 0;
  }
  std::string joy_device = argv[1];

   int fd = open (joy_device.c_str(), O_RDONLY);
  if (fd<0) {
    cerr << "no joy found" << endl;
    return 0;
  }

  struct js_event e;
  while(1){
    while (read (fd, &e, sizeof(e)) > 0 ){
      printf ("EVENT time: %u, axis: %d, type: %d, value: %d\n",
	      e.time,
	      e.number,
	      e.type,
	      e.value);
    }
  }
}
