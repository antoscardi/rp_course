#include "srrg2_slam_interfaces/instances.h"
#include <srrg_config/configurable_manager.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;

const std::string exe_name("convert_bench_results");
#define LOG std::cerr << exe_name + "|"

int main(int argc, char** argv) {
  srrg2_core::srrgInit(argc, argv, exe_name.c_str());
  srrg2_core::messages_registerTypes();
  srrg2_slam_interfaces::srrg2_slam_interfaces_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_dl_stub_file(&cmd, "dlc", "dl-config", "stub where to read the stub", "");
  ArgumentString arg_trajectory(&cmd, "t", "trajectory", "trajectory filename", "out.json");
  ArgumentString arg_format(&cmd, "f", "format", "output format { kitti , tum }", "kitti");

  cmd.parse();

  const std::string& format          = arg_format.value();
  const std::string& trajectory_file = arg_trajectory.value();

  if (!arg_trajectory.isSet()) {
    LOG << FG_RED("ERROR, not enough arguments");
    std::cerr << cmd.options() << std::endl;
    throw std::runtime_error(exe_name + "|ERROR, invalid shell arguments");
  }

  if (!isAccessible(trajectory_file)) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access trajectory file [" +
                             trajectory_file + " ]");
  }

  if (!isAccessible(arg_dl_stub_file.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access dynamic library file [" +
                             arg_dl_stub_file.value() + " ]");
  }

  if (format != "kitti" || format != "tum") {
    throw std::runtime_error(exe_name + "|ERROR, invalid formats. Choose between { kitti, tum }");
  }

  MessageFileSource source;
  source.open(trajectory_file);

  BaseSensorMessagePtr msg = nullptr;

  while ((msg = source.getMessage())) {
  }

  return 0;
}
