#include "srrg2_slam_interfaces/instances.h"
#include <srrg_benchmark/trajectory_writers.h>
#include <srrg_config/configurable_manager.h>
#include <srrg_messages/instances.h>
#include <srrg_system_utils/parse_command_line.h>
#include <srrg_system_utils/shell_colors.h>

using namespace srrg2_core;
using namespace srrg2_slam_interfaces;

const std::string exe_name("app_benchmark");
#define LOG std::cerr << exe_name + "|"

bool requires_platform = false;
bool has_platform      = false;
using LocalMapType     = MultiGraphSLAM3D::LocalMapType;

struct StampedIsometry3f {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StampedIsometry3f(const double& ts_, const Isometry3f& iso_) {
    timestamp = ts_;
    isometry  = iso_;
  }
  double timestamp    = -1.f;
  Isometry3f isometry = Isometry3f::Identity();
};

using StampedIsometry3fVector =
  std::vector<StampedIsometry3f, Eigen::aligned_allocator<StampedIsometry3f>>;
using LocalMapLocalPosesMap =
  std::map<LocalMapType*,
           StampedIsometry3fVector,
           std::less<LocalMapType*>,
           Eigen::aligned_allocator<std::pair<LocalMapType*, StampedIsometry3fVector>>>;
using TimestampLocalMapUMap = std::unordered_map<double, LocalMapType*>;

void unrollFullTrajectory(TimestampIsometry3fMap& unrolled_trajectory_,
                          const LocalMapLocalPosesMap& full_trajectory_);

int main(int argc, char** argv) {
  srrg2_core::srrgInit(argc, argv, exe_name.c_str());
  srrg2_core::messages_registerTypes();
  srrg2_slam_interfaces::srrg2_slam_interfaces_registerTypes();

  ParseCommandLine cmd(argv);
  ArgumentString arg_dl_stub_file(&cmd, "dlc", "dl-config", "stub where to read the stub", "");
  ArgumentString arg_config_name(&cmd, "c", "config", "configuration to use in this bench", "");
  ArgumentString arg_dataset(&cmd, "d", "dataset", "dataset filename BOSS ONLY", "");
  ArgumentString arg_pipeline_name(
    &cmd, "pn", "pipeline", "pipeline name in the config", "pipeline");
  ArgumentString arg_source_name(&cmd, "sn", "source", "source name in the config", "source");
  ArgumentString arg_slam_name(&cmd, "sl", "slam", "slam name in the config", "slam");
  ArgumentString arg_output_name(&cmd, "o", "output", "output file name (no extension)", "out");
  ArgumentString arg_format(&cmd, "f", "format", "output format { kitti , tum }", "kitti");
  cmd.parse();

  const std::string& format      = arg_format.value();
  const std::string& output_name = arg_output_name.value();

  if (!arg_config_name.isSet() || !arg_dataset.isSet() || !arg_dl_stub_file.isSet()) {
    LOG << FG_RED("ERROR, not enough arguments\n");
    throw std::runtime_error(exe_name + "|ERROR, invalid shell arguments");
  }

  if (!isAccessible(arg_config_name.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access config file [" +
                             arg_config_name.value() + " ]");
  }

  if (!isAccessible(arg_dataset.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access dataset [" + arg_dataset.value() +
                             " ]");
  }
  if (!isAccessible(arg_dl_stub_file.value())) {
    throw std::runtime_error(exe_name + "|ERROR, cannot access dynamic library file [" +
                             arg_dl_stub_file.value() + " ]");
  }

  if (format != "kitti" && format != "tum") {
    throw std::runtime_error(exe_name + "|ERROR, invalid format " + format +
                             ". Choose between { kitti, tum }");
  }

  ConfigurableManager::initFactory(arg_dl_stub_file.value());
  // ia load config and get objects
  LOG << "reading configuration [ " << arg_config_name.value() << " ]\n";
  ConfigurableManager manager;
  manager.read(arg_config_name.value());

  auto source_ptr   = manager.getByName<MessageFileSourceBase>(arg_source_name.value());
  auto pipeline_ptr = manager.getByName<MessageSinkBase>(arg_pipeline_name.value());
  auto slammer_ptr  = manager.getByName<MessageSinkBase>(arg_slam_name.value());

  if (!pipeline_ptr || !source_ptr || !slammer_ptr) {
    throw std::runtime_error(exe_name + "|ERROR, invalid config");
  }
  std::string filename = arg_dataset.value();

  LOG << "opening dataset [ " << filename << " ]\n";
  source_ptr->open(filename);

  std::string benchamin("benchamin");
  auto benchamin_ptr = manager.create<MapListener>(benchamin);

  slammer_ptr->param_push_sinks.pushBack(benchamin_ptr);

  // ia start processing the thing
  LOG << "start processing\n";
  BaseSensorMessagePtr msg = nullptr;

  while ((msg = source_ptr->getMessage())) {
    pipeline_ptr->putMessage(msg);
  }

  std::string traj_type("");

  auto graph = benchamin_ptr->graph();
  if (graph && graph->variables().size()) {
    if (dynamic_cast<LocalMap2D const*>(graph->variables().begin().value())) {
      traj_type = "2D";
    } else if (dynamic_cast<LocalMap3D const*>(graph->variables().begin().value())) {
      traj_type = "3D";
    }
  }

  Trajectory3D output_trajectory;

  if (traj_type == "2D") {
    Trajectory2D trj;
    benchamin_ptr->computeTrajectory(trj);
    for (const auto& t : trj) {
      output_trajectory.insert(std::make_pair(t.first, geometry3d::get3dFrom2dPose(t.second)));
    }
  } else if (traj_type == "3D") {
    Trajectory3D trj;
    benchamin_ptr->computeTrajectory(trj);
    output_trajectory = std::move(trj);
  }

  if (format == "kitti") {
    writeTrajectoryToFileKITTI(output_trajectory, output_name + "_" + format + ".txt");
  } else if (format == "tum") {
    writeTrajectoryToFileTUM(output_trajectory, output_name + "_" + format + ".txt");
  }

  return 0;
}

void unrollFullTrajectory(TimestampIsometry3fMap& unrolled_trajectory_,
                          const LocalMapLocalPosesMap& full_trajectory_) {
  unrolled_trajectory_.clear();
  for (const auto& entry : full_trajectory_) {
    for (const auto& stamped_pose : entry.second) {
      const Isometry3f global_pose = entry.first->estimate() * stamped_pose.isometry;
      unrolled_trajectory_.insert(std::make_pair(stamped_pose.timestamp, global_pose));
    }
  }
}
