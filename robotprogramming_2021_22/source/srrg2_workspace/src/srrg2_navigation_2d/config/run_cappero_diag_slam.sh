WS_ROOT="${ROSLISP_PACKAGE_DIRECTORIES%share*}"
echo "WS_ROOT=${WS_ROOT}"
source ${WS_ROOT}/setup.bash
echo "sourcing ws=${WS_ROOT}"
rosrun srrg2_executor srrg2_shell -vt shared run_cappero_diag_slam.srrg
