#include "graph_generator.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_se3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/variable_point3_ad.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_pose_geodesic_error_factor.h"
#include "srrg_solver/variables_and_factors/types_3d/se3_pose_point_offset_error_factor.h"
#include "srrg_solver/variables_and_factors/types_projective/variable_sim3_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/sim3_pose_pose_error_factor_ad.h"
#include "srrg_solver/variables_and_factors/types_projective/se3_pose_point_omni_ba_error_factor.h"

namespace srrg2_solver {
  using namespace std;

  GraphGeneratorBase::~GraphGeneratorBase() {}
  
  void GraphGeneratorBase::compute() {
    
    landmarks_num_seen.resize(sim.landmarks.size());
    std::fill(landmarks_num_seen.begin(), landmarks_num_seen.end(), 0);
    landmark_vars.resize(sim.landmarks.size());
    std::fill(landmark_vars.begin(), landmark_vars.end(), nullptr);
    pending_landmark_factors.resize(sim.landmarks.size());

    SimRecordPtr  record (new SimRecord);
    record->epoch=-1;
    
    // cerr << "generating graph for incremental optimization" << endl;
    
    graph=FactorGraphPtr (new FactorGraph);
    // cerr << "adding sensor offset fixed var" << endl;
    // add the offset
    
    VariableBase* offset_var = createLandmarkSensorOffsetVariable();
    graph->addVariable(VariableBasePtr(offset_var));
    landmark_idx_offset=sim.poses.size()+1;

    record->vars.push_back(offset_var->graphId());
    records.push_back(record);

    // process the poses sequentially,
    // for each pose:
    //     add a new variable
    //     if needed add an odometry factor
    //     update the landmark seen


    size_t la_idx=0;
    size_t pa_idx=0;

    VariableBase* pose_prev=0;
    for (int idx_from=0; idx_from< (int) sim.poses.size(); ++idx_from) {
      
      // cerr << "processing pose: " << idx_from << endl;
      SimRecordPtr  record (new SimRecord);
      record->epoch=idx_from;
      
      VariableBase* pose_var=createPoseVariable(idx_from);
      graph->addVariable(VariableBasePtr(pose_var));

      if (! pose_prev)
        pose_var->setStatus(VariableBase::Fixed);
      record->vars.push_back(pose_var->graphId());

      // odometry
      if (has_odom && pose_prev) {
        // cerr << "\tmaking odometry: " << pose_prev->graphId() << " -> " << pose_var->graphId() << endl;
        FactorBase* odom_factor=createPoseFactor(pose_prev->graphId(), pose_var->graphId());
        graph->addFactor(FactorBasePtr(odom_factor));

        record->factors.push_back(odom_factor->graphId());
      }
      pose_prev=pose_var;

      // cerr << "scanning landmarks " << endl;      //landmarks
      while(la_idx<sim.landmark_associations.size()) {
        const IntPair& l_ass=sim.landmark_associations[la_idx];
        if (l_ass.first!=idx_from)
          break;
        
        // the first time a landmark is seen we make the variable, otherwise
        // we retrieve it
        int idx_to=l_ass.second;
        ++la_idx;
        
        // cerr << "\tlandmark tmeasurement "<< idx_from << " -> " << landmark_idx_offset+idx_to << " added to pending list" << endl;

        // cerr << "\t\tlandmark:  " << landmark_idx_offset+idx_to ;
        
        VariableBase* landmark_var=landmark_vars[idx_to];
        if (!landmark_var) {
          landmark_var=createLandmarkVariable(idx_to);
          landmark_vars[idx_to]=landmark_var;
        }
        
        //we add the factor to the pending list
        FactorBase* landmark_factor=createLandmarkFactor(idx_from, idx_to, offset_var->graphId());
        if (! landmark_factor)
          continue;
        
        pending_landmark_factors[idx_to].push_back(landmark_factor);
        
        bool has_variable=graph->variable(landmark_var->graphId());
        if (! has_variable &&
            (int) pending_landmark_factors[idx_to].size()<landmark_min_measurements) {
          // cerr << "\t\tskip adding (mm)  " <<  endl;
          continue;
        }
        if (! has_variable) {
          // cerr << "\t\tlandmark added" << endl;
          graph->addVariable(VariableBasePtr(landmark_var));
          record->vars.push_back(landmark_var->graphId());
        }
        // cerr << "\t\tflush: [ ";
        for (auto this_factor: pending_landmark_factors[idx_to]) {
          
          // cerr << "("    << this_factor->variableId(0)
          //               << " -> " << this_factor->variableId(1) << ") ";
          graph->addFactor(FactorBasePtr(this_factor));
          record->factors.push_back(this_factor->graphId());
        }
        // cerr << "]" << endl;
        pending_landmark_factors[idx_to].clear();
      }

      //poses
      while(pa_idx<sim.pose_associations.size()) {
        const IntPair& p_ass=sim.pose_associations[pa_idx];
        if (p_ass.first!=idx_from)
          break;
        int idx_to=p_ass.second;
        ++pa_idx;
        FactorBase* pose_factor=createPoseFactor(idx_from, idx_to);
        graph->addFactor(FactorBasePtr(pose_factor));
        // cerr << "\tpose tmeasurement "<< idx_from << " -> " << idx_to << " added" << endl;
        record->factors.push_back(pose_factor->graphId());
      }
      records.push_back(record);
    }
  }

  //======================== SE3 generator ========================

  VariableBase* GraphGeneratorSE3::createLandmarkSensorOffsetVariable() {
    VariableSE3QuaternionRightAD* offset_var=new VariableSE3QuaternionRightAD;
    offset_var->setEstimate(sim.landmark_sensor_offset);
    offset_var->setGraphId(sim.poses.size());
    offset_var->setStatus(VariableBase::Fixed); // lock the offset
    return offset_var;
  }

  VariableBase* GraphGeneratorSE3::createPoseVariable(int idx) {
    VariableSE3QuaternionRightAD* pose_var=new VariableSE3QuaternionRightAD;
    pose_var->setGraphId(idx);
    pose_var->setEstimate(sim.poses[idx]);
    return pose_var;
  }

  FactorBase* GraphGeneratorSE3::createPoseFactor(int idx_from, int idx_to) {
    const VariableSE3QuaternionRightAD* pose_from=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSE3QuaternionRightAD* pose_to=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_to));
    assert(pose_from && pose_to && "bookkeeping error, no vars in graph");
    SE3PosePoseGeodesicErrorFactor* pose_factor=new SE3PosePoseGeodesicErrorFactor();
    pose_factor->setVariableId(0, pose_from->graphId());
    pose_factor->setVariableId(1, pose_to->graphId());
    pose_factor->setMeasurement(pose_from->estimate().inverse()
                                *pose_to->estimate());
    return pose_factor;
  }

  VariableBase* GraphGeneratorSE3::createLandmarkVariable(int idx) {
    if (landmark_vars[idx]!=nullptr) {
      return landmark_vars[idx];
    }
    VariablePoint3AD* landmark_var=new VariablePoint3AD;
    landmark_var->setGraphId(landmark_idx_offset+idx);
    landmark_var->setEstimate(sim.landmarks[idx]);
    return landmark_var;
  }

  FactorBase* GraphGeneratorSE3::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {
    const VariableSE3QuaternionRightAD* pose_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSE3QuaternionRightAD* offset_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_offset));
    const VariablePoint3AD* landmark_var=dynamic_cast<const VariablePoint3AD*>(landmark_vars[idx_to]);
    SE3PosePointOffsetErrorFactor* landmark_factor=new SE3PosePointOffsetErrorFactor();
    Isometry3f sensor_pose=pose_var->estimate()*offset_var->estimate();
    landmark_factor->setMeasurement(sensor_pose.inverse()*sim.landmarks[idx_to]);
    landmark_factor->setVariableId(0, pose_var->graphId());
    landmark_factor->setVariableId(1, landmark_var->graphId());
    landmark_factor->setVariableId(2, offset_var->graphId());
    return landmark_factor;
  }

    //======================== Sim generator ========================

  VariableBase* GraphGeneratorSim3::createLandmarkSensorOffsetVariable() {
    VariableSE3QuaternionRightAD* offset_var=new VariableSE3QuaternionRightAD;
    offset_var->setEstimate(sim.landmark_sensor_offset);
    offset_var->setGraphId(sim.poses.size());
    offset_var->setStatus(VariableBase::Fixed); // lock the offset
    return offset_var;
  }

  VariableBase* GraphGeneratorSim3::createPoseVariable(int idx) {
    VariableSim3QuaternionRightAD* pose_var=new VariableSim3QuaternionRightAD;
    pose_var->setGraphId(idx);
    Similiarity_<float,3> v;
    v.linear()=sim.poses[idx].linear();
    v.translation()=sim.poses[idx].translation();
    v.inverseScaling()=1.f;
    pose_var->setEstimate(v);
    return pose_var;
  }

  FactorBase* GraphGeneratorSim3::createPoseFactor(int idx_from, int idx_to) {
    const VariableSim3QuaternionRightAD* pose_from=dynamic_cast<const VariableSim3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSim3QuaternionRightAD* pose_to=dynamic_cast<const VariableSim3QuaternionRightAD*>(graph->variable(idx_to));
    assert(pose_from && pose_to && "bookkeeping error, no vars in graph");
    Sim3PosePoseErrorFactorAD* pose_factor=new Sim3PosePoseErrorFactorAD();
    pose_factor->setVariableId(0, pose_from->graphId());
    pose_factor->setVariableId(1, pose_to->graphId());
    pose_factor->setMeasurement(pose_from->estimate().inverse()
                                *pose_to->estimate());
    return pose_factor;
  }

  VariableBase* GraphGeneratorSim3::createLandmarkVariable(int idx) {
    throw std::runtime_error("generator does not support landmarks");
    return 0;
  }

  FactorBase* GraphGeneratorSim3::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {
    throw std::runtime_error("generator does not support landmarks");
    return 0;
  }

  //=============================== SE3_BA ===============================

  FactorBase* GraphGeneratorSE3BA::createLandmarkFactor(int idx_from, int idx_to, int idx_offset) {
    const VariableSE3QuaternionRightAD* pose_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_from));
    const VariableSE3QuaternionRightAD* offset_var=dynamic_cast<const VariableSE3QuaternionRightAD*>(graph->variable(idx_offset));
    const VariablePoint3AD* landmark_var=dynamic_cast<const VariablePoint3AD*>(landmark_vars[idx_to]);
    SE3PosePointOmniBAErrorFactor* landmark_factor=new SE3PosePointOmniBAErrorFactor();
    Isometry3f sensor_pose=pose_var->estimate()*offset_var->estimate();
    Vector3f point_in_sensor=sensor_pose.inverse()*sim.landmarks[idx_to];
    float n = point_in_sensor.norm();
    if (n<1e-3)
      return 0;
    landmark_factor->setMeasurement(point_in_sensor/n);
    landmark_factor->setVariableId(0, pose_var->graphId());
    landmark_factor->setVariableId(1, landmark_var->graphId());
    landmark_factor->setVariableId(2, offset_var->graphId());
    return landmark_factor;
  }

}
