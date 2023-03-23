#include "grid_mapper_2d.h"
#include <srrg_data_structures/grid_map_2d.h>
#include <srrg_geometry/geometry3d.h>
#include <srrg_geometry/geometry2d.h>
#include <srrg_data_structures/traverse_line.h>
#include <srrg_solver/variables_and_factors/types_2d/se2_pose_pose_geodesic_error_factor.h>
#include <srrg_config/configurable_command.h>
#include <unistd.h>

namespace srrg2_laser_slam_2d {
  using namespace srrg2_core;
  using namespace srrg2_slam_interfaces;
  using namespace srrg2_solver;
  using namespace std;


  GridMapper2D::GridMapper2D():
    _graph(new FactorGraph){
    addCommand (new ConfigurableCommand_
                < GridMapper2D, typeof(&GridMapper2D::cmdSaveGraph), std::string, std::string>
                (this,
                 "saveGraph",
                 "saves a graph to a json file",
                 &GridMapper2D::cmdSaveGraph));
    addCommand (new ConfigurableCommand_
                < GridMapper2D, typeof(&GridMapper2D::cmdSaveMap), std::string, std::string>
                (this,
                 "saveMap",
                 "renders a map and saves it to an image. PRosuces the yaml and in files for ros and stage",
                 &GridMapper2D::cmdSaveMap));
    _global_grid_map.reset();
    reset();
  }

  void GridMapper2D::reset()  {
    _graph->clear();
    _pending_maps.clear();
    _pending_laser_messages.clear();
    _lower_left=Eigen::Vector2f(std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max());
    _upper_right=Eigen::Vector2f(-std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max());
    _global_grid_map.reset();
    MessageSinkBase::reset();
  }
  
  GridMapper2D::~GridMapper2D() {}

  bool GridMapper2D::handleLocalMapMessage(BaseSensorMessagePtr msg_) {
    LocalMapMessage2DPtr lmap_msg=std::dynamic_pointer_cast<LocalMapMessage2D>(msg_);
    if (! lmap_msg) {
      return false;
    }
    //cerr << "GridMapper2D|handleLocalMapMessage" << endl;
    int graph_id=lmap_msg->id.value();
    Eigen::Isometry2f estimate=lmap_msg->estimate.value();
    VariableBase* v=_graph->variable(graph_id);
    LocalMap2D* lmap=nullptr;
    GridMap2DPtr grid_map=nullptr;
    if (! v) {
      lmap=new LocalMap2D();
      lmap->setGraphId(graph_id);
      lmap->setEstimate(estimate);
      _graph->addVariable(LocalMap2DPtr(lmap));
      GridMap2DPtr grid_map(new GridMap2D);
      Eigen::Vector2f size(param_local_map_size.value(), param_local_map_size.value());
      new PropertyGridMap("grid_map_2d",
                          "",
                          &(lmap->dynamic_properties),
                          grid_map);
      PropertyFrequencyGridType* fgrid = new PropertyFrequencyGridType("frequency",
                                                                       "",
                                                                       grid_map.get(),
                                                                       FrequencyGridType());
      grid_map->setResolution(param_resolution.value());
      grid_map->setSize(size);
      grid_map->setOrigin(estimate);
      fgrid->value().fill(Eigen::Vector2f::Zero());
      new PropertyHistory("motion_in_local_map","",&(lmap->dynamic_properties), HistoryType());
    } else {
      lmap = dynamic_cast<LocalMap2D*>(v);
      PropertyGridMap* grid_map_prop=lmap->dynamic_properties.property<PropertyGridMap>("grid_map_2d");     grid_map=grid_map_prop->value();
      PropertyFrequencyGridType* fgrid = grid_map->property<PropertyFrequencyGridType>("frequency");
      if (! fgrid){
        throw std::runtime_error("cast error, the proprerty is not a frequency grid");
      }
    }
    PropertyHistory* history=lmap->dynamic_properties.property<PropertyHistory>("motion_in_local_map");   assert(history);
    for (auto it: lmap_msg->history) {
      history->value().push_back(it);
    }
    _pending_maps.insert(std::make_pair(lmap_msg->timestamp.value(), lmap));
    return true;
  }

  bool GridMapper2D::handleFactorMessage(BaseSensorMessagePtr msg_) {
    FactorMessageSE2Ptr factor_msg=std::dynamic_pointer_cast<FactorMessageSE2>(msg_);
    if (! factor_msg)
      return false;
    //cerr << "GridMapper2D|handleFactorMessage" << endl;
    
    const VariableBase* v_from = _graph->variable(factor_msg->from_id.value());
    const VariableBase* v_to   = _graph->variable(factor_msg->to_id.value());
    if (! v_from || ! v_to)
      return false;
    std::shared_ptr<SE2PosePoseGeodesicErrorFactor> factor(new SE2PosePoseGeodesicErrorFactor);
    factor->setGraphId(factor_msg->id.value());
    factor->setVariableId(0, factor_msg->from_id.value());
    factor->setVariableId(1, factor_msg->to_id.value());
    _graph->addFactor(factor);
    return true;
  }

  bool GridMapper2D::handleNodeUpdateMessage(BaseSensorMessagePtr msg_) {
    using PropertyGridMap=Property_<GridMap2DPtr>;
    NodeUpdateMessage2DPtr node_update_msg= std::dynamic_pointer_cast<NodeUpdateMessage2D>(msg_);
    if (! node_update_msg)
      return false;
    //cerr << "GridMapper2D|handleNodeUpdateMessage" << endl;
    _lower_left=Eigen::Vector2f(std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max());
    _upper_right=Eigen::Vector2f(-std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max());
    for (auto& it: node_update_msg->updates) {
      int id=it.first;
      Eigen::Isometry2f pose=it.second;
      VariableBase* v = _graph->variable(id);
      if (!v)
        continue;
      LocalMap2D* lmap= dynamic_cast<LocalMap2D*>(v);
      if (!lmap)
        continue;
      lmap->setEstimate(pose);
      PropertyGridMap* grid_map_prop=lmap->dynamic_properties.property<PropertyGridMap>("grid_map_2d");     GridMap2DPtr grid_map=grid_map_prop->value();
      grid_map->setOrigin(pose);
    }
    return true;
  }

  bool GridMapper2D::handleLaserMessage(BaseSensorMessagePtr  msg_) {
    LaserMessagePtr scan_msg= std::dynamic_pointer_cast<LaserMessage>(msg_);
    if (! scan_msg)
      return false;
    Eigen::Isometry2f laser_on_robot;
    laser_on_robot.setIdentity();
    if (platform()) {
      if (!platform()->getTransform(laser_on_robot,
                                    scan_msg->frame_id.value(),
                                    "/base_link",
                                    scan_msg->timestamp.value())){
        return false;
      }
    }
    LaserMessageWithTfPtr m(new LaserMessageWithTf(scan_msg, laser_on_robot));
    //cerr << "GridMapper2D|handleLaserMessage" << endl;
    _pending_laser_messages.insert(std::make_pair(scan_msg->timestamp.value(), m));
    return true;
  }

  bool GridMapper2D::putMessage(BaseSensorMessagePtr msg_) {
    bool process_pending=false;
    double current_timestamp=msg_->timestamp.value();
    process_pending |= handleLocalMapMessage(msg_);
    process_pending |= handleFactorMessage(msg_);
    process_pending |= handleNodeUpdateMessage(msg_);
    process_pending |= handleLaserMessage(msg_);
    if (process_pending) {
      processPendingMessages(current_timestamp);
    }
    return true;
  }

  void GridMapper2D::processPendingMessages(double current_timestamp) {
    double delete_time=current_timestamp-param_time_horizon.value();
    // destroy all laser messages prior current_timestamp - interval
    while(!_pending_laser_messages.empty() && _pending_laser_messages.begin()->first< delete_time)
      _pending_laser_messages.erase(_pending_laser_messages.begin());
    // destroy all map updates prior current timestamp - interval
    while(!_pending_maps.empty() && _pending_maps.begin()->first < delete_time )
      _pending_maps.erase(_pending_maps.begin());
    // for each local map, scan the history and seek in the map a timestamp that is the one
    // in the history. If found, integrate the scan in the frequency map

    if (_pending_maps.empty())
      return;
   
    //cerr << "GridMapper2D|processPendingMessages (" << _pending_maps.size() << ")" << endl;
    // process the first map update, if present
    for (auto it=_pending_maps.begin(); it!=_pending_maps.end(); ++it) {
      bool erase = handlePendingLocalMapUpdate(*(it->second),
                                               it->first);
      if (erase) {
        auto erased=it;
        ++it;
        _pending_maps.erase(erased);
        if (_pending_maps.empty())
          break;
      }
    }
  }

  GridMapper2D::LaserMessageWithTfPtr GridMapper2D::findScanInPending(const std::string& topic,
                                                        int seq,
                                                        double timestamp) {
    LaserMessageWithTfPtr returned=nullptr;
    for(auto it: _pending_laser_messages) {
      if (it.second->message->topic.value()!=topic)
        continue;
      double dts=abs(it.second->message->timestamp.value()-timestamp);
      if (dts>param_time_horizon.value()/2)
        break;
      if (it.second->message->seq.value()!=seq)
        continue;
      else
        return it.second;
    }
    return returned;
  }

  bool GridMapper2D::handlePendingLocalMapUpdate(LocalMap2D& lmap, double timestamp) {
    PropertyGridMap* grid_map_prop=lmap.dynamic_properties.property<PropertyGridMap>("grid_map_2d");     assert(grid_map_prop);

    GridMap2DPtr grid_map=grid_map_prop->value();
    
    PropertyFrequencyGridType* fgrid_prop = grid_map->property<PropertyFrequencyGridType>("frequency");
    assert(fgrid_prop);
    FrequencyGridType& fgrid = fgrid_prop->value();

    PropertyHistory* history_prop=lmap.dynamic_properties.property<PropertyHistory>("motion_in_local_map");
    assert(history_prop);
    HistoryType& history(history_prop->value());
    std::list<HistoryType::iterator> erased_updates;
    for (auto hit = history.begin(); hit!=history.end(); ++hit) {
      TrackerReportRecord& report=*(*hit);
      const Eigen::Isometry2f& pose_in_local_map=geometry3d::get2dFrom3dPose(report.pose_in_local_map.value());
      for (auto sit =  report.measurements.begin(); sit!=report.measurements.end(); ++sit){
        TrackerInputHandlePtr handle=*sit;
        LaserMessageWithTfPtr scan=findScanInPending(handle->topic,  handle->seq, handle->timestamp);
        if(!scan)
          continue;
        else {
          auto erased=sit;
          ++sit;
          report.measurements.erase(erased);
          integrateScan(*grid_map, fgrid, pose_in_local_map*scan->laser_in_robot, scan->message);
        }
      }
      if (report.measurements.empty()) {
        auto erased=hit;
        ++hit;
        history.erase(erased);
      }
    }
    return history.empty();
  }
  

  struct UpdateFrequencyAction{
    UpdateFrequencyAction(FrequencyGridType& fmap_):
      fmap(fmap_){}
    inline bool operator()(int x , int y) {
      if (!fmap.inside(x,y))
        return false;
      Eigen::Vector2f& p=fmap.at(x,y);
      p(1)+=1;
      return true;
    };
    FrequencyGridType& fmap;
  };


  void GridMapper2D::integrateScan(GridMap2D& grid_map,
                                   FrequencyGridType& fgrid,
                                   const Eigen::Isometry2f& pose_in_local_map,
                                   LaserMessagePtr scan) {
    cerr << "integrating scan, topic: [" << scan->topic.value() << "] "
         << "pose: [" << geometry2d::t2v(pose_in_local_map).transpose() << "] "
         << "seq:  [" << scan->seq.value() << "]" << endl;
    Eigen::Isometry2f laser_pose_in_robot=Eigen::Isometry2f::Identity();
    Eigen::Isometry2f laser_pose_in_local_map=pose_in_local_map*laser_pose_in_robot;
    Eigen::Vector2f beam_origin=laser_pose_in_local_map.translation();
    Eigen::Vector2i index_origin=grid_map.local2indices(beam_origin);
    if (! fgrid.inside(index_origin.x(), index_origin.y()))
      return;
    UpdateFrequencyAction action(fgrid);
    int int_ep_radius = param_endpoint_radius.value()/grid_map.resolution();
    float ep_gain=param_endpoint_gain.value();
    for (size_t i=0; i<scan->ranges.size(); ++i) {
      float r=scan->ranges.value(i);
      if (r<scan->range_min.value())
        continue;
      if (r>scan->range_max.value())
        continue;
      bool draw_endpoint=true;
      if (r>param_usable_range.value()) {
        if (param_max_range_invalid.value())
          continue;
        draw_endpoint=false;
        r=param_usable_range.value();
      }
      float alpha= scan->angle_min.value()+scan->angle_increment.value()*i;
      float c=cos(alpha);
      float s=sin(alpha);
      Eigen::Vector2f local_endpoint(r*c,r*s);
      Eigen::Vector2f beam_endpoint=laser_pose_in_local_map*local_endpoint;
      Eigen::Vector2i index_endpoint=grid_map.local2indices(beam_endpoint);
      //std::cerr << "\t ep: " << i << " " << beam_endpoint.transpose() << " " << index_endpoint.transpose() << std::endl;
      //updateBBox(grid_map->local2global(beam_endpoint));
      
      draw_endpoint &= traverseLine(index_origin.x(),
                                    index_origin.y(),
                                    index_endpoint.x(),
                                    index_endpoint.y(), action);
      if (draw_endpoint) {
        for (int r=-int_ep_radius; r<=int_ep_radius; ++r) {
          for (int c=-int_ep_radius; c<=int_ep_radius; ++c) {
            int rr=r+index_endpoint.x();
            int cc=c+index_endpoint.y();
            if (fgrid.inside(rr,cc)){
              fgrid.at(rr,cc)(0)+=ep_gain;
              fgrid.at(rr,cc)(1)+=ep_gain;
            }
          }
        }
      }
    }    
  }

  void GridMapper2D::updateBoundingBox(GridMap2DPtr grid_map) {
    Eigen::Matrix<float, 2,4>  corners;
    grid_map->cornersInGlobal(corners);
    Eigen::Matrix2f R=_global_grid_map->origin().linear().transpose();
    for (int i=0; i<4; ++i) {
      Eigen::Vector2f corner=R*corners.col(i);
      _lower_left.x()=std::min(corner.x(), _lower_left.x());
      _lower_left.y()=std::min(corner.y(), _lower_left.y());
      _upper_right.x()=std::max(corner.x(), _upper_right.x());
      _upper_right.y()=std::max(corner.y(), _upper_right.y());
    }
  }


  struct FillMultiMapAction{
    FillMultiMapAction(std::multimap<int, int>& line_):
      line(line_){}
    inline bool operator()(int x , int y) {
      line.insert(std::make_pair(x,y));
      return true;
    };
    std::multimap<int, int>& line;
  };

  void GridMapper2D::pasteLocalMap(GridMap2DPtr grid_map) {
    using PropertyFrequencyGridType=Property_<FrequencyGridType>;
    PropertyFrequencyGridType* fgrid_prop=grid_map->property<PropertyFrequencyGridType>("frequency");
    if (! fgrid_prop){
      throw std::runtime_error("cast error, the proprerty is not a frequency grid");
    }
    
    PropertyFrequencyGridType* dgrid_prop=_global_grid_map->property<PropertyFrequencyGridType>("frequency");
    if (! dgrid_prop){
      throw std::runtime_error("cast error, the proprerty is not a frequency grid");
    }

    
    FrequencyGridType& fmap=fgrid_prop->value();
    FrequencyGridType& dmap=dgrid_prop->value();
    Eigen::Matrix<float, 2,4>  corners;
    grid_map->cornersInGlobal(corners);
    std::multimap<int, int> contour;
    FillMultiMapAction action(contour);
    for (int i=0; i<4; ++i) {
      Eigen::Vector2f p1=corners.col(i%4);
      Eigen::Vector2f p2=corners.col((i+1)%4);
      Eigen::Vector2i ip1=_global_grid_map->global2indices(p1);
      Eigen::Vector2i ip2=_global_grid_map->global2indices(p2);
      traverseLine(ip1.x(), ip1.y(), ip2.x(), ip2.y(), action);
    }
    int current_row=-1;
    int min_col=std::numeric_limits<int>::max();
    int max_col=-std::numeric_limits<int>::max();
    for (auto it=contour.begin(); it!=contour.end(); ++it) {
      int r=it->first;
      int c=it->second;
      if (r>current_row) {
        for (int cc=min_col; cc<=max_col; ++cc) {
          if (! dmap.inside(r,cc) )
            continue;
          Vector2f global_point=_global_grid_map->indices2global(Eigen::Vector2i(r,cc));
          Eigen::Vector2i src_idx = grid_map->global2indices(global_point);
          if (! fmap.inside(src_idx))
            continue;
          dmap.at(r,cc)+=fmap.at(src_idx);
        }
        min_col=std::numeric_limits<int>::max();
        max_col=-std::numeric_limits<int>::max();
      } 
      current_row=r;
      min_col=std::min(c, min_col);
      max_col=std::max(c, max_col);
    }
  }


  void GridMapper2D::saveImage(const std::string& filename) {
    if (! _global_grid_map) {
      std::cerr << "no global map computed, cannot save image" << std::endl;
    }
    using PropertyFrequencyGridType=Property_<FrequencyGridType>;
    PropertyFrequencyGridType* fprop=_global_grid_map->property<PropertyFrequencyGridType>("frequency");
    FrequencyGridType& fmap=fprop->value();
    Vector2i size=_global_grid_map->GridMap2DHeader::size();
    size_t rows=size(0);
    size_t cols=size(1);
    ImageVector4uc image(rows, cols);
    for (size_t r=0; r<rows; ++r)
      for (size_t c=0; c<cols; ++c) {
        const Vector2f& f=fmap.at(r,c);
        Eigen::Matrix<unsigned char,4,1>  d;
        d << 255, 255, 255, 0;
        if (f(1)!=0) {
          unsigned char occ=255-((f(0)/f(1)))*255;
          d << occ, occ, occ, 255;
        }
        image.at(r,c)=d;
      }
    cv::Mat dest;
    image.toCv(dest);
    cv::imwrite(filename.c_str(),dest);
  }

  bool GridMapper2D::cmdSaveGraph(std::string& response, const std::string& filename) {
    response = className()
      + "| saving graph to file ["
      + filename
      + "]";
    if (! _graph) {
      response += " No Graph, Aborting";
      return false;
    }
    _graph->write(filename);
    return true;
  }

  void GridMapper2D::updateBoundingBox() {
    _lower_left=Eigen::Vector2f(std::numeric_limits<float>::max(),
                                std::numeric_limits<float>::max());
    _upper_right=Eigen::Vector2f(-std::numeric_limits<float>::max(),
                                 -std::numeric_limits<float>::max());
    for (auto it: _graph->variables()) {
      srrg2_solver::VariableBase* v=it.second;
      srrg2_slam_interfaces::LocalMap2D* lmap=dynamic_cast<srrg2_slam_interfaces::LocalMap2D*>(v);
      PropertyGridMap* grid_map=lmap->dynamic_properties.property<PropertyGridMap>("grid_map_2d");
      if (! grid_map)
        continue;
      if (! grid_map->value())
        continue;
      updateBoundingBox(grid_map->value());
    }

  }

  bool GridMapper2D::cmdSaveMap(std::string& response, const std::string& file_prefix){
    ostringstream response_stream;

    response_stream << "creating grid_map" << std::endl;
    if (! _global_grid_map) {
      
      _global_grid_map=GridMap2DPtr(new GridMap2D);
    }

    PropertyFrequencyGridType* fgrid = _global_grid_map->property<PropertyFrequencyGridType>("frequency");
    if (! fgrid){
        fgrid=new PropertyFrequencyGridType("frequency",
                                            "",
                                            _global_grid_map.get(),
                                            FrequencyGridType());
    }
    
    _global_grid_map->setResolution(param_resolution.value());
    Isometry2f global_origin=geometry2d::v2t(Vector3f(0,0,param_global_map_orientation.value()));
    _global_grid_map->setOrigin(global_origin);
    response_stream << "recomputing bounding box" << std::endl;
    updateBoundingBox();
    
    response_stream << "Rendering, bbox: ["<< _lower_left.transpose() << "] x [" << _upper_right.transpose() << "]" << std::endl;
    Eigen::Vector2f metric_size=_upper_right-_lower_left;
    response_stream << "Rendering: metric size:  ["<< metric_size.transpose() << "]" << std::endl;
    _global_center=(_upper_right+_lower_left)*0.5;
    
    _global_grid_map->setSize(metric_size);
    fgrid->value().fill(Vector2f(0,0));
    global_origin.translation()=_global_center;
    _global_grid_map->setOrigin(global_origin);
    response_stream << "Rendering: cell size:  ["<< _global_grid_map->GridMap2DHeader::size().transpose() << "]" << std::endl;

    for (auto it: _graph->variables()) {
      srrg2_solver::VariableBase* v=it.second;
      srrg2_slam_interfaces::LocalMap2D* lmap=dynamic_cast<srrg2_slam_interfaces::LocalMap2D*>(v);
      if (!lmap) {
        continue;
      }
      int local_map_id=lmap->graphId();
      PropertyGridMap* grid_map=lmap->dynamic_properties.property<PropertyGridMap>("grid_map_2d");
      if (! grid_map)
        continue;
      response_stream << "processing local map id: " << local_map_id << std::endl;
      pasteLocalMap(grid_map->value());
    }
    std::string image_filename = file_prefix + std::string(".png");
    saveImage(image_filename);

    std::string yaml_filename  = file_prefix + std::string(".yaml");
    ofstream os_yaml (yaml_filename);
    if (! os_yaml) {
      response_stream << "cannot save file" << std:: endl;
      response = response_stream.str();
      return false;
    }
    os_yaml << "image:           " << image_filename << std::endl;
    os_yaml << "resolution:      " << _global_grid_map->resolution() << std::endl;
    os_yaml << "origin:          [ 0.0,  0.0, 0.0 ]" << std::endl;
    os_yaml << "occupied_thresh: 0.85" << std::endl;
    os_yaml << "free_thresh:     0.1"  << std::endl;
    os_yaml << "negate:          0"    << std::endl;
    os_yaml << "mode:            scale"    << std::endl;

    std::string inc_filename  = file_prefix + std::string(".inc");
    ofstream os_inc (inc_filename);
    if (! os_inc) {
      response_stream << "cannot save file" << std:: endl;
      return false;
    }
    os_inc << "floorplan" << std::endl;
    os_inc << "(" << std::endl;
    os_inc << "   bitmap \"" << image_filename << "\"" << std::endl;
    os_inc << "   size   [" << metric_size.y() << " " << metric_size.x() << " 0.5]" << std::endl;
    os_inc << "   pose   [0 0 0 0]" << std::endl;
    os_inc << ")" << std::endl;
    response = response_stream.str();
    return true;
  }
}
