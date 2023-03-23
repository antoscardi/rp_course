#include "multi_graph_slam_messages.h"
namespace srrg2_slam_interfaces {
  using namespace srrg2_core;

  void LocalMapMessageBase::serialize(ObjectData& odata, IdContext& context) {
    BaseSensorMessage::serialize(odata, context);
    srrg2_core::ArrayData* adata = new srrg2_core::ArrayData;
    for (auto it : history) {
      if (!it) {
        continue;
      }
      srrg2_core::ObjectData* hdata = new srrg2_core::ObjectData;
      it->serialize(*hdata, context);
      adata->push_back(hdata);
    }
    odata.setField("history", adata);
  }

  void LocalMapMessageBase::deserialize(ObjectData& odata, IdContext& context) {
    BaseSensorMessage::deserialize(odata, context);
    srrg2_core::ValueData * vdata_ptr=odata.getField("history");
    srrg2_core::ArrayData* adata_ptr=dynamic_cast<srrg2_core::ArrayData*>(vdata_ptr);
    if (! adata_ptr)
      return;
    
    srrg2_core::ArrayData& adata = *adata_ptr;
    history.clear();
    for (size_t i = 0; i < adata.size(); ++i) {
      srrg2_core::ValueData& vdata  = adata[i];
      srrg2_core::ObjectData& odata = dynamic_cast<srrg2_core::ObjectData&>(vdata);
      TrackerReportRecordPtr report(new TrackerReportRecord);
      report->deserialize(odata, context);
      history.push_back(report);
    }
  }

} // namespace srrg2_slam_interfaces
