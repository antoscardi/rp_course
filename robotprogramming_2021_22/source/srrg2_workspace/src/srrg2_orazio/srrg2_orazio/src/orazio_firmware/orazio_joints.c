#include <string.h>
#include "orazio_globals.h"
#include "encoder.h"
#include "orazio_joints_internal.h"
#include "orazio_hbridge_internal.h"
#include "adc.h"

static HBridge bridges[NUM_JOINTS];

void Orazio_jointsPreInit(void) {
  for (int i=0; i<NUM_JOINTS; ++i) {
    bridges[i].enabled=0;
    bridges[i].ops=0;
  }
}

void Orazio_jointsInit(void){
  for (int i=0; i<NUM_JOINTS; ++i){
    memset(joint_controllers+i, 0, sizeof(JointController));
    JointController* c=joint_controllers+i;
    JointParams* params=&joint_params[i].param;
    JointInfo* status=&joint_status[i].info;
    status->enabled=1;
    JointControl* control=&joint_control[i].control;
    JointController_init(c, params, status, control);

    HBridge* bridge=bridges+i;
    HBridgeType type=params->h_bridge_type;
    if (bridge->ops && bridge->ops->fini_fn) 
      (*bridge->ops->fini_fn)(bridge);

    HBridge_init(bridge, type, params->h_bridge_pins);
    status->enabled=bridge->enabled;
  }
}

const static int _adc_channel_route[]= {
  3,
  2,
  0,
  1
};

void Orazio_jointsHandle(void){
  // sample encoder values
  Encoder_sample();
  // compute desired speed based on the encoder input
  // and the control strategy
  for (uint8_t i=0; i<NUM_JOINTS; ++i){
    if (joint_params[i].header.system_enabled)
      JointController_handle(joint_controllers+i, i);
  }
  // apply the control to each joint
  for (uint8_t i=0; i<NUM_JOINTS; ++i){
    if (joint_params[i].header.system_enabled){
      HBridge* bridge=bridges+i;
      JointController* controller=joint_controllers+i;

      if (controller->status->enabled)
        HBridge_setSpeed(bridge,controller->output);
      controller->status->sensed_current=ADC_getValue(i);
    }
  }
}

void Orazio_jointsDisable(void){
  for (uint8_t i=0; i<NUM_JOINTS; ++i){
    joint_controllers[i].control->mode=JointDisabled;
    joint_controllers[i].output=0;
  }
}
