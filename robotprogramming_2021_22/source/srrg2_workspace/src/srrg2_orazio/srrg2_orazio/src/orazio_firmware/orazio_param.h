#pragma once
#include "orazio_globals.h"


// resets the eeprom with factory values. Need to reboot arduino
void Orazio_paramHardReset(void);

// initializes the parameter subsystem
// if the protocol version changed, the old parameters
// in the eeprom are refreshed with new values
// otherwise the old parameters are retained
// the deafault operation is to load the parameters in the eeprom
void Orazio_paramInit(void);

// loads the parameter of a sybsystem from eeprom
PacketStatus Orazio_paramLoad(uint8_t param_type, int8_t index);

// saves the parameter of a sybsystem in eeprom
PacketStatus Orazio_paramSave(uint8_t param_type, int8_t index);

// call this to set restart a subsystem whose parameters are chenged
PacketStatus Orazio_paramSet(uint8_t param_type, int8_t index);
