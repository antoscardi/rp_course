#pragma once
#include "orazio_globals.h"

//initializes global joint vars
void Orazio_jointsPreInit(void);

// initializes the joint subsystems based on the configuration
// also initializes the h bridges and the pins
void Orazio_jointsInit(void);

// handles one update of the joints
void Orazio_jointsHandle(void);

// sets to 0 all joint controls
void Orazio_jointsDisable(void);
