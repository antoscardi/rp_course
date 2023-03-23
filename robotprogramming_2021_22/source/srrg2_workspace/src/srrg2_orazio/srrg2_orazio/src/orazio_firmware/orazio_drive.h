#pragma once

// initiaizes the drive controller
void Orazio_driveInit(void);

// updates odometry in the status packet
// flushes to the joints velocities computed to move the base
// with translational and rotational velocity
void Orazio_driveHandle(void);


