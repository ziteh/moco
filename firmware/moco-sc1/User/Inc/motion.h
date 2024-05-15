
#ifndef MOTION_H_
#define MOTION_H_

#include "algorithm.h"
#include "parameters.h"
#include "stspin32g4.h"

void open_loop_velocity(float target_vel, float voltage, float vm, modulation_fn_t mod_fn);

#endif /* MOTION_H_ */
