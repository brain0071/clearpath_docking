//
// Copyright 2025 University of Washington

#pragma once

#include "clearpath_docking/encoder_value.h"

namespace clearpath_docking {

struct MotorState {
  EncoderValue position, velocity, current;
};

}  // namespace testbed_driver
