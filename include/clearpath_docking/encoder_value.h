//
//
// Copyright 2025 University of Washington

#pragma once

#include <stdint.h>

namespace clearpath_docking {

struct EncoderValue {
  EncoderValue() = default;

  EncoderValue(int64_t raw,
               double rawToMetric = 1.0)  // NOLINT [runtime/explicit]
      : _raw(raw), raw_to_metric_(rawToMetric), _offset(0) {}

  // Copy, adding offset
  EncoderValue(const EncoderValue &other,
               int64_t offset = 0)  // NOLINT [runtime/explicit]
      : _raw(other._raw),
        raw_to_metric_(other.raw_to_metric_),
        _offset(offset) {}

  void setOffsetRaw(int64_t offset) { _offset = offset; }
  int64_t offsetRaw(void) const { return _offset; }

  int64_t raw(void) const { return _raw - _offset; }
  int64_t trueRaw(void) const { return _raw; }

  double metric(void) const { return raw() * raw_to_metric_; }

  double rawToMetric(void) const { return raw_to_metric_; }

  int64_t _raw, _offset;
  double raw_to_metric_;
};

}  
