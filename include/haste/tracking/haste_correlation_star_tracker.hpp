// Copyright (c) 2021 by Ignacio Alzugaray <alzugaray dot ign at gmail dot com>
// ETH Zurich, Vision for Robotics Lab.
#pragma once

#include "haste/tracking/hypothesis_tracker.hpp"
namespace haste {
class HasteCorrelationStarTracker : public HypothesisPatchTracker {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline HasteCorrelationStarTracker(const Time &t, const Location &x, const Location &y, const Orientation &theta);
  inline auto trackerName() const -> std::string override;
  inline auto updateTemplate() -> void override;
  inline auto eventWindowToModel(const EventWindow &event_window, const Hypothesis &hypothesis) const -> Patch override;
  inline auto initializeHypotheses() -> void override;
  inline auto updateHypothesesScore(const EventTuple &oldest_event, const EventTuple &newest_event) -> void override;

 protected:
  static constexpr Weight kWeight_ = 1.0 / kEventWindowSize;
  Patch template_normalized_;
};
}// namespace haste

#include "haste/tracking/haste_correlation_star_tracker_impl.hpp"