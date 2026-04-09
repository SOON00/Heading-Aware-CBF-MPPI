#include "nav2_mppi_controller/critics/heading_fixed_region_critic.hpp"

#include <cmath>
#include <regex>
#include <stdexcept>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

namespace mppi::critics
{

void HeadingFixedRegionCritic::initialize()
{
  auto getParam = parameters_handler_->getParamGetter(name_);

  getParam(enabled_, "enabled", true);
  getParam(power_, "cost_power", 1);
  getParam(default_cost_weight_, "default_cost_weight", 0.0);

  getParam(region_names_, "region_names", std::vector<std::string>{});
  getParam(region_polygons_str_, "region_polygons", std::vector<std::string>{});
  getParam(region_locked_headings_, "region_locked_headings", std::vector<double>{});
  getParam(region_cost_weights_, "region_cost_weights", std::vector<double>{});

  if (
    region_names_.size() != region_polygons_str_.size() ||
    region_names_.size() != region_locked_headings_.size() ||
    region_names_.size() != region_cost_weights_.size())
  {
    throw std::runtime_error(
            "[" + name_ + "] region_names / region_polygons / region_locked_headings / "
            "region_cost_weights size mismatch");
  }

  regions_.clear();
  regions_.reserve(region_names_.size());

  for (size_t i = 0; i < region_names_.size(); ++i) {
    RegionConfig region;
    region.name = region_names_[i];
    region.polygon = parsePolygonString(region_polygons_str_[i]);
    region.locked_heading = region_locked_headings_[i];
    region.cost_weight = region_cost_weights_[i];

    if (region.polygon.size() < 3) {
      throw std::runtime_error(
              "[" + name_ + "] region '" + region.name + "' polygon must have at least 3 points");
    }

    regions_.push_back(region);

    RCLCPP_INFO(
      logger_,
      "[%s] region[%zu] name=%s, vertices=%zu, locked_heading=%.3f, cost_weight=%.3f",
      name_.c_str(),
      i,
      region.name.c_str(),
      region.polygon.size(),
      region.locked_heading,
      region.cost_weight);
  }

  RCLCPP_INFO(
    logger_,
    "[%s] instantiated with cost_power=%d, default_cost_weight=%.3f, num_regions=%zu",
    name_.c_str(), power_, default_cost_weight_, regions_.size());
}

std::vector<HeadingFixedRegionCritic::Point2D>
HeadingFixedRegionCritic::parsePolygonString(const std::string & polygon_str) const
{
  std::vector<Point2D> polygon;

  static const std::regex number_regex(R"([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)");
  std::sregex_iterator begin(polygon_str.begin(), polygon_str.end(), number_regex);
  std::sregex_iterator end;

  std::vector<double> values;
  for (auto it = begin; it != end; ++it) {
    values.push_back(std::stod(it->str()));
  }

  if (values.size() % 2 != 0) {
    throw std::runtime_error(
            "[" + name_ + "] invalid polygon string, odd number of coordinates: " + polygon_str);
  }

  for (size_t i = 0; i < values.size(); i += 2) {
    polygon.push_back(Point2D{values[i], values[i + 1]});
  }

  return polygon;
}

bool HeadingFixedRegionCritic::pointInPolygon(
  double x, double y,
  const std::vector<Point2D> & polygon) const
{
  bool inside = false;
  const size_t n = polygon.size();

  if (n < 3) {
    return false;
  }

  for (size_t i = 0, j = n - 1; i < n; j = i++) {
    const auto & pi = polygon[i];
    const auto & pj = polygon[j];

    const bool intersect =
      ((pi.y > y) != (pj.y > y)) &&
      (x < (pj.x - pi.x) * (y - pi.y) / ((pj.y - pi.y) + 1e-12) + pi.x);

    if (intersect) {
      inside = !inside;
    }
  }

  return inside;
}

bool HeadingFixedRegionCritic::findActiveRegion(
  double x, double y,
  RegionConfig & active_region) const
{
  // YAML에 적은 순서대로 첫 번째 매칭 영역 사용
  for (const auto & region : regions_) {
    if (pointInPolygon(x, y, region.polygon)) {
      active_region = region;
      return true;
    }
  }
  return false;
}

void HeadingFixedRegionCritic::score(CriticData & data)
{
  using xt::evaluation_strategy::immediate;

  if (!enabled_) {
    return;
  }

  if (regions_.empty()) {
    return;
  }

  const double robot_x = data.state.pose.pose.position.x;
  const double robot_y = data.state.pose.pose.position.y;

  RegionConfig active_region;
  const bool is_in_region = findActiveRegion(robot_x, robot_y, active_region);

  if (!is_in_region) {
    // 영역 밖이면 cost_weight = 0.0 과 동일하게 동작
    return;
  }

  if (active_region.cost_weight <= 0.0) {
    return;
  }

  auto angle_errors = xt::abs(
    utils::shortest_angular_distance(
      data.trajectories.yaws,
      active_region.locked_heading));

  data.costs += xt::pow(
    xt::mean(angle_errors, {1}, immediate) * active_region.cost_weight,
    power_);

  RCLCPP_DEBUG(
    logger_,
    "[%s] active_region=%s, robot=(%.3f, %.3f), locked_heading=%.3f, cost_weight=%.3f",
    name_.c_str(),
    active_region.name.c_str(),
    robot_x,
    robot_y,
    active_region.locked_heading,
    active_region.cost_weight);
}

}  // namespace mppi::critics

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::HeadingFixedRegionCritic,
  mppi::critics::CriticFunction)