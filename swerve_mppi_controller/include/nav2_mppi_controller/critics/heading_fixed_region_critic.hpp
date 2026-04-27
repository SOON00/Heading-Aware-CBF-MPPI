#ifndef NAV2_MPPI_CONTROLLER__CRITICS__HEADING_FIXED_REGION_CRITIC_HPP_
#define NAV2_MPPI_CONTROLLER__CRITICS__HEADING_FIXED_REGION_CRITIC_HPP_

#include <string>
#include <vector>
#include <cmath>
#include <regex>
#include <stdexcept>
#include <sstream>

#include <pluginlib/class_list_macros.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xview.hpp>

#include "nav2_mppi_controller/critic_function.hpp"
#include "nav2_mppi_controller/tools/utils.hpp"


namespace mppi::critics
{

class HeadingFixedRegionCritic : public CriticFunction
{
public:
  void initialize() override;
  void score(CriticData & data) override;

protected:
  struct Point2D
  {
    double x;
    double y;
  };

  struct RegionConfig
  {
    std::string name;
    std::vector<Point2D> polygon;
    double locked_heading {0.0};
    double cost_weight {0.0};
  };

  bool pointInPolygon(double x, double y, const std::vector<Point2D> & polygon) const;
  bool findActiveRegion(
    double x, double y,
    RegionConfig & active_region) const;

  std::vector<Point2D> parsePolygonString(const std::string & polygon_str) const;

protected:
  int power_ {1};
  double default_cost_weight_ {0.0};

  std::vector<std::string> region_names_;
  std::vector<std::string> region_polygons_str_;
  std::vector<double> region_locked_headings_;
  std::vector<double> region_cost_weights_;

  std::vector<RegionConfig> regions_;

  };
}  // namespace mppi::critics

#endif  // NAV2_MPPI_CONTROLLER__CRITICS__HEADING_FIXED_REGION_CRITIC_HPP_