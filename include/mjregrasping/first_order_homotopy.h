#include <memory>

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/first_order_deformation.h>
#include <functional>

using namespace EigenHelpers;
using namespace arc_utilities::FirstOrderDeformation;

std::vector<ConfigType> getFirstOrderHomotopyPoints(const std::function<bool(Eigen::Vector3d)> &is_collision, const VectorVector3d& b1, const VectorVector3d& b2);
