#include <memory>

#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/first_order_deformation.h>
#include <sdf_tools/sdf_generation.hpp>

using namespace EigenHelpers;
using namespace arc_utilities::FirstOrderDeformation;

std::vector<ConfigType> getFirstOrderHomotopyPoints(sdf_tools::SignedDistanceField const &sdf, const VectorVector3d& b1, const VectorVector3d& b2, const double sdf_threshold);
