#include <arc_utilities/eigen_helpers.hpp>

#include <mjregrasping/first_order_homotopy.h>

using namespace EigenHelpers;
using namespace Eigen;
using namespace arc_utilities;
using namespace arc_utilities::FirstOrderDeformation;

// FIXME: this is copying the SDF every time, which could be slow!
std::vector<ConfigType> getFirstOrderHomotopyPoints(sdf_tools::SignedDistanceField const &sdf, const VectorVector3d& b1, const VectorVector3d& b2, const double sdf_threshold=0.0) {
    static const double one_div_min_step_size = 50.0;

    // Checks if the straight line between elements of the two paths is collision free.
    // i.e. a point on path b1 and a point on path b2
    // In this case we check against an SDF
    const auto straight_line_collision_check_fn = [&] (
            const ssize_t b1_ind,
            const ssize_t b2_ind)
    {
        const auto& b1_node = b1[b1_ind];
        const auto& b2_node = b2[b2_ind];

        const size_t num_steps = (size_t)std::ceil((b2_node - b1_node).norm() * one_div_min_step_size);

        if (num_steps == 0)
        {
            return true;
        }

        // Checking 0 and num_steps to catch the endpoints of the line
        for (size_t ind = 0; ind <= num_steps; ++ind)
        {
            const double ratio = (double)ind / (double)num_steps;
            const Vector3d interpolated_point = Interpolate(b1_node, b2_node, ratio);
            if (sdf.GetImmutable3d(interpolated_point).first < sdf_threshold)
            {
                return false;
            }
        }

        return true;
    };

    return FirstOrderDeformation::GetFirstOrderDeformation(b1.size(), b2.size(), straight_line_collision_check_fn);
}

