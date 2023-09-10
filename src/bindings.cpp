#include <mjregrasping/first_order_homotopy.h>
#include <mjregrasping/planning.h>
#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <pyrosmsg/converters.hpp>

namespace py = pybind11;

PYBIND11_MODULE(pymjregrasping_cpp, m) {
  m.def("get_first_order_homotopy_points", getFirstOrderHomotopyPoints, py::arg("is_collision"), py::arg("b1"),
        py::arg("b2"));

  m.def("seedOmpl", seedOmpl, py::arg("seed"));

  py::class_<RRTPlanner>(m, "RRTPlanner")
      .def(py::init<>())
      .def("plan", &RRTPlanner::plan, py::arg("scene_msg"), py::arg("group_name"), py::arg("goal_positions"),
           py::arg("viz") = true, py::arg("allowed_planning_time") = 5.0, py::arg("pos_noise") = 0.001, py::arg("max_ik_attempts") = 35, py::arg("max_ik_solutions") = 10, py::arg("joint_noise") = 1)
      .def("is_state_valid", &RRTPlanner::is_state_valid, py::arg("scene_msg"))
      .def("display_result", &RRTPlanner::display_result, py::arg("res_msg"));
}
