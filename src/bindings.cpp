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
           py::arg("viz") = true, py::arg("allowed_planning_time") = 5.0)
      .def("display_result", &RRTPlanner::display_result, py::arg("res_msg"));
}
