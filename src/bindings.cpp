#include <moveit/python/pybind_rosmsg_typecasters.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <mjregrasping/first_order_homotopy.h>

namespace py = pybind11;

PYBIND11_MODULE(pymjregrasping_cpp, m) {
    m.def("get_first_order_homotopy_points", getFirstOrderHomotopyPoints, py::arg("sdf"), py::arg("b1"), py::arg("b2"), py::arg("sdf_threshold"));
}
