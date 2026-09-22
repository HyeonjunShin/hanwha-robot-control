#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>
#include "hanwha_robot.h"

namespace py = pybind11;

PYBIND11_MODULE(hanwha_robot_py, m) {
    py::class_<IRobot>(m, "IRobot");
    py::class_<Hanwha, IRobot>(m, "Hanwha")
      .def(py::init<const std::string &>(), py::arg("IP"))
      
      .def("disconnect", &Hanwha::disconnect, "Disconnect from the robot")
      .def("run", &Hanwha::run, "Run the robot process")
      
      .def("conn_flange_shm", &Hanwha::conn_flange_shm, py::arg("shm_name"), 
            "Connect to flange shared memory");
}
