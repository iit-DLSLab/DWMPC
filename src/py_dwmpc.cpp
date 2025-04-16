#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
// #include <pybind11/stl_bind.h>
#include "controllers/dwmpc/dwmpc.hpp"// Include the header file for the Dwmpc class

namespace py = pybind11;
Eigen::Quaterniond numpy_to_quaternion(const py::array_t<double>& array) {
    auto buf = array.unchecked<1>();  // Unpack the NumPy array
    return Eigen::Quaterniond(buf(0), buf(1), buf(2), buf(3));  // w, x, y, z order
}
PYBIND11_MAKE_OPAQUE(std::vector<double>)

    

namespace controllers
{
PYBIND11_MODULE(pydwmpc, m) {
    py::class_<pdata>(m, "pdata")
        .def(py::init<>())
        .def_readwrite("p", &pdata::p, "Position")
        .def_readwrite("quat", &pdata::quat, "Quaternion")
        .def_readwrite("q", &pdata::q, "Joint angles")
        .def_readwrite("dp", &pdata::dp, "Linear velocity prediction")
        .def_readwrite("omega", &pdata::omega, "Angular velocity")
        .def_readwrite("dq", &pdata::dq, "Joint velocity")
        .def_readwrite("grf", &pdata::grf, "Ground reaction forces")
        .def_readwrite("tau", &pdata::tau, "Joint torque")
        .def_readwrite("dual", &pdata::dual, "Dual variables")
        .def_readwrite("residual", &pdata::residual, "Residuals");
    py::class_<std::vector<double>>(m, "DoubleVector")
        .def(py::init<>())
        .def(py::init<const std::vector<double>&>())  // Initialize from a list of double
        .def(py::init([](py::list list) {
            std::vector<double> vec;
            for (auto item : list) {
                vec.push_back(py::cast<double>(item));
            }
            return new std::vector<double>(vec);
        }))
        .def("clear", &std::vector<double>::clear)
        .def("pop_back", &std::vector<double>::pop_back)
        .def("__len__", [](const std::vector<double> &v) { return v.size(); })
        .def("__iter__", [](std::vector<double> &v) {
          return py::make_iterator(v.begin(), v.end());
        }, py::keep_alive<0, 1>())
        .def("__getitem__", [](const std::vector<double> &v, size_t i) {
            if (i >= v.size()) throw py::index_error();
            return v[i];})
        .def("__setitem__", [](std::vector<double> &v, size_t i, double val) {
            if (i >= v.size()) throw py::index_error();
            v[i] = val;
        })
        .def("print", [](const std::vector<double> &v) {
            for (const auto& item : v) {
                 std::cout << item << " ";
            }
            std::cout << std::endl;
        })
        .def("getList", [](const std::vector<double>& v) {
            py::list list;
            for (const auto& item : v) {
                list.append(item);
            }
            return list;
            })
        ;
    py::class_<Dwmpc>(m, "Dwmpc")
        .def(py::init<>())  // Assuming Dwmpc has a default constructor
        .def("init", &Dwmpc::init)
        .def("run", static_cast<void (controllers::Dwmpc::*)(const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const Eigen::Ref<Eigen::Vector4d>&,
                                                             const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const double&,
                                                             const Eigen::Ref<const Eigen::Vector4d>&,
                                                             const Eigen::Ref<const Eigen::MatrixXd>&,
                                                             const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const Eigen::Ref<const Eigen::VectorXd>&,
                                                             const Eigen::Ref<Eigen::Vector4d>&,
                                                             std::vector<double>&,
                                                             std::vector<double>&,
                                                             std::vector<double>&,
                                                             std::vector<double>&)>(&controllers::Dwmpc::run),
             py::arg("p"), py::arg("quat"), py::arg("q_op"), py::arg("dp"), py::arg("omega"), py::arg("dq_op"),
             py::arg("loop_dt"), py::arg("current_contact"), py::arg("foot_op"), py::arg("desired_linear_speed"),
             py::arg("desired_angular_speed"), py::arg("desired_orientation"), py::arg("des_contact"),
             py::arg("des_tau"), py::arg("des_q"), py::arg("des_dq"))
        .def("setWeight", &Dwmpc::setWeight)
        .def("setGaitParam", py::overload_cast<const double, const double, const std::vector<double>&>(&Dwmpc::setGaitParam))
        .def("setGaitParam", py::overload_cast<const double, const double, const int>(&Dwmpc::setGaitParam))
        .def("updateTimer", &Dwmpc::updateTimer)
        .def("reset", &Dwmpc::reset)
        .def("startWalking", &Dwmpc::startWalking)
        .def("stopWalking", &Dwmpc::stopWalking)
        .def("prepare", &Dwmpc::prepare)
        .def("sineWave", &Dwmpc::sineWave)
        .def("setSineParam", &Dwmpc::setSineParam)
        .def("startSineWave", &Dwmpc::startSineWave)
        .def("stopSineWave", &Dwmpc::stopSineWave)
        .def("getWeight", &Dwmpc::getWeight)
        .def("goHandStand", &Dwmpc::goHandStand)
        .def("stopHandStand", &Dwmpc::stopHandStand)
        .def("setStepHeight", &Dwmpc::setStepHeight)
        .def("getFullPrediction", [](Dwmpc &self) {
            // Wrapper function for getFullPrediction to create and return a Python dictionary
            std::map<std::string, pdata> prediction;
            self.getFullPrediction(prediction);
            return prediction; // Pybind11 converts std::map<std::string, pdata> to a Python dict
        }, "Retrieve the full prediction data");
}
} //namespace controllers