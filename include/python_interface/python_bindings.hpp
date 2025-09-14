#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>
#include "problem_description/problem_description.hpp"

namespace py = pybind11;

template <typename... Args>
using overload_cast_ = pybind11::detail::overload_cast_impl<Args...>;

void init_distributions(py::module_ &);
void init_constraints(py::module_ &);
void init_transformations(py::module_ &);
void init_problem_descriptions(py::module_ &);
void init_gaussian_process(py::module_ &);
void init_kernel(py::module_ &);
void init_polynomial(py::module &);

template <typename Class>
void problem_methods(py::class_<Class, grampc_s::ProblemDescription, std::shared_ptr<Class>> &cls)
{
    cls.def("ffct", &Class::ffct);
    cls.def("dfdx_vec", &Class::dfdx_vec);
    cls.def("dfdu_vec", &Class::dfdu_vec);
    cls.def("dfdp_vec", &Class::dfdp_vec);

    cls.def("lfct", &Class::lfct);
    cls.def("dldx", &Class::dldx);
    cls.def("dldu", &Class::dldu);
    cls.def("dldp", &Class::dldp);

    cls.def("Vfct", &Class::Vfct);
    cls.def("dVdx", &Class::dVdx);
    cls.def("dVdp", &Class::dVdp);
    cls.def("dVdT", &Class::dVdT);

    cls.def("gfct", &Class::gfct);
    cls.def("dgdx_vec", &Class::dgdx_vec);
    cls.def("dgdu_vec", &Class::dgdu_vec);
    cls.def("dgdp_vec", &Class::dgdp_vec);

    cls.def("hfct", &Class::hfct);
    cls.def("dhdx_vec", &Class::dhdx_vec);
    cls.def("dhdu_vec", &Class::dhdu_vec);
    cls.def("dhdp_vec", &Class::dhdp_vec);

    cls.def("gTfct", &Class::gTfct);
    cls.def("dgTdx_vec", &Class::dgTdx_vec);
    cls.def("dgTdp_vec", &Class::dgTdp_vec);
    cls.def("dgTdT_vec", &Class::dgTdT_vec);

    cls.def("hTfct", &Class::hTfct);
    cls.def("dhTdx_vec", &Class::dhTdx_vec);
    cls.def("dhTdp_vec", &Class::dhTdp_vec);
    cls.def("dhTdT_vec", &Class::dhTdT_vec);
}
