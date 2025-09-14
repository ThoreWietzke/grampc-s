#include "python_interface/python_bindings.hpp"
#include "problem_description/resampling_problem_description.hpp"
#include "problem_description/monte_carlo_problem_description.hpp"
#include "problem_description/resampling_GP_problem_description.hpp"
#include "problem_description/sigma_point_problem_description.hpp"
#include "problem_description/taylor_problem_description.hpp"

using namespace grampc_s;

void init_problem_descriptions(py::module_ &m)
{
    py::module_::import("pygrampc");
    
    py::class_<ResamplingProblemDescription, ProblemDescription, ResamplingProblemDescriptionPtr> resampling(m, "ResamplingProblemDescription");
    resampling.def(py::init<ProblemDescriptionPtr, ChanceConstraintApproximationConstPtr, PointTransformationPtr>())
        .def(py::init<ProblemDescriptionPtr, ChanceConstraintApproximationConstPtr, PointTransformationPtr, MatrixConstRef>())
        .def(py::init<ProblemDescriptionPtr, PointTransformationPtr>())
        .def(py::init<ProblemDescriptionPtr, PointTransformationPtr, MatrixConstRef>())
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr>()(&ResamplingProblemDescription::compute_x0_and_p0))
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr, DistributionPtr>()(&ResamplingProblemDescription::compute_x0_and_p0))
        .def_property_readonly("x0", &ResamplingProblemDescription::x0)
        .def_property_readonly("p0", &ResamplingProblemDescription::p0);

    problem_methods<ResamplingProblemDescription>(resampling);

    py::class_<ResamplingGPProblemDescription, ProblemDescription, ResamplingGPProblemDescriptionPtr> resampling_gp(m, "ResamplingGPProblemDescription");
    resampling_gp.def(py::init<ProblemDescriptionPtr, ChanceConstraintApproximationConstPtr, PointTransformationPtr, const std::vector<GaussianProcessPtr>&, const std::vector<typeInt>&>())
        .def(py::init<ProblemDescriptionPtr, PointTransformationPtr,const std::vector<GaussianProcessPtr>&, const std::vector<typeInt>&>())
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr>()(&ResamplingGPProblemDescription::compute_x0_and_p0))
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr, DistributionPtr>()(&ResamplingGPProblemDescription::compute_x0_and_p0))
        .def_property_readonly("x0", &ResamplingGPProblemDescription::x0)
        .def_property_readonly("p0", &ResamplingGPProblemDescription::p0);

    problem_methods<ResamplingGPProblemDescription>(resampling_gp);
        
    py::class_<MonteCarloProblemDescription, ProblemDescription, MonteCarloProblemDescriptionPtr> monte_carlo(m, "MonteCarloProblemDescription");
    monte_carlo.def(py::init<ProblemDescriptionPtr, PointTransformationPtr>())
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr>()(&MonteCarloProblemDescription::compute_x0_and_p0))
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr, DistributionPtr>()(&MonteCarloProblemDescription::compute_x0_and_p0))
        .def_property_readonly("x0", &MonteCarloProblemDescription::x0)
        .def_property_readonly("p0", &MonteCarloProblemDescription::p0);

    problem_methods<MonteCarloProblemDescription>(monte_carlo);

    py::class_<SigmaPointProblemDescription, ProblemDescription, SigmaPointProblemDescriptionPtr> sigma_point(m, "SigmaPointProblemDescription");
    sigma_point.def(py::init<ProblemDescriptionPtr, ChanceConstraintApproximationConstPtr, PointTransformationPtr>())
        .def(py::init<ProblemDescriptionPtr, PointTransformationPtr>())
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr>()(&SigmaPointProblemDescription::compute_x0_and_p0))
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr, DistributionPtr>()(&SigmaPointProblemDescription::compute_x0_and_p0))
        .def_property_readonly("x0", &SigmaPointProblemDescription::x0)
        .def_property_readonly("p0", &SigmaPointProblemDescription::p0);

    problem_methods<SigmaPointProblemDescription>(sigma_point);

    py::class_<TaylorProblemDescription, ProblemDescription, TaylorProblemDescriptionPtr> taylor(m, "TaylorProblemDescription");
    taylor.def(py::init<ProblemDescriptionPtr, ChanceConstraintApproximationConstPtr>())
        .def(py::init<ProblemDescriptionPtr, ChanceConstraintApproximationConstPtr, MatrixConstRef>())
        .def(py::init<ProblemDescriptionPtr>())
        .def(py::init<ProblemDescriptionPtr, MatrixConstRef>())
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr>()(&TaylorProblemDescription::compute_x0_and_p0))
        .def("compute_x0_and_p0", overload_cast_<DistributionPtr, DistributionPtr>()(&TaylorProblemDescription::compute_x0_and_p0))
        .def_property_readonly("x0", &TaylorProblemDescription::x0)
        .def_property_readonly("p0", &TaylorProblemDescription::p0);

    problem_methods<TaylorProblemDescription>(taylor);
}