#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "patchworkpp.h"

namespace py = pybind11;

PYBIND11_MODULE(pypatchworkpp, m) {

    m.doc() = "Python Patchwork";
    m.attr("__version__") = 1;

    py::class_<patchwork::Params>(m, "Parameters")
        .def(py::init<>())
        .def_readwrite("sensor_height", &patchwork::Params::sensor_height)
        .def_readwrite("verbose",       &patchwork::Params::verbose)
        .def_readwrite("enable_RNR",    &patchwork::Params::enable_RNR)
        .def_readwrite("enable_RVPF",   &patchwork::Params::enable_RVPF)
        .def_readwrite("enable_TGR",    &patchwork::Params::enable_TGR)
        .def_readwrite("num_iter",      &patchwork::Params::num_iter)
        .def_readwrite("num_lpr",       &patchwork::Params::num_lpr)
        .def_readwrite("num_min_pts",   &patchwork::Params::num_min_pts)
        .def_readwrite("num_zones",     &patchwork::Params::num_zones)
        .def_readwrite("num_rings_of_interest",     &patchwork::Params::num_rings_of_interest)
        .def_readwrite("noise_filter_channel_num",  &patchwork::Params::noise_filter_channel_num)
        .def_readwrite("pc_num_channel",    &patchwork::Params::pc_num_channel)
        .def_readwrite("sensor_height",     &patchwork::Params::sensor_height)
        .def_readwrite("th_seeds",      &patchwork::Params::th_seeds)
        .def_readwrite("th_dist",       &patchwork::Params::th_dist)
        .def_readwrite("th_seeds_v",    &patchwork::Params::th_seeds_v)
        .def_readwrite("th_dist_v",     &patchwork::Params::th_dist_v)
        .def_readwrite("max_range",     &patchwork::Params::max_range)
        .def_readwrite("min_range",     &patchwork::Params::min_range)
        .def_readwrite("uprightness_thr", &patchwork::Params::uprightness_thr)
        .def_readwrite("adaptive_seed_selection_margin", &patchwork::Params::adaptive_seed_selection_margin)
        .def_readwrite("intensity_thr", &patchwork::Params::intensity_thr)
        .def_readwrite("num_sectors_each_zone", &patchwork::Params::num_sectors_each_zone)
        .def_readwrite("num_rings_each_zone", &patchwork::Params::num_rings_each_zone)
        .def_readwrite("max_flatness_storage", &patchwork::Params::max_flatness_storage)
        .def_readwrite("max_elevation_storage", &patchwork::Params::max_elevation_storage)
        .def_readwrite("elevation_thr", &patchwork::Params::elevation_thr)
        .def_readwrite("flatness_thr", &patchwork::Params::flatness_thr);

    py::class_<patchwork::PatchWorkpp>(m, "patchworkpp")
        .def(py::init<patchwork::Params>())
        .def("getHeight",       &patchwork::PatchWorkpp::getHeight)
        .def("getTimeTaken",    &patchwork::PatchWorkpp::getTimeTaken)
        .def("getGround",       &patchwork::PatchWorkpp::getGround)
        .def("getNonground",    &patchwork::PatchWorkpp::getNonground)
        .def("getCenters",      &patchwork::PatchWorkpp::getCenters)
        .def("getNormals",      &patchwork::PatchWorkpp::getNormals)        
        .def("estimateGround",  &patchwork::PatchWorkpp::estimateGround);
        // .def_readwrite("sensor_height_", &PatchWorkpp::sensor_height_);
}