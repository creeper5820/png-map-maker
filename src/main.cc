#include "backend/backend.hh"

#include <cmath>
#include <cstddef>
#include <cxxopts.hpp>
#include <iostream>
#include <stdexcept>

using namespace rmcs;

constexpr auto greeting_texture = R"(
   ██████╗██████╗ ███████╗███████╗██████╗ ███████╗██████╗ 
  ██╔════╝██╔══██╗██╔════╝██╔════╝██╔══██╗██╔════╝██╔══██╗
  ██║     ██████╔╝█████╗  █████╗  ██████╔╝█████╗  ██████╔╝
  ██║     ██╔══██╗██╔══╝  ██╔══╝  ██╔═══╝ ██╔══╝  ██╔══██╗
  ╚██████╗██║  ██║███████╗███████╗██║     ███████╗██║  ██║
   ╚═════╝╚═╝  ╚═╝╚══════╝╚══════╝╚═╝     ╚══════╝╚═╝  ╚═╝
  ________________________________________________________
  Light tool to generate png map from pcd
  creeper5820
)";

auto main(int argc, const char* const* argv) -> int {

    const auto desr = std::vector{
        "Help Info",                                                                           // 0
        "PCD file path, as map generation data, ground should be near Z=0, right-hand system", // 1
        "Output file path, default is output.png",                                             // 2
        "Registration map path, if provided, registration will be performed",                  // 3
        "Grid map resolution, actual length of one grid in meters",                            // 4
        "The limit of effective points in a grid",                                             // 5
        "The distance limit between min and max height",                                       // 6
        "The grid size of expanding",                                                          // 7
    };

    auto options = cxxopts::Options{"RMCS Map Maker", "From pcd file to png map!"};
    options.add_options()                                                                      //
        ("h, help", desr[0])                                                                   //
        ("p, pcd", desr[1], cxxopts::value<std::string>())                                     //
        ("o, out", desr[2], cxxopts::value<std::string>())                                     //
        ("registration", desr[3], cxxopts::value<std::string>())                               //
        ("resolution", desr[4], cxxopts::value<double>())                                      //
        ("points_limit", desr[5], cxxopts::value<std::size_t>())                               //
        ("height_limit", desr[6], cxxopts::value<double>())                                    //
        ("expand_size", desr[7], cxxopts::value<std::size_t>())                                //
        ;

    const auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << greeting_texture << std::endl;
        std::cout << options.help() << std::endl;
        return 0;
    }

    auto path_pcd = std::string{};
    if (result.count("pcd")) {
        path_pcd = result["pcd"].as<std::string>();
    } else {
        std::cout << greeting_texture << std::endl;
        return 0;
    }

    auto path_out = std::string{"output.png"};
    if (result.count("out")) {
        path_out = result["out"].as<std::string>();
    }

    auto resolution = std::double_t{0.1};
    if (result.count("resolution")) {
        resolution = result["resolution"].as<double>();
        if (resolution < 0.01)
            throw std::runtime_error{"这么小的分辨率，你想你内存爆掉吗？"};
    }

    auto points_limit = std::size_t{5};
    if (result.count("points_limit")) {
        points_limit = result["points_limit"].as<std::size_t>();
    }

    auto height_limit = double{0.1};
    if (result.count("height_limit")) {
        height_limit = result["height_limit"].as<double>();
    }

    auto expand_size = std::size_t{0};
    if (result.count("expand_size")) {
        expand_size = result["expand_size"].as<std::size_t>();
    }

    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    auto pointcloud  = backend::read_pointcloud(path_pcd);

    config::set_resolution(resolution);
    config::set_points_limit(points_limit);
    config::set_height_limit(height_limit);
    config::set_expand_size(expand_size);
    auto map = backend::generate_map(*pointcloud);

    backend::encode_png(path_out, *map);

    return 0;
}
