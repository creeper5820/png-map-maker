#include "backend/backend.hh"

#include <cxxopts.hpp>
#include <iostream>

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
    auto options = cxxopts::Options{"RMCS Map Maker", "From pcd file to png map!"};

    auto desr = std::vector{
        "Help", "Pcd file path", "Full output path",
        "Using pcl internal method to remove ground on map",
        "Input order: map path, standard template path"};

    options.add_options()                                                     //
        ("h, help", desr[0])                                                  //
        ("p, pcd", desr[1], cxxopts::value<std::string>())                    //
        ("o, out", desr[2], cxxopts::value<std::string>())                    //
        ("registration", desr[4], cxxopts::value<std::vector<std::string>>()) //
        ;

    auto result = options.parse(argc, argv);
    if (result.count("help")) {
        std::cout << options.help();
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

    if (path_pcd.empty() || !path_pcd.ends_with(".pcd"))
        throw std::runtime_error{"Unavailable pcd path"};
    if (path_out.empty())
        path_out = "obstacle-map.png";

    using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
    auto pointcloud  = backend::read_pointcloud(path_pcd);
    backend::remove_ground(pointcloud);

    auto map = backend::generate_map(*pointcloud);
    // backend::encode_png(path_out, *map);

    return 0;
}
