#pragma once

#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "lodepng/lodepng.hh"
#include "node.hh"

namespace std {
template <> struct hash<std::pair<std::size_t, std::size_t>> {
    std::size_t operator()(const std::pair<std::size_t, std::size_t>& p) const {
        auto h1 = std::hash<std::size_t> {}(p.first);
        auto h2 = std::hash<std::size_t> {}(p.second);
        return h1 ^ h2;
    }
};
} // namespace std

namespace rmcs {

namespace config {

    inline auto resolution = std::double_t { 0.1 };
    static void set_resolution(std::double_t resolution) { config::resolution = resolution; }

    inline auto points_limit = std::size_t { 5 };
    static void set_points_limit(std::size_t points_limit) { config::points_limit = points_limit; }

    inline auto height_limit = std::double_t { 0.2 };
    static void set_height_limit(std::double_t height_limit) {
        config::height_limit = height_limit;
    }

    inline auto expand_size = std::size_t { 0 };
    static void set_expand_size(std::size_t expand_size) { config::expand_size = expand_size; }

} // namespace config

template <typename _value_type, class _next_config> struct _internal_config_value {
protected:
    _next_config& set_value(const _value_type& value) {
        this->value = value;
        return this->next;
    }
    _next_config& get_value(_value_type& value) {
        value = this->value;
        return this->next;
    }
    _next_config next;
    _value_type value;
};

#define RMCS_ADD_CONFIG(NAME, TYPE)                                                                \
    template <class _next_config, typename _value_type = TYPE>                                     \
    struct NAME : protected _internal_config_value<TYPE, _next_config> {                           \
        _next_config& set_##NAME(const _value_type& v) { return this->set_value(v); }              \
        _next_config& get_##NAME(_value_type& v) { return this->get_value(v); }                    \
    }

struct backend {
public:
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    static std::shared_ptr<PointCloud> read_pointcloud(const std::string& path) {
        std::printf("从 %s 加载点云数据\n", path.c_str());

        auto pointcloud = std::make_shared<PointCloud>();
        if (pcl::io::loadPCDFile(path, *pointcloud) == -1) {
            throw std::runtime_error { "Something went wrong while reading pcd file" };
        }

        std::printf("加载完毕，点云数量为 %zu\n", pointcloud->size());
        return pointcloud;
    }

    static void remove_ground(const std::shared_ptr<PointCloud>& pointcloud) {
        auto outside_condition = std::make_shared<pcl::ConditionAnd<PointCloud::PointType>>();
        outside_condition->addComparison(
            std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
                "z", pcl::ComparisonOps::LT, 2.0));
        outside_condition->addComparison(
            std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
                "z", pcl::ComparisonOps::GT, -0.5));

        pcl::ConditionalRemoval<PointCloud::PointType> removal;
        removal.setCondition(outside_condition);
        removal.setInputCloud(pointcloud);

        auto pointcloud_fit_height = std::make_shared<PointCloud>();
        removal.filter(*pointcloud_fit_height);

        auto pass_through = pcl::PassThrough<PointCloud::PointType> {};
        pass_through.setInputCloud(pointcloud_fit_height);
        pass_through.setFilterFieldName("z");
        pass_through.setFilterLimits(-1, 0.2);

        // 只留下地面附近的点云作为分割的输入
        auto pointcloud_for_segment             = std::make_shared<PointCloud>();
        auto indices_segment_source_from_origin = std::make_shared<pcl::PointIndices>();
        pass_through.filter(*pointcloud_for_segment);
        pass_through.filter(indices_segment_source_from_origin->indices);

        auto indices_ground_from_segment_source = std::make_shared<pcl::PointIndices>();
        auto coefficients                       = std::make_shared<pcl::ModelCoefficients>();
        auto segmentation                       = pcl::SACSegmentation<Point> {};
        segmentation.setOptimizeCoefficients(true);
        segmentation.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);

        segmentation.setAxis(Eigen::Vector3f::UnitZ());
        segmentation.setEpsAngle(std::numbers::pi / 180 * 10);

        segmentation.setDistanceThreshold(0.05);
        segmentation.setMaxIterations(1'000);
        segmentation.setInputCloud(pointcloud_for_segment);
        segmentation.segment(*indices_ground_from_segment_source, *coefficients);

        // 把分割后的点云从最初只限制的大范围的场景点云中提出，所以需要二次映射序列
        // 重新映射点的序列，由于分割前的点云已经经过了处理，我们需要保留一份处理过的点云的序列
        // 然后从序列中取出分割的序列，再映射回原点云，获取足够完美的分割地图
        auto indices_ground_from_origin = std::make_shared<pcl::PointIndices>();
        for (auto index : indices_ground_from_segment_source->indices) {
            if (index >= 0 && index < indices_segment_source_from_origin->indices.size())
                indices_ground_from_origin->indices.push_back(
                    indices_segment_source_from_origin->indices[index]);
        }

        auto extract = pcl::ExtractIndices<PointCloud::PointType> {};
        extract.setInputCloud(pointcloud_fit_height);
        extract.setIndices(indices_ground_from_segment_source);
        extract.setNegative(true);
        extract.filter(*pointcloud);

        std::printf("去除地面结束，剩余点云大小为 %zu\n", pointcloud->size());
    }

    static void remove_outlier_points(const std::shared_ptr<PointCloud>& pointcloud) {
        const auto resolution = config::resolution;

        auto outlier_removal_filter = std::make_unique<pcl::StatisticalOutlierRemoval<Point>>();
        outlier_removal_filter->setMeanK(20);
        outlier_removal_filter->setStddevMulThresh(0.5);
        outlier_removal_filter->setInputCloud(pointcloud);
        outlier_removal_filter->filter(*pointcloud);

        auto voxel = pcl::VoxelGrid<Point> {};
        voxel.setLeafSize(Eigen::Vector4f(resolution, resolution, resolution, 1.0));
        voxel.setInputCloud(pointcloud);
        voxel.filter(*pointcloud);

        std::printf("滤波完毕，剩余点云大小为 %zu\n", pointcloud->size());
    }

    static std::unique_ptr<ObstacleMap> generate_map(const PointCloud& pointcloud) {
        const auto resolution = config::resolution;
        const auto expand     = static_cast<size_t>(0.1 / resolution);

        auto point_min = Point {};
        auto point_max = Point {};
        pcl::getMinMax3D(pointcloud, point_min, point_max);
        std::printf("点云区域: (%4.2f, %4.2f, %4.2f) -> (%4.2f, %4.2f, %4.2f)\n", point_min.x,
            point_min.y, point_min.z, point_max.x, point_max.y, point_max.z);

        auto f   = [&](float v) { return static_cast<std::size_t>(v / resolution); };
        auto w   = f(point_max.x - point_min.x) + 1;
        auto h   = f(point_max.y - point_min.y) + 1;
        auto map = std::make_unique<ObstacleMap>(w, h);

        // 更新高度表，由于点云较为稀疏，所以取 0.1 / resolution 的影响范围，增加单个栅格内的信息量
        auto visited = std::unordered_set<std::pair<std::size_t, std::size_t>> {};
        for (const auto& point : pointcloud) {
            const auto update = [&](std::size_t x, std::size_t y, ObstacleMap::Node& node) {
                node.update_height_table(point.z);
                visited.insert(std::make_pair(x, y));
            };
            const auto x = f(point.x - point_min.x), y = f(point.y - point_min.y);
            map->update_node_with_round_area(x, y, expand, update);
        }
        std::printf("更新所有节点高程表\n");

        for (auto i = f(std::abs(point_min.x)); i < w; i++)
            (*map)(i, f(std::abs(point_min.y))).value = 255;
        for (auto i = f(std::abs(point_min.y)); i < h; i++)
            (*map)(f(std::abs(point_min.x)), i).value = 255;
        std::printf("初始点坐标 (%zu, %zu)\n", f(std::abs(point_min.x)), f(std::abs(point_min.y)));

        for (const auto [x, y] : visited) {
            auto& node = (*map)(x, y);
            if (node.height_table_size() < config::points_limit) continue;
            if (node.maximum_height_range() < config::height_limit) continue;
            map->update_node_value(x, y, config::expand_size, 0);
        }
        std::printf("更新所有节点值\n");

        return map;
    }

    static void encode_png(const std::string& path, const ObstacleMap& map) {
        std::printf("障碍地图栅格大小x方向: %zu, y方向: %zu(右手系)\n", map.w(), map.h());

        auto data = std::vector<std::uint8_t>();
        data.resize(map.w() * map.h(), 0);

        for (auto x = 0; x < map.w(); x++)
            for (auto y = 0; y < map.h(); y++) {
                auto x_ = map.w() - x - 1, y_ = map.h() - y - 1;
                data[x_ * map.h() + y_] = map(x, y).color();
            }

        auto w      = uint { static_cast<uint>(map.h()) };
        auto h      = uint { static_cast<uint>(map.w()) };
        auto result = lodepng::encode(path, data, w, h, LCT_GREY);
    }
};

} // namespace rmcs
