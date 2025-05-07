#pragma once

#include <memory>
#include <string>

#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "lodepng/lodepng.hh"
#include "node.hh"

namespace rmcs {

template <typename _value_type, class _next_config>
struct _internal_config_value {
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

#define RMCS_ADD_CONFIG(NAME, TYPE)                                                   \
    template <class _next_config, typename _value_type = TYPE>                        \
    struct NAME : protected _internal_config_value<TYPE, _next_config> {              \
        _next_config& set_##NAME(const _value_type& v) { return this->set_value(v); } \
        _next_config& get_##NAME(_value_type& v) { return this->get_value(v); }       \
    }

struct backend {
public:
    using Point      = pcl::PointXYZ;
    using PointCloud = pcl::PointCloud<Point>;

    static std::shared_ptr<PointCloud> read_pointcloud(const std::string& path) {
        auto pointcloud = std::make_shared<PointCloud>();
        if (pcl::io::loadPCDFile(path, *pointcloud) == -1) {
            throw std::runtime_error{"Something went wrong while reading pcd file"};
        }
        return pointcloud;
    }

    static void remove_ground(const std::shared_ptr<PointCloud>& pointcloud) {
        auto outside_condition = std::make_shared<pcl::ConditionAnd<PointCloud::PointType>>();
        outside_condition->addComparison(
            std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
                "z", pcl::ComparisonOps::LT, 3.0));
        outside_condition->addComparison(
            std::make_shared<const pcl::FieldComparison<PointCloud::PointType>>(
                "z", pcl::ComparisonOps::GT, 0.0));

        pcl::ConditionalRemoval<PointCloud::PointType> removal;
        removal.setCondition(outside_condition);
        removal.setInputCloud(pointcloud);

        auto pointcloud_fit_height = std::make_shared<PointCloud>();
        removal.filter(*pointcloud_fit_height);

        auto pass_through = pcl::PassThrough<PointCloud::PointType>{};
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
        auto segmentation                       = pcl::SACSegmentation<Point>{};
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

        auto extract = pcl::ExtractIndices<PointCloud::PointType>{};
        extract.setInputCloud(pointcloud_fit_height);
        extract.setIndices(indices_ground_from_segment_source);
        extract.setNegative(true);
        extract.filter(*pointcloud);
    }

    static std::unique_ptr<ObstacleMap>
        generate_map(const PointCloud& pointcloud, double resolution = 0.1) {
        const auto f = [&](float v) { return static_cast<std::size_t>(v / resolution); };

        auto point_min = Point{};
        auto point_max = Point{};
        pcl::getMinMax3D(pointcloud, point_min, point_max);
        std::printf(
            "Point cloud area: (%4.2f, %4.2f, %4.2f) -> (%4.2f, %4.2f, %4.2f)\n", point_min.x,
            point_min.y, point_min.z, point_max.x, point_max.y, point_max.z);

        auto w   = point_max.x - point_min.x;
        auto h   = point_max.y - point_min.y;
        auto map = std::make_unique<ObstacleMap>(w, h);

        for (const auto& point : pointcloud) {}

        return nullptr;
    }

    static void encode_png(const std::string& path, const ObstacleMap& map) {
        auto data = std::vector<std::uint8_t>(map.h() * map.w(), std::uint8_t{0});
        for (auto x = 0; x < map.w(); x++)
            for (auto y = 0; y < map.h(); y++) {
                data[x + y * map.w()] = std::clamp<uint8_t>(map(x, y).value, 0, 255);
            }

        auto w      = uint{static_cast<uint>(map.w())};
        auto h      = uint{static_cast<uint>(map.h())};
        auto result = lodepng::encode(path, data, w, h, LCT_GREY);
    }
};

} // namespace rmcs
