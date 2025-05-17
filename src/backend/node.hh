#pragma once

#include <algorithm>
#include <cassert>
#include <cstdint>
#include <functional>
#include <unordered_set>
#include <vector>

namespace rmcs {

struct ObstacleMap {
public:
    /// @note 当 node_height 为 -1 时，UNKNOWN
    struct Node {
        uint8_t value { 255 };
        std::unordered_set<uint8_t> height_table;

        std::size_t height_table_size() const { return height_table.size(); }

        double maximum_height_range() const {
            if (height_table.empty()) { // 关键修复：处理空集合
                return 0.0;
            }
            const auto [min, max] = std::minmax_element(height_table.begin(), height_table.end());
            return static_cast<double>(*max - *min) / 100.0;
        }

        void update_height_table(double height) {
            const auto key = static_cast<uint8_t>(std::clamp(height, 0.0, 1.0) * 100);
            height_table.insert(key);
        }

        uint8_t color() const { return value; }
    };
    using NodesMatrix = std::vector<std::vector<Node>>;

    /// @note: w -> x
    /// @note: h -> y
    explicit ObstacleMap(std::size_t w, std::size_t h)
        : internal_w(w)
        , internal_h(h) {
        internal_nodes.resize(w);
        for (auto& nodes : internal_nodes)
            nodes.resize(h, Node {});
    }

    void update_node_value(std::size_t x, std::size_t y, std::size_t expand, int8_t v) {
        update_node_with_round_area(
            x, y, expand, [v](std::size_t, std::size_t, Node& node) { node.value = v; });
    }

    void update_node_height(std::size_t x, std::size_t y, std::size_t expand, double height) {
        update_node_with_round_area(x, y, expand,
            [height](std::size_t, std::size_t, Node& node) { node.update_height_table(height); });
    }

    void update_node_with_round_area(std::size_t x, std::size_t y, std::size_t expand,
        std::function<void(std::size_t, std::size_t, Node&)> apply) {

        std::invoke(apply, x, y, (*this)(x, y));

        std::size_t x_min = (x >= expand) ? (x - expand) : 0;
        std::size_t x_max = std::min(x + expand, w() - 1);
        std::size_t y_min = (y >= expand) ? (y - expand) : 0;
        std::size_t y_max = std::min(y + expand, h() - 1);

        const int64_t expand_sq = static_cast<int64_t>(expand) * expand;

        for (std::size_t _x = x_min; _x <= x_max; ++_x) {
            for (std::size_t _y = y_min; _y <= y_max; ++_y) {
                const int64_t dx = static_cast<int64_t>(_x) - static_cast<int64_t>(x);
                const int64_t dy = static_cast<int64_t>(_y) - static_cast<int64_t>(y);

                const int64_t distance_sq = dx * dx + dy * dy;
                if (distance_sq <= expand_sq) std::invoke(apply, _x, _y, (*this)(_x, _y));
            }
        }
    }

    void foreach (std::function<void(std::size_t x, std::size_t y, Node& node)> callback) {
        for (std::size_t x = 0; x < w(); ++x) {
            for (std::size_t y = 0; y < h(); ++y) {
                callback(x, y, (*this)(x, y));
            }
        }
    }

    void foreach (
        std::function<void(std::size_t x, std::size_t y, const Node& node)> callback) const {
        for (std::size_t x = 0; x < w(); ++x) {
            for (std::size_t y = 0; y < h(); ++y) {
                callback(x, y, (*this)(x, y));
            }
        }
    }

    const Node& operator()(std::size_t row, std::size_t col) const {
        assert(row < w() && col < h());
        return internal_nodes[row][col];
    }
    Node& operator()(std::size_t row, std::size_t col) {
        assert(row < w() && col < h());
        return internal_nodes[row][col];
    }

    const NodesMatrix& data() const { return internal_nodes; }
    NodesMatrix& data() { return internal_nodes; }

    std::size_t w() const { return internal_w; }
    std::size_t h() const { return internal_h; }

private:
    NodesMatrix internal_nodes;
    std::size_t internal_w;
    std::size_t internal_h;
};

} // namespace rmcs
