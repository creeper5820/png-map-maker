#pragma once

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <unordered_set>
#include <vector>

namespace rmcs {

struct ObstacleMap {
public:
    /// @note 当 node_height 为 -1 时，UNKNOWN
    struct Node {
        int8_t value{-1};

        // 高度表，0 -> 100 对应 0 -> 1 m
        std::unordered_set<uint8_t> height_table;

        std::size_t height_table_size() const { //
            return height_table.size();
        }
        double maximum_height_range() const {
            const auto& table        = height_table;
            const auto& [begin, end] = std::tuple(table.begin(), table.end());
            const auto& [min, max]   = std::minmax_element(begin, end);
            return static_cast<double>(*max - *min) / 100.;
        }
        void update_height_table(double height) {
            const auto get_height_value = [](double h) {
                return static_cast<uint8_t>(std::clamp(h, 0., 1.) * 100);
            };
            const auto key = get_height_value(height);
            height_table.insert(key);
        }
    };
    using NodesMatrix = std::vector<std::vector<Node>>;

    /// @note: w -> x
    /// @note: h -> y
    explicit ObstacleMap(std::size_t w, std::size_t h)
        : internal_w(w) {
        internal_nodes.resize(w);
        for (auto& nodes : internal_nodes)
            nodes.resize(h, Node{});
    }

    void update_node(std::size_t x, std::size_t y, int8_t v) {
        assert(x < w() && y < h());
        internal_nodes[x][y].value = v;
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
