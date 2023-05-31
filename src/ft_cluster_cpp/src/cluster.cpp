#include <algorithm>
#include <atomic>
#include <iterator>
#include <optional>
#include <cmath>
#include <cstring>
#include <vector>
#include <array>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <functional>
#include <cstdlib>

#include <rclcpp/rclcpp.hpp>
#include <eufs_msgs/srv/cluster.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/cone_with_covariance.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <ament_index_cpp/get_package_prefix.hpp>

#include "cluster_gl.hpp"

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "eufs_msgs/msg/detail/cone_array__struct.hpp"
#include "rclcpp/publisher.hpp"

using namespace std;

using uchar = unsigned char;
using Point = std::array<float, 3>;

Point add_points(Point a, Point b) {
    return {a[0] + b[0],
            a[1] + b[1],
            a[2] + b[2]};
}

Point div_point(Point a, float denominator) {
    return {a[0] / denominator,
            a[1] / denominator,
            a[2] / denominator};
}

float point_length(Point p) { return p[0] * p[0] + p[1] * p[1] + p[2] * p[2]; }

// Color is stored externally
struct ConeMetric {
    Point sum;
    int area;

    ConeMetric(Point point) : area(1), sum(point) {}
    ConeMetric() : area(0), sum{0,0,0} {}

    bool empty() {
        return area == 0;
    }

    ConeMetric operator+=(const ConeMetric & other) {
        this->sum = add_points(this->sum, other.sum);
        this->area += other.area;
        return *this;
    }

    ConeMetric operator+(const ConeMetric & other) const {
        ConeMetric sum;
        sum += *this;
        sum += other;
        return sum;
    }
    
    ConeMetric operator+=(const Point & other) {
        this->sum = add_points(this->sum, other);
        this->area++;
    }

    ConeMetric operator+(const Point & other) const {
        ConeMetric sum;
        sum += *this;
        sum += other;
        return sum;
    }

};

std::vector<Point> bytes_to_points(const std::vector<uchar> & bytes) {
    std::vector<Point> points;
    points.resize(bytes.size() / sizeof(Point));
    memcpy(points.data(), bytes.data(), bytes.size());
    return points;
}

std::vector<unsigned char> points_to_bytes(const std::vector<Point> & points) {
    std::vector<uchar> bytes;
    bytes.resize(points.size() * sizeof(Point));
    memcpy(bytes.data(), points.data(), bytes.size());
    return bytes;
}

ConeMetric traverse(int index, int cluster_id, const std::vector<std::vector<int>> & neighbors, std::vector<uchar> & cluster_map, const std::vector<Point> & points, bool orange = false) {
    if (cluster_map[index] == 0 && neighbors.size() > 15) {
        ConeMetric cone = ConeMetric(points[index]);
        //if (orange)
            //cout << "x: " << points[index][0] << ", y:" << points[index][1] << ", z:" << points[index][2] << endl;
        cluster_map[index] = cluster_id;
        for (auto neighbor : neighbors[index]) {
            auto result = traverse(neighbor, cluster_id, neighbors, cluster_map, points, orange);
            cone += result;
        }
        return cone;
    } else {
        cluster_map[cluster_id] = 255;
        return ConeMetric();
    }
}

class ClusterNode
    : public rclcpp::Node {
private:
    using Cluster = eufs_msgs::srv::Cluster;
    using ConeArrayWithCovariance = eufs_msgs::msg::ConeArrayWithCovariance;
    using ConeWithCovariance = eufs_msgs::msg::ConeWithCovariance;
    GLCluster gl_cluster;
    rclcpp::Service<Cluster>::SharedPtr service;
    rclcpp::Publisher<ConeArrayWithCovariance>::SharedPtr cone_array_publisher;
    
    std::vector<uchar> cluster(const std::vector<Point> & points,
			       const std::vector<uchar> & class_map,
			       float epsilon)
    {
	std::vector<uchar> cluster_map(points.size());
        eufs_msgs::msg::ConeArrayWithCovariance cone_array;
        cone_array.header.frame_id = "base_footprint";
        cone_array.header.stamp = this->get_clock()->now();
        
        this->gl_cluster.setEps(epsilon);
        
        int cluster_id = 1;
        
        for (int c = 1; c <= 3; c++) {
            // When we collect high quality points of a particular class, we need to remember
            // their origins in the full array.
            std::vector<int> map_back;
            std::vector<Point> class_points;
            
            for (int i = 0; i < points.size(); i++) {
                float sum = points[i][0] + points[i][1] + points[i][2];
                if (sum != 0 and not std::isnan(sum) and not std::isinf(sum)
                    and (class_map[i] == c or c == 3 && class_map[i] == 4)) {
                    class_points.push_back(points[i]);
                    map_back.push_back(i);
                }
            }
            
            std::vector<std::vector<int>> class_neighbors_map(class_points.size());
            
            this->gl_cluster.setMaximumPoints(class_points.size());
            this->gl_cluster.setInputData((float*)class_points.data(), class_points.size(), false);
            
            for (int i = 0; i < class_points.size(); i += this->gl_cluster.getBatchSize()) {
                this->gl_cluster.generateOutput(i, class_neighbors_map);
            }

            std::vector<uchar> class_cluster_map(class_points.size());

            std::vector<ConeWithCovariance> * class_cone_array; 
            if (c == 1) class_cone_array = &cone_array.blue_cones;
            else if (c == 2) class_cone_array = &cone_array.yellow_cones;
            else class_cone_array = &cone_array.orange_cones;
            
            for (int i = 0; i < class_points.size(); i++) {
                auto cone = traverse(i, cluster_id, class_neighbors_map, class_cluster_map, class_points, true);
                if (!cone.empty()) {
                    /*cone.area * (1 + this->get_parameter("depth_allowance").get_parameter_value().get<float>())
                        < this->get_parameter("area_threshold").get_parameter_value().get<int>();*/
                    if (cone.area > 5) {
                        Point center = div_point(cone.sum, cone.area);
                        ConeWithCovariance cone;
                        cone.point.x = -center[2] / 1000.0;
                        cone.point.y = center[0] / 1000.0;
                        class_cone_array->push_back(cone);
                        cout << "Found a cone {\n x:" << cone.point.x << ",\n z:" << cone.point.z << "\n} of class " << c << endl;
                    }
                    
                    cluster_id++;
                }
            }

            
            for (int i = 0; i < class_points.size(); i++) {
                if (class_cluster_map[i] != 255) {
                    cluster_map[map_back[i]] = class_cluster_map[i];
                }
            }
        }

        cout << "Found " << cone_array.blue_cones.size() << " blue cones" << endl;
        cout << "Found " << cone_array.yellow_cones.size() << " yellow cones" << endl;
        cout << "Found " << cone_array.orange_cones.size() << " orange cones" << endl;

        this->cone_array_publisher->publish(cone_array);

        return cluster_map;
    }

    void cluster_callback(Cluster::Request::SharedPtr request, Cluster::Response::SharedPtr response) {
	auto & image = response->clusters;
	image.header.frame_id = request->points.header.frame_id;
	image.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
	image.height = request->points.height;
	image.width = request->points.width;
	image.data = cluster(bytes_to_points(request->points.data),
			     request->segmentation_mask.data,
			     this->get_parameter("eps").get_parameter_value().get<float>());
    }
    
public:
    ClusterNode()
	: rclcpp::Node("cluster_node") {
	using namespace std::placeholders;
	this->declare_parameter<float>("eps", 500);
        this->declare_parameter<float>("depth_allowance", 0.01); // Expected decrease in cone size per centimeter
        this->declare_parameter<int>("area_threshold", 10);
        this->declare_parameter<int>("batch_size", 100);
        this->gl_cluster.setBatchSize(this->get_parameter("batch_size").get_parameter_value().get<int>());
        this->gl_cluster.setEps(this->get_parameter("eps").get_parameter_value().get<float>());
	this->service = this->create_service<Cluster>("cluster",
						      std::bind(&ClusterNode::cluster_callback, this, _1, _2));
        this->cone_array_publisher = this->create_publisher<ConeArrayWithCovariance>("cones", 1);
	RCLCPP_INFO(this->get_logger(), "Initialised c++ cluster node");
    }
};

//#define FT_TEST

int
main(int argc, char *argv[])
{
    
    initializeEGL();
    
    #ifndef FT_TEST
    
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ClusterNode>());
    rclcpp::shutdown();
    
    #else
    
    std::vector<float> data {
        5913, 49, 4913, // 0
        900, 39, 500, // 1
        100, 3., 50, // 2
        0.0, 0.0, 0.0, // 3
        1.0, 1.0, 1.0, // 4
        -1.0, -1.0, -1.0, // 5
        5.0, 5.0, 5.0, // 6
        6.0, 6.0, 6.0, // 7
        4.0, 4.0, 4.0, // 8
        4.5, 4.0, 5.0, // 9
        40,40,40, // 10
        41,40,40, // 11
        40,41,40 // 12
    };
    
    int n = data.size() / 3;
    
    GLCluster cluster;
    cluster.setBatchSize(13125);
    cluster.setEps(5.0);
    cluster.setMaximumPoints(n);
    cluster.setInputData(data.data(), n, false);

    std::vector<std::vector<int>> neighborMap(n);

    for (int i = 0; i < n; i+=cluster.getBatchSize())
        cluster.generateOutput(i, neighborMap);
    
    std::vector<uchar> clusterMap(neighborMap.size());

    int id = 1;
    
    for (int i = 0; i < neighborMap.size(); i++) {
        auto cone_here = traverse(i, id, neighborMap, clusterMap, 1);
        if (cone_here) {
            id++;
        }
    }

    //cout << "Clusters" << endl;
    for (auto elem : clusterMap) {
        //cout << "- " << (int) elem << endl;
    }

    cout << "Done" << endl;
    #endif
}
