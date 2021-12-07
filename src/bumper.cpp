/* headers //{ */

// clang: MatousFormat
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/range.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/circular_buffer.hpp>

#include <fog_msgs/msg/obstacle_sectors.hpp>
#include "geometry/cyclic.hpp"

#include <thread>
#include <atomic>
#include <mutex>
#include <iomanip>

//}

using namespace std::placeholders;

namespace fog_bumper
{

using ObstacleSectors = fog_msgs::msg::ObstacleSectors;
using radians         = mrs_lib::geometry::radians;
using angle_range_t   = std::pair<double, double>;
using hist_t          = std::vector<float>;

// --------------------------------------------------------------
// |                   Bumper class definition                  |
// --------------------------------------------------------------

/* class Bumper //{ */
class Bumper : public rclcpp::Node {
public:
  Bumper(rclcpp::NodeOptions options);

private:
  // --------------------------------------------------------------
  // |                ROS-related member variables                |
  // --------------------------------------------------------------

  /* Parameters, loaded from ROS //{ */
  std::string m_frame_id;
  int         m_median_filter_size;

  int m_lidar2d_filter_size;

  //}

  /* ROS related variables (subscribers, timers etc.) //{ */
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr m_lidar2d_sub;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr     m_lidar1d_down_sub;

  rclcpp::CallbackGroup::SharedPtr m_callback_group;
  rclcpp::TimerBase::SharedPtr     m_main_timer;

  std::shared_ptr<tf2_ros::Buffer>            m_tf_buffer;
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener_ptr;

  rclcpp::Publisher<ObstacleSectors>::SharedPtr m_obstacles_pub;

  std::unique_ptr<sensor_msgs::msg::LaserScan> m_lidar2d_msg;
  std::unique_ptr<sensor_msgs::msg::Range>     m_lidar1d_down_msg;

  //}

private:
  // --------------------------------------------------------------
  // |                   Other member variables                   |
  // --------------------------------------------------------------

  /* Misc. member variables //{ */
  uint32_t                                    m_n_horizontal_sectors;
  uint32_t                                    m_bottom_sector_idx;
  uint32_t                                    m_top_sector_idx;
  std::vector<angle_range_t>                  m_horizontal_sector_ranges;
  std::vector<boost::circular_buffer<double>> m_sector_filters;
  uint32_t                                    m_n_total_sectors;
  double                                      m_vertical_fov;

  bool m_sectors_initialized;

  bool   m_lidar2d_offset_initialized;
  double m_lidar2d_offset;

  std::mutex m_mutex;
  //}

  // --------------------------------------------------------------
  // |                     Callback functions                     |
  // --------------------------------------------------------------
  //
  // subscriber callbacks
  void lidar2d_callback(const sensor_msgs::msg::LaserScan::UniquePtr msg);
  void lidar1d_down_callback(const sensor_msgs::msg::Range::UniquePtr msg);

  // --------------------------------------------------------------
  // |                 Obstacle detection methods                 |
  // --------------------------------------------------------------

  std::vector<double> find_obstacles_lidar1d(const sensor_msgs::msg::Range& msg, const int sector);
  std::vector<double> find_obstacles_lidar2d(const sensor_msgs::msg::LaserScan& scan_msg);

  // --------------------------------------------------------------
  // |                       Helper methods                       |
  // --------------------------------------------------------------

  void main_routine(void);

  void initialize_sectors(int n_horizontal_sectors, double vfov);

  std::vector<angle_range_t> initialize_ranges(uint32_t m_n_horizontal_sectors);
  angle_range_t              get_horizontal_sector_angle_interval(unsigned sector_it);
  bool                       angle_in_range(double angle, const angle_range_t& angle_range);
  void                       update_filter_sizes();

  void initialize_lidar2d_offset(const sensor_msgs::msg::LaserScan& lidar2d_msg);

  template <typename T>
  void print_median_buffer(boost::circular_buffer<T> fil, T median, unsigned n);

  template <typename T>
  std::vector<T> filter_sectors(const std::vector<T>& sectors);

  template <class T>
  bool parse_param(const std::string& param_name, T& param_dest);
};
//}

// --------------------------------------------------------------
// |                         Constructor                        |
// --------------------------------------------------------------

/* Bumper(..) method //{ */
Bumper::Bumper(rclcpp::NodeOptions options) : Node("Bumper", options) {


  RCLCPP_INFO(this->get_logger(), "[%s]: Initializing...", this->get_name());
  RCLCPP_INFO(this->get_logger(), "[%s]: -------------- Loading parameters --------------", this->get_name());

  int n_horizontal_sectors;
  int update_rate;

  /* parse params from config file //{ */
  bool loaded_successfully = true;

  loaded_successfully &= parse_param("update_rate", update_rate);
  loaded_successfully &= parse_param("frame_id", m_frame_id);
  loaded_successfully &= parse_param("median_filter_size", m_median_filter_size);
  loaded_successfully &= parse_param("lidar2d.filter_size", m_lidar2d_filter_size);
  loaded_successfully &= parse_param("n_horizontal_sectors", n_horizontal_sectors);

  if (!loaded_successfully) {
    const std::string str = "Could not load all non-optional parameters. Shutting down.";
    RCLCPP_ERROR(this->get_logger(), str.c_str());
    rclcpp::shutdown();
    return;
  }

  //}

  initialize_sectors(n_horizontal_sectors, 0.00872665); // 0.5 degree

  m_lidar2d_offset_initialized = false;
  m_lidar2d_msg.reset();
  m_lidar1d_down_msg.reset();

  m_callback_group = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto sub_opt = rclcpp::SubscriptionOptions();
  sub_opt.callback_group = m_callback_group;

  // Initialize subscribers
  m_lidar2d_sub =
      this->create_subscription<sensor_msgs::msg::LaserScan>("lidar2d_in", rclcpp::SystemDefaultsQoS(), std::bind(&Bumper::lidar2d_callback, this, _1), sub_opt);
  m_lidar1d_down_sub =
      this->create_subscription<sensor_msgs::msg::Range>("lidar1d_down_in", rclcpp::SystemDefaultsQoS(), std::bind(&Bumper::lidar1d_down_callback, this, _1), sub_opt);

  // Initialize publishers
  m_obstacles_pub = this->create_publisher<ObstacleSectors>("obstacle_sectors_out", 10);

  /* initialize tf_listener //{ */

  m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  m_tf_buffer->setUsingDedicatedThread(true);
  m_tf_listener_ptr = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer, this, false);

  //}

  m_main_timer = this->create_wall_timer(std::chrono::duration<double>(1.0 / update_rate), std::bind(&Bumper::main_routine, this), m_callback_group);

  std::cout << "----------------------------------------------------------" << std::endl;
}
//}

// --------------------------------------------------------------
// |                          Callbacks                         |
// --------------------------------------------------------------

/* lidar2d_callback //{ */
void Bumper::lidar2d_callback(const sensor_msgs::msg::LaserScan::UniquePtr msg) {
  std::scoped_lock lock(m_mutex);
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting lidar2d msgs...", this->get_name());
  m_lidar2d_msg = std::unique_ptr<sensor_msgs::msg::LaserScan>{new sensor_msgs::msg::LaserScan{*msg}};
}
//}

/* lidar1d_down_callback //{ */
void Bumper::lidar1d_down_callback(const sensor_msgs::msg::Range::UniquePtr msg) {
  std::scoped_lock lock(m_mutex);
  RCLCPP_INFO_ONCE(this->get_logger(), "[%s]: Getting lidar1d_down msgs...", this->get_name());
  m_lidar1d_down_msg = std::unique_ptr<sensor_msgs::msg::Range>{new sensor_msgs::msg::Range{*msg}};
}
//}


// --------------------------------------------------------------
// |                          Main loop                         |
// --------------------------------------------------------------

/* main_routine //{ */
void Bumper::main_routine() {
  static rclcpp::Time last_processed_msg = this->get_clock()->now();

  bool                        has_lidar2d_new_msg      = false;
  bool                        has_lidar1d_down_new_msg = false;
  sensor_msgs::msg::LaserScan lidar2d_msg;
  sensor_msgs::msg::Range     lidar1d_down_msg;

  {
    std::scoped_lock lock(m_mutex);
    if (m_lidar2d_msg != nullptr) {
      has_lidar2d_new_msg = true;
      lidar2d_msg         = *m_lidar2d_msg;
      m_lidar2d_msg.reset();
    }

    if (m_lidar1d_down_msg != nullptr) {
      has_lidar1d_down_new_msg = true;
      lidar1d_down_msg         = *m_lidar1d_down_msg;
      m_lidar1d_down_msg.reset();
    }
  }

  /* Initialize horizontal angle offset of 2D lidar from a new message //{ */
  if (!m_lidar2d_offset_initialized && has_lidar2d_new_msg) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[Bumper]: Initializing 2D lidar horizontal angle offset");

    initialize_lidar2d_offset(lidar2d_msg);

    if (m_lidar2d_offset_initialized) {
      RCLCPP_INFO(this->get_logger(), "[Bumper]: 2D lidar horizontal angle offset: %.2f", m_lidar2d_offset);
    } else {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[Bumper]: 2D lidar horizontal angle offset initialization failed, will retry.");
    }
  }
  //}

  if (m_sectors_initialized) {
    std::vector<std::tuple<int, std::vector<double>, rclcpp::Time>> sensors_sectors;
    std::vector<std::string>                                        sensors_topics;

    /* process any new messages from sensors //{ */

    // Check data from the horizontal 2D lidar
    if (m_lidar2d_offset_initialized && has_lidar2d_new_msg) {
      std::vector<double> obstacle_sectors = find_obstacles_lidar2d(lidar2d_msg);
      sensors_sectors.push_back({ObstacleSectors::SENSOR_LIDAR2D, obstacle_sectors, lidar2d_msg.header.stamp});
      sensors_topics.push_back(m_lidar2d_sub->get_topic_name());
    }

    // Check data from the down-facing lidar
    if (has_lidar1d_down_new_msg) {
      std::vector<double> obstacle_sectors = find_obstacles_lidar1d(lidar1d_down_msg, m_bottom_sector_idx);
      sensors_sectors.push_back({ObstacleSectors::SENSOR_LIDAR1D, obstacle_sectors, lidar1d_down_msg.header.stamp});
      sensors_topics.push_back(m_lidar1d_down_sub->get_topic_name());
    }

    //}

    std::vector<double> res_obstacles(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);
    std::vector<int8_t> res_sensors(m_n_total_sectors, ObstacleSectors::SENSOR_NONE);
    rclcpp::Time        res_stamp = this->get_clock()->now();

    /* put the resuls from different sensors together //{ */

    for (const auto& sensor_sectors : sensors_sectors) {
      const auto [sensor, sectors, stamp] = sensor_sectors;
      for (size_t sect_it = 0; sect_it < m_n_total_sectors; sect_it++) {
        const auto obstacle_dist = sectors.at(sect_it);
        // check if an obstacle was detected (*obstacle_sure*)
        const auto obstacle_unknown = obstacle_dist == ObstacleSectors::OBSTACLE_NO_DATA;
        auto&      cur_value        = res_obstacles.at(sect_it);
        auto       cur_unknown      = cur_value == ObstacleSectors::OBSTACLE_NO_DATA;
        auto&      cur_sensor       = res_sensors.at(sect_it);
        // If the previous obstacle information in this sector is unknown or a closer
        // obstacle was detected by this sensor, update the information.
        if (!obstacle_unknown && (cur_value > obstacle_dist || cur_unknown)) {
          cur_value  = obstacle_dist;
          cur_sensor = sensor;
          if (res_stamp > stamp) {
            res_stamp = stamp;
          }
        }
      }
    }

    //}

    // filter the obstacles using a median filter
    res_obstacles = filter_sectors(res_obstacles);

    /* Prepare and publish the ObstacleSectors message to be published //{ */
    {
      ObstacleSectors obst_msg;
      obst_msg.header.frame_id      = m_frame_id;
      obst_msg.header.stamp         = res_stamp;
      obst_msg.n_horizontal_sectors = m_n_horizontal_sectors;
      obst_msg.sectors_vertical_fov = m_vertical_fov;
      obst_msg.sectors              = res_obstacles;
      obst_msg.sector_sensors       = res_sensors;
      m_obstacles_pub->publish(obst_msg);
    }
    //}

    /* print out some info to the console //{ */

    {
      std::vector<std::string> used_sensors;
      for (const auto& el : sensors_sectors) {
        const auto sensor = std::get<0>(el);
        switch (sensor) {
          case ObstacleSectors::SENSOR_NONE:
            used_sensors.push_back("\033[1;31minvalid \033[0m");
            break;
          case ObstacleSectors::SENSOR_DEPTH:
            used_sensors.push_back("depthmap");
            break;
          case ObstacleSectors::SENSOR_LIDAR1D:
            used_sensors.push_back("LiDAR 1D");
            break;
          case ObstacleSectors::SENSOR_LIDAR2D:
            used_sensors.push_back("LiDAR 2D");
            break;
          case ObstacleSectors::SENSOR_LIDAR3D:
            used_sensors.push_back("LiDAR 3D");
            break;
        }
      }
      std::stringstream ss;
      for (size_t it = 0; it < used_sensors.size(); it++) {
        ss << std::endl << "\t[" << used_sensors.at(it) << "] at topic \"" << sensors_topics.at(it) << "\"";
      }
      if (!used_sensors.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[Bumper]: Updating bumper using sensors: %s", ss.str().c_str());
        last_processed_msg = res_stamp;
      } else if ((this->get_clock()->now() - last_processed_msg).nanoseconds() / 1e9 >= 2.0){
        RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "[Bumper]: No new data");
      }
    }

    //}
  }
}
//}

// --------------------------------------------------------------
// |                 Obstacle detection methods                 |
// --------------------------------------------------------------

/* find_obstacles_lidar1d() method //{ */
std::vector<double> Bumper::find_obstacles_lidar1d(const sensor_msgs::msg::Range& msg, const int sector) {
  std::vector<double> ret(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);

  double obstacle_dist = msg.range;
  if (obstacle_dist <= msg.min_range || obstacle_dist >= msg.max_range) {
    obstacle_dist = ObstacleSectors::OBSTACLE_NOT_DETECTED;
  }
  ret.at(sector) = obstacle_dist;

  return ret;
}
//}

/* find_obstacles_lidar2d() method //{ */
std::vector<double> Bumper::find_obstacles_lidar2d(const sensor_msgs::msg::LaserScan& scan_msg) {
  std::vector<double> ret(m_n_total_sectors, ObstacleSectors::OBSTACLE_NO_DATA);
  const auto          buffer_length = m_lidar2d_filter_size;
  // check minimal obstacle distance for each horizontal sector
  for (size_t it = 0; it < m_n_horizontal_sectors; it++) {
    const auto& cur_angle_range = m_horizontal_sector_ranges.at(it);
    double      min_range       = std::numeric_limits<double>::max();
    // buffer of the last *buffer_length* measurements
    boost::circular_buffer<double> buffer(buffer_length);
    for (unsigned ray_it = 0; ray_it < scan_msg.ranges.size(); ray_it++) {
      const double ray_range = scan_msg.ranges.at(ray_it) + buffer_length / 2 * scan_msg.angle_increment;
      const double ray_angle = scan_msg.angle_min + ray_it * scan_msg.angle_increment + m_lidar2d_offset - buffer_length / 2 * scan_msg.angle_increment;
      // check if the ray is in the current horizontal sector
      if (angle_in_range(ray_angle, cur_angle_range)) {
        buffer.push_back(ray_range);
        /* const double cur_max_range = buffer_max(buffer); */
        const double cur_max_range = *std::max_element(std::begin(buffer), std::end(buffer));
        // If the buffer has *buffer_length* measurements and maximal distance
        // in the buffer is lower than *min_range*, update *min_range*.
        // This should filter out solitary false detections of the laser rangefinder,
        // which would otherwise trigger the repulsion failsafe mechanism.
        if (buffer.full() && cur_max_range < min_range) {
          min_range = cur_max_range;
        }
      }
    }
    if (min_range < scan_msg.range_min || min_range > scan_msg.range_max) {
      min_range = ObstacleSectors::OBSTACLE_NOT_DETECTED;
    }
    ret.at(it) = min_range;
  }
  return ret;
}
//}

// --------------------------------------------------------------
// |                       Helper methods                       |
// --------------------------------------------------------------

/* initialize_sectors() method //{ */
// initializes some helper variables related to the sectors
void Bumper::initialize_sectors(int n_horizontal_sectors, double vfov) {
  m_n_horizontal_sectors     = n_horizontal_sectors;
  m_bottom_sector_idx        = n_horizontal_sectors;
  m_top_sector_idx           = n_horizontal_sectors + 1;
  m_horizontal_sector_ranges = initialize_ranges(n_horizontal_sectors);
  m_n_total_sectors          = n_horizontal_sectors + 2;
  m_vertical_fov             = vfov;
  update_filter_sizes();
  m_sectors_initialized = true;
}
//}

/* initialize_ranges() method //{ */
// initializes angle ranges of horizontal sectors
std::vector<angle_range_t> Bumper::initialize_ranges(uint32_t m_n_horizontal_sectors) {
  std::vector<angle_range_t> ret;
  ret.reserve(m_n_horizontal_sectors);
  for (unsigned sector_it = 0; sector_it < m_n_horizontal_sectors; sector_it++)
    ret.push_back(get_horizontal_sector_angle_interval(sector_it));
  return ret;
}
//}

/* get_horizontal_sector_angle_interval() method //{ */
// helper method for initialization of angle ranges of horizontal sectors
angle_range_t Bumper::get_horizontal_sector_angle_interval(unsigned sector_it) {
  assert(sector_it < m_n_horizontal_sectors);
  const double angle_step  = 2.0 * M_PI / m_n_horizontal_sectors;
  const double angle_start = radians::wrap(sector_it * angle_step - angle_step / 2.0);
  const double angle_end   = radians::wrap(angle_start + angle_step);
  return {angle_start, angle_end};
}
//}

/* angle_in_range() method //{ */
bool Bumper::angle_in_range(double angle, const angle_range_t& angle_range) {
  angle         = radians::wrap(angle);
  bool in_range = angle > angle_range.first && angle < angle_range.second;
  // corner-case for the first sector (which would have angle_range.first < 0.0, but it is normalized as angle_range.first + 2.0*M_PI)
  if (angle_range.first > angle_range.second) {
    in_range = angle < angle_range.second || angle > angle_range.first;
  }
  return in_range;
}
//}

/* update_filter_sizes() method //{ */
void Bumper::update_filter_sizes() {
  const boost::circular_buffer<double> init_bfr(m_median_filter_size, ObstacleSectors::OBSTACLE_NO_DATA);
  m_sector_filters.resize(m_n_total_sectors, init_bfr);
  for (auto& fil : m_sector_filters)
    fil.rset_capacity(m_median_filter_size);
}
//}

// | ----- helper methods for 2D lidar obstacle detection ----- |

/* initialize_lidar2d_offset() method //{ */
// initializes an angle offset of a 2D LiDAR using transforms
void Bumper::initialize_lidar2d_offset(const sensor_msgs::msg::LaserScan& lidar2d_msg) {
  geometry_msgs::msg::Vector3Stamped x_lidar;
  x_lidar.header   = lidar2d_msg.header;
  x_lidar.vector.x = 1.0;
  x_lidar.vector.y = x_lidar.vector.z = 0.0;
  geometry_msgs::msg::Vector3Stamped x_fcu;

  try {
    x_fcu = m_tf_buffer->transform(x_lidar, m_frame_id);
  }
  catch (...) {
    return;
  }

  m_lidar2d_offset             = std::atan2(x_fcu.vector.y, x_fcu.vector.x);
  m_lidar2d_offset_initialized = true;
}
//}

// | ----------- helper methods for median filtering ---------- |

/* get_median() method //{ */
template <typename T>
static T get_median(const boost::circular_buffer<T>& buffer) {
  // copy the buffer to a helper vector
  std::vector<T> data;
  data.reserve(buffer.size());
  for (const auto& el : buffer)
    data.push_back(el);
  // sort the vector up to the nth element
  std::nth_element(std::begin(data), std::begin(data) + data.size() / 2, std::end(data));
  // get the nth element (that's the median)
  const T median = *(std::begin(data) + data.size() / 2);
  return median;
}
//}

/* print_median_buffer() method //{ */
template <typename T>
void Bumper::print_median_buffer(boost::circular_buffer<T> fil, T median, unsigned n) {
  std::cout << "Buffer #" << n << ": " << median << std::endl;
  for (unsigned it = 0; it < fil.size(); it++)
    std::cout << it << "\t";
  std::cout << std::endl;
  for (const auto& v : fil)
    std::cout << std::fixed << std::setprecision(3) << v << "\t";
  std::cout << std::endl;
}
//}

/* filter_sectors() method //{ */
// applies one step of a median filter on each sector and returns the result
template <typename T>
std::vector<T> Bumper::filter_sectors(const std::vector<T>& sectors) {
  assert(sectors.size() == m_sector_filters.size());
  std::vector<T> ret;
  ret.reserve(sectors.size());
  for (unsigned it = 0; it < sectors.size(); it++) {
    const T sec = sectors.at(it);
    auto&   fil = m_sector_filters.at(it);
    fil.push_back(sec);
    const auto median = get_median(fil);
    if (std::isinf(median)) {
      RCLCPP_WARN(this->get_logger(), "[Bumper]: median is inf...");
    }
    ret.push_back(median);
#ifdef DEBUG_MEDIAN_FILTER
    print_median_buffer(fil, median, it);
#endif
  }
  return ret;
}
//}

// | --------- helper methods for loading parameters --------- |

/* parse_param //{ */
template <class T>
bool Bumper::parse_param(const std::string& param_name, T& param_dest) {
  this->declare_parameter(param_name);
  if (!this->get_parameter(param_name, param_dest)) {
    RCLCPP_ERROR(this->get_logger(), "[%s]: Could not load param '%s'", this->get_name(), param_name.c_str());
    return false;
  } else {
    RCLCPP_INFO_STREAM(this->get_logger(), "[" << this->get_name() << "]: Loaded '" << param_name << "' = '" << param_dest << "'");
  }
  return true;
}

template bool Bumper::parse_param<int>(const std::string& param_name, int& param_dest);
template bool Bumper::parse_param<double>(const std::string& param_name, double& param_dest);
template bool Bumper::parse_param<float>(const std::string& param_name, float& param_dest);
template bool Bumper::parse_param<std::string>(const std::string& param_name, std::string& param_dest);
template bool Bumper::parse_param<bool>(const std::string& param_name, bool& param_dest);
template bool Bumper::parse_param<unsigned int>(const std::string& param_name, unsigned int& param_dest);
//}
//
}  // namespace fog_bumper

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(fog_bumper::Bumper)
