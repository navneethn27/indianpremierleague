#include <memory>
#include <string>
#include <Eigen/Dense>
#include "extended_map_server/saver_node.hpp"


MapSaverNode::MapSaverNode() : 
          Node("map_saver_node"),
          db_(client["gridmap_db"]),
          collection_(db_["location_a"]),
          gps_msg_(),
          received_map_(false),
          received_gps_(false),
          use_gps_(true),
          map_server_mode_(true)
{
  this->declare_parameter("map_file_name", "map_output_file");
  this->declare_parameter("image_format", "pgm");
  this->declare_parameter("free_thresh", 0.25);
  this->declare_parameter("occupied_thresh", 0.65);
  this->declare_parameter("max_height", 500.0);
  this->declare_parameter("min_height", 500.0);
  this->declare_parameter("costmap_topic_in", "/lidar_3d_gridmap");
  this->declare_parameter("gps_topic_in", "vectornav/GPS");
  this->declare_parameter("use_gps", use_gps_);
  this->declare_parameter("db_data_path", db_data_path);
  this->declare_parameter("mission_id", mission_id);
  this->declare_parameter("max_pairwise_distance", 100.0);
  this->declare_parameter("matches_threshold", 10.0);

  map_name_ = this->get_parameter("costmap_topic_in").as_string();
  gps_name_ = this->get_parameter("gps_topic_in").as_string();
  use_gps_ = this->get_parameter("use_gps").as_bool();
  db_data_path = this->get_parameter("db_data_path").as_string();
  mission_id = this->get_parameter("mission_id").as_string();
  collection_ = db_[mission_id];
  
  save_parameters.map_file_name = this->get_parameter("map_file_name").as_string();
  save_parameters.image_format = this->get_parameter("image_format").as_string();
  save_parameters.free_thresh = this->get_parameter("free_thresh").as_double();
  save_parameters.occupied_thresh = this->get_parameter("occupied_thresh").as_double();
    // For grid_map config
  save_parameters.max_height = this->get_parameter("max_height").as_double();
  save_parameters.min_height = this->get_parameter("min_height").as_double();

  save_parameters.mode = extended_map_server::MapMode::Raw;

  // map_publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map_out", 1);
  grid_map_pub_ = create_publisher<grid_map_msgs::msg::GridMap>(
    std::string("grid_map_out"),
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  server_ = this->create_service<extended_mapping_msgs::srv::LoadMapNearby>("mongo/query_nearby_map",
      std::bind(&MapSaverNode::loadMapCallback, this, std::placeholders::_1,
      std::placeholders::_2));

  merge_posegraph_server_ = this->create_service<std_srvs::srv::SetBool>("mongo/merge_posegraph",
      std::bind(&MapSaverNode::mergePoseGraph, this, std::placeholders::_1,
      std::placeholders::_2));
      
  enable_storage_server_ = this->create_service<std_srvs::srv::SetBool>("mongo/toggle_storage",
      std::bind(&MapSaverNode::ToggleStorage, this, std::placeholders::_1,
      std::placeholders::_2));

  enable_retrival_server_ = this->create_service<std_srvs::srv::SetBool>("mongo/toggle_retrival",
      std::bind(&MapSaverNode::ToggleRetrival, this, std::placeholders::_1,
      std::placeholders::_2));

  // tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  // tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  
  
  mongocxx::options::index index_options{};
  index_options.unique(true); // Adjust if needed
  collection_.create_index(
      bsoncxx::builder::stream::document{} << "location" << "2dsphere" << bsoncxx::builder::stream::finalize,
      index_options
  );

  auto mapCallback =
    [this](const grid_map_msgs::msg::GridMap::SharedPtr msg) -> void
    {
        

        // std::string serialized_msg_str(buffer_begin, buffer_end);
        // std::cout << serialized_msg_str << std::endl;

      if(!use_gps_){
        received_gps_ = true;
        gps_msg_.longitude = 0.0;
        gps_msg_.latitude = 0.0;
      }
        if(received_gps_ && toggle_storage_ && save_map_flag_){
          std::cout << "storing gridmap map at dist: " << dist_travelled << std::endl;
       
          rclcpp::SerializedMessage serialized_msg;
          rclcpp::Serialization<grid_map_msgs::msg::GridMap> mapSerializer;
          const void* msg_ptr = reinterpret_cast<const void*>(msg.get());
          mapSerializer.serialize_message(msg_ptr, &serialized_msg);
          size_t serialized_msg_msg_size = serialized_msg.size();
          const auto serialized_msg_buffer = serialized_msg.get_rcl_serialized_message().buffer;
          std::cout << "cpp size: " << serialized_msg_msg_size << " bytes, buffer data: "<< serialized_msg_buffer << std::endl;

          bsoncxx::types::b_binary data{
                                          bsoncxx::binary_sub_type::k_binary, 
                                          static_cast<std::uint32_t>(serialized_msg_msg_size), 
                                          serialized_msg_buffer
                                        };

          mongocxx::collection collection = collection_;

          auto builder = bsoncxx::builder::stream::document{};
          bsoncxx::document::value doc_to_add = 
            builder << "payload" << data
                    << "location" << bsoncxx::builder::stream::open_document
                      << "type" << "Point"
                        << "coordinates" << bsoncxx::builder::stream::open_array
                          << gps_msg_.longitude << gps_msg_.latitude 
                        << bsoncxx::builder::stream::close_array
                    << bsoncxx::builder::stream::close_document
                    << bsoncxx::builder::stream::finalize;
          collection.insert_one(doc_to_add.view());

          // extended_map_server::saveMapToFile(*msg, gps_msg_, save_parameters);
          received_gps_ = false;
          save_map_flag_ = false;
          dist_travelled = 0.0;
        }
    };

  map_sub_ = this->create_subscription<grid_map_msgs::msg::GridMap>(
    map_name_, rclcpp::QoS(1), mapCallback);
  
  auto gpsCallback =
    [this](const sensor_msgs::msg::NavSatFix::SharedPtr msg) -> void
    {
        gps_msg_ = *msg;
        received_gps_ = true;
        if(!gps_init){
          lat_anchor = gps_msg_.latitude;
          long_anchor = gps_msg_.longitude;
          std::cout << "gps_init: " << std::endl;
          gps_init = true;
        }
        
        std::cout << "dist_travelled: " << dist_travelled << std::endl;
        if(dist_travelled >= travel_threshold){
          lat_anchor = gps_msg_.latitude;
          long_anchor = gps_msg_.longitude;
          save_map_flag_ = true;
        }

        dist_travelled = distance(lat_anchor, long_anchor, gps_msg_.latitude , gps_msg_.longitude);

        if(toggle_retrival_){
          mongocxx::collection collection = collection_;
          bsoncxx::builder::stream::document query_builder;
            query_builder << "location" << bsoncxx::builder::stream::open_document
                            << "$nearSphere" << bsoncxx::builder::stream::open_document
                              << "$geometry" << bsoncxx::builder::stream::open_document
                                << "type" << "Point"
                                  << "coordinates" << bsoncxx::builder::stream::open_array
                                  << gps_msg_.longitude << gps_msg_.latitude // The order is important: longitude first, then latitude
                                << bsoncxx::builder::stream::close_array
                              << bsoncxx::builder::stream::close_document
                              << "$minDistance" << 0.0
                              << "$maxDistance" << 10.0
                            << bsoncxx::builder::stream::close_document
                          << bsoncxx::builder::stream::close_document;
            

            // Create options for the query (if needed)
            mongocxx::options::find find_options;
            find_options.projection(bsoncxx::builder::stream::document{} << "_id" << 0 << bsoncxx::builder::stream::finalize);

            // Execute the nearby query
            mongocxx::cursor cursor = collection_.find(query_builder.view());

            // Iterate over the result cursor and print documents
            for (auto&& docu : cursor) {
                auto json_document = bsoncxx::to_json(docu);
                auto binaryData = docu["payload"].get_binary();
                grid_map_msgs::msg::GridMap mapMsg;
                rclcpp::Serialization<grid_map_msgs::msg::GridMap> mapSerializer;
                rclcpp::SerializedMessage new_msg(binaryData.size); 
                memcpy(new_msg.get_rcl_serialized_message().buffer, binaryData.bytes, binaryData.size);
                new_msg.get_rcl_serialized_message().buffer_length = binaryData.size;
                mapSerializer.deserialize_message(&new_msg, &mapMsg);
                grid_map_pub_->publish(mapMsg);
                break;
            }

        }
    };

  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    gps_name_, rclcpp::SensorDataQoS(), gpsCallback);


  auto novatelGpsCallback =
    [this](const novatel_gps_msgs::msg::Inspva::SharedPtr msg) -> void
    {
        gps_msg_.longitude = msg->longitude;
        gps_msg_.latitude = msg->latitude;
        received_gps_ = true;
        if(!map_server_mode_){
          mongocxx::collection collection = collection_;
          bsoncxx::builder::stream::document query_builder;
            query_builder << "location" << bsoncxx::builder::stream::open_document
                          << "$near" << bsoncxx::builder::stream::open_document
                          << "$geometry" << bsoncxx::builder::stream::open_document
                          << "type" << "Point"
                          << "coordinates" << bsoncxx::builder::stream::open_array
                          << gps_msg_.longitude << gps_msg_.latitude // The order is important: longitude first, then latitude
                          << bsoncxx::builder::stream::close_array
                          << bsoncxx::builder::stream::close_document
                          << "$maxDistance" << 1.0
                          << bsoncxx::builder::stream::close_document
                          << bsoncxx::builder::stream::close_document;

            // Create options for the query (if needed)
            mongocxx::options::find find_options;
            find_options.projection(bsoncxx::builder::stream::document{} << "_id" << 0 << bsoncxx::builder::stream::finalize);

            // Execute the nearby query
            mongocxx::cursor cursor = collection_.find(query_builder.view());

            // Iterate over the result cursor and print documents
            for (auto&& docu : cursor) {
                std::cout << bsoncxx::to_json(docu) << std::endl;
                auto json_document = bsoncxx::to_json(docu);
                std::string metadata = docu["metadata"].get_string().value.to_string();
                auto map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
                extended_map_server::loadMapFromYaml(metadata, *map_msg);
                grid_map::GridMap grid_map_to_fill({"elevation"});
                grid_map::GridMapRosConverter::fromOccupancyGrid(*map_msg, "occupancy", grid_map_to_fill);
                auto msg_grid_map = *grid_map::GridMapRosConverter::toMessage(grid_map_to_fill);
                grid_map_pub_->publish(msg_grid_map);
                break;
            }
        }

        
    };

  novatel_gps_sub_ = this->create_subscription<novatel_gps_msgs::msg::Inspva>(
    "gps_novtel", rclcpp::QoS(1), novatelGpsCallback);
}

bool MapSaverNode::loadMapCallback(
  // const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<extended_mapping_msgs::srv::LoadMapNearby::Request> request,
  std::shared_ptr<extended_mapping_msgs::srv::LoadMapNearby::Response> response)
{
  mongocxx::collection collection = collection_;

  bsoncxx::builder::stream::document query_builder;
    query_builder << "location" << bsoncxx::builder::stream::open_document
                  << "$near" << bsoncxx::builder::stream::open_document
                  << "$geometry" << bsoncxx::builder::stream::open_document
                  << "type" << "Point"
                  << "coordinates" << bsoncxx::builder::stream::open_array
                  << request->longitude << request->latitude // The order is important: longitude first, then latitude
                  << bsoncxx::builder::stream::close_array
                  << bsoncxx::builder::stream::close_document
                  << "$maxDistance" << 1.0
                  << bsoncxx::builder::stream::close_document
                  << bsoncxx::builder::stream::close_document;

    // Create options for the query (if needed)
    mongocxx::options::find find_options;
    find_options.projection(bsoncxx::builder::stream::document{} << "_id" << 0 << bsoncxx::builder::stream::finalize);

    // Execute the nearby query
    mongocxx::cursor cursor = collection_.find(query_builder.view());

    // Iterate over the result cursor and print documents
    for (auto&& docu : cursor) {
        // std::cout << bsoncxx::to_json(doc) << std::endl;
        auto json_document = bsoncxx::to_json(docu);
        std::string metadata = docu["metadata"].get_string().value.to_string();
        auto map_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();
        extended_map_server::loadMapFromYaml(metadata, *map_msg);

        grid_map::GridMap grid_map_to_fill({"elevation"});
        grid_map::GridMapRosConverter::fromOccupancyGrid(*map_msg, "occupancy", grid_map_to_fill);
        auto msg_grid_map = *grid_map::GridMapRosConverter::toMessage(grid_map_to_fill);
        grid_map_pub_->publish(msg_grid_map);

        // map_publisher_->publish(*map_msg);
        break;
    }
    
    response->result = 1;
  return true;
}

bool MapSaverNode::ToggleStorage(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  toggle_storage_ = request->data;
  if(toggle_storage_){
    std::cout << "strorage Enabled and retrival Disabled" << std::endl;
    toggle_retrival_ = false;
  } else {
    std::cout << "strorage Disabled" << std::endl;
  }
  response->success = true;

  return true;
}

bool MapSaverNode::ToggleRetrival(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  toggle_retrival_ = request->data;
  if(toggle_retrival_){
    std::cout << "retrival Enabled and strorage Disabled" << std::endl;
    toggle_storage_ = false;
  } else {
    std::cout << "retrival Disabled" << std::endl;
  }
  response->success = true;

  return true;
}

bool MapSaverNode::mergePoseGraph(const std::shared_ptr<std_srvs::srv::SetBool::Request>,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response){
    mongocxx::collection collection = collection_;
    std::cout << "mergePoseGraph" << std::endl;
    bsoncxx::builder::stream::document filter_builder{};
    bsoncxx::document::view filter = filter_builder.view();

    // Define optional query options (if needed)
    mongocxx::options::find options{};
    options.projection(bsoncxx::builder::stream::document{} << "_id" << 0 << bsoncxx::builder::stream::finalize);

    // Execute the query and retrieve "mappath" values
    mongocxx::cursor cursor = collection.find(filter, options);


    const double resolution{0.5};
    double size_x = 1000.0; //m
    double size_y = 1000.0; //m
    double init_pose_x = 0.0;
    double init_pose_y = 0.0;
    bool init{false};
    gridmap_fused_.add("global_map", 100.0);
    // gridmap_fused_.get("global_map").setConstant(1.0);
    // gridmap_fused_.setGeometry(grid_map::Length(size_x, size_y), 
    //                                                   resolution, grid_map::Position(init_pose_x, init_pose_y));
    // gridmap_fused_.setFrameId("map");
    // for (grid_map::GridMapIterator it(gridmap_fused_); !it.isPastEnd(); ++it) {
    //     const auto& index = *it;
    //     gridmap_fused_.at("global_map", index) = 100.0;
    // }
    // auto gridmap_msg = grid_map::GridMapRosConverter::toMessage(gridmap_fused_);
    // grid_map_pub_->publish(std::move(gridmap_msg));
    
    // gridmap_fused_.clear("global_map");

    for (auto&& docu : cursor) {
        // std::cout << bsoncxx::to_json(docu) << std::endl;
        auto json_document = bsoncxx::to_json(docu);
        auto binaryData = docu["payload"].get_binary();
        grid_map_msgs::msg::GridMap mapMsg;
        rclcpp::Serialization<grid_map_msgs::msg::GridMap> mapSerializer;
        rclcpp::SerializedMessage new_msg(binaryData.size); 
        memcpy(new_msg.get_rcl_serialized_message().buffer, binaryData.bytes, binaryData.size);
        new_msg.get_rcl_serialized_message().buffer_length = binaryData.size;
        mapSerializer.deserialize_message(&new_msg, &mapMsg);

        if(!init){
          init_pose_x = mapMsg.info.pose.position.x;
          
          init_pose_y = mapMsg.info.pose.position.y;
          gridmap_fused_.setGeometry(grid_map::Length(size_x, size_y),resolution, grid_map::Position(init_pose_x, init_pose_y));
          gridmap_fused_.setFrameId(mapMsg.header.frame_id);
          // gridmap_fused_.clear("global_map");
          init = true;
        }

        grid_map::GridMap subMap;
        grid_map::Index startIndex;
        grid_map::GridMapRosConverter::fromMessage(mapMsg, subMap);  
              

        gridmap_fused_.getIndex(subMap.getPosition(), startIndex);
    

        subMap.get("raytrace").array() = (subMap.get("raytrace").array() == 100.0).select(std::numeric_limits<double>::quiet_NaN(), subMap.get("raytrace").array());

        
        Eigen::Array2i copyPosition(startIndex);

     
        for(grid_map::SubmapIterator submapIt(gridmap_fused_, startIndex, subMap.getSize()); !submapIt.isPastEnd(); ++submapIt){

          if (std::isnan(gridmap_fused_.at("global_map", *submapIt))) {

                gridmap_fused_.at("global_map", *submapIt) = -100.0;
            }
        }

        auto& elevationLarge = gridmap_fused_.get("global_map");
        const auto& elevationSmall = subMap.get("raytrace");


        elevationLarge.block(copyPosition[0], copyPosition[1], subMap.getSize()[0], subMap.getSize()[1])
        = elevationLarge.block(copyPosition[0], copyPosition[1], subMap.getSize()[0], subMap.getSize()[1])
              .cwiseMax(elevationSmall);


        // auto gridmap_msg = grid_map::GridMapRosConverter::toMessage(gridmap_fused_);
        // grid_map_pub_->publish(std::move(gridmap_msg));  
        // RCLCPP_INFO(get_logger(), "Received and updated grid map at position ");      
        // rclcpp::sleep_for(std::chrono::milliseconds(100));

    }
    // gridmap_fused_.get("global_map").array() = (gridmap_fused_.get("raytrace").array() == 100.0).select(std::numeric_limits<double>::quiet_NaN(), gridmap_fused_.get("raytrace").array());
    gridmap_fused_.get("global_map").array() = (gridmap_fused_.get("global_map").array() == -100.0).select(100.0, gridmap_fused_.get("global_map").array());
    // RCLCPP_INFO(get_logger(), "Received and updated grid map at position ");
    auto gridmap_msg = grid_map::GridMapRosConverter::toMessage(gridmap_fused_);
    grid_map_pub_->publish(std::move(gridmap_msg));  
    RCLCPP_INFO(get_logger(), "Received and updated grid map at position ");      
    rclcpp::sleep_for(std::chrono::milliseconds(100));

    
    // auto gridmap_msg = grid_map::GridMapRosConverter::toMessage(gridmap_fused_);
    // grid_map_pub_->publish(std::move(gridmap_msg));

    response->success = false;
    return response->success;
  }

long double MapSaverNode::toRadians(const long double & degree)
{
    // cmath library in C++ 
    // defines the constant
    // M_PI as the value of
    // pi accurate to 1e-30
    long double one_deg = (M_PI) / 180;
    return (one_deg * degree);
}
 

long double MapSaverNode::distance(long double lat1, long double long1, 
                     long double lat2, long double long2)
{
    lat1 = toRadians(lat1);
    long1 = toRadians(long1);
    lat2 = toRadians(lat2);
    long2 = toRadians(long2);
     
    // Haversine Formula
    long double dlong = long2 - long1;
    long double dlat = lat2 - lat1;
 
    long double dist = std::pow(std::sin(dlat / 2), 2) + 
                          std::cos(lat1) * std::cos(lat2) * 
                          std::pow(std::sin(dlong / 2), 2);
 
    dist = 2 * std::asin(std::sqrt(dist));
 
    // Radius of Earth in 
    // Kilometers, R = 6371
    // Use R = 3956 for miles
    long double R = 6371000; // meters
     
    dist = dist * R;
 
    return dist;
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapSaverNode>());
  rclcpp::shutdown();
  return 0;
}

