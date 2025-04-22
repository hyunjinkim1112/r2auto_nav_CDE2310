// Acknowledgement: This code is based on the code provided by Omar Salem, which is modified to suit the requirements of the project. https://github.com/Omar-Salem/auto_mapper


#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <array>
#include <filesystem>
#include <slam_toolbox/srv/detail/save_map__struct.hpp>
#include <fstream>

#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "map_msgs/msg/occupancy_grid_update.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/occ_grid_values.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "std_msgs/std_msgs/msg/color_rgba.hpp"
#include "nav2_map_server/map_mode.hpp"
#include "nav2_map_server/map_saver.hpp"
#include "slam_toolbox/srv/serialize_pose_graph.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

//Added std messages
#include "std_msgs/msg/string.hpp"
#include <chrono>

using std::placeholders::_1;
using sensor_msgs::msg::Range;
using geometry_msgs::msg::PoseWithCovarianceStamped;
using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Point;
using nav_msgs::msg::OccupancyGrid;
using nav2_msgs::action::NavigateToPose;
using map_msgs::msg::OccupancyGridUpdate;
using visualization_msgs::msg::MarkerArray;
using visualization_msgs::msg::Marker;
using std_msgs::msg::ColorRGBA;
using nav2_costmap_2d::Costmap2D;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;
using nav2_costmap_2d::FREE_SPACE;
using std::to_string;
using std::abs;
using std::chrono::milliseconds;
using namespace std::chrono_literals;
using namespace std;
using namespace rclcpp;
using namespace rclcpp_action;
using namespace nav2_map_server;
using namespace slam_toolbox;
using std::chrono::steady_clock;
using geometry_msgs::msg::Twist;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

class AutoMapper : public Node {
public:
    AutoMapper()
            : Node("auto_mapper") {
        RCLCPP_INFO(get_logger(), "AutoMapper started...");

        move_pub_ = create_publisher<Twist>("/cmd_vel", 10);

        poseSubscription_ = create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, bind(&AutoMapper::poseTopicCallback, this, _1));

        mapSubscription_ = create_subscription<OccupancyGrid>(
                "/map", 10, bind(&AutoMapper::updateFullMap, this, _1));

        markerArrayPublisher_ = create_publisher<MarkerArray>("/frontiers", 10);
        poseNavigator_ = rclcpp_action::create_client<NavigateToPose>(
                this,
                "/navigate_to_pose"); 

        poseNavigator_->wait_for_action_server();
        RCLCPP_INFO(get_logger(), "AutoMapper poseNavigator_");
        declare_parameter("map_path", rclcpp::PARAMETER_STRING);
        get_parameter("map_path", mapPath_);

        //Create status topics
        statusSubscription_ = create_subscription<std_msgs::msg::String>(
                "/navigation_command_in", 10, bind(&AutoMapper::statusTopicCallback, this, _1));
        
        statusPublisher_ = create_publisher<std_msgs::msg::String>("/navigation_status_out", 10);
    }

private:
    const double MIN_FRONTIER_DENSITY = 0.1;
    const double MIN_DISTANCE_TO_FRONTIER_SQUARED = pow(0.65,2);
    const int MIN_FREE_THRESHOLD = 2;
    //const int MAX_OBSTACLE_THRESHOLD = 2;
    Costmap2D costmap_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr poseNavigator_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr move_pub_;
    Publisher<MarkerArray>::SharedPtr markerArrayPublisher_;
    MarkerArray markersMsg_;
    Subscription<OccupancyGrid>::SharedPtr mapSubscription_;
    bool isExploring_ = false;
    int markerId_;
    string mapPath_;
    // Create int to store number of exploration attempts failed
    int exploration_attempts_failed = 0;
    // Added subscriber and publisher for status and command messages
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr statusSubscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr statusPublisher_;
    std_msgs::msg::String::SharedPtr status_;
    // Added failed goal index
    int failed_goal_index_increment = 0;
    // Added explore start check
    bool explore_start = false;

    Subscription<PoseWithCovarianceStamped>::SharedPtr poseSubscription_;
    PoseWithCovarianceStamped::UniquePtr pose_;

    std::chrono::steady_clock::time_point goal_start_time_;
    rclcpp::TimerBase::SharedPtr timer_;  // Timer to handle quick goal completion

    array<unsigned char, 256> costTranslationTable_ = initTranslationTable();

    static array<unsigned char, 256> initTranslationTable() {
        array<unsigned char, 256> cost_translation_table{};

        // lineary mapped from [0..100] to [0..255]
        for (size_t i = 0; i < 256; ++i) {
            cost_translation_table[i] =
                    static_cast<unsigned char>(1 + (251 * (i - 1)) / 97);
        }

        // special values:
        cost_translation_table[0] = FREE_SPACE;
        cost_translation_table[99] = 253;
        cost_translation_table[100] = LETHAL_OBSTACLE;
        /*
        for (size_t i = 3; i <= 252; ++i) {
        cost_translation_table[i] = LETHAL_OBSTACLE;
        }
        */
        cost_translation_table[static_cast<unsigned char>(-1)] = NO_INFORMATION;

        return cost_translation_table;
    }

    struct Frontier {
        Point centroid;
        vector<Point> points;
        string getKey() const{to_string(centroid.x) + "," + to_string(centroid.y);}
    };

    void statusTopicCallback(std_msgs::msg::String::SharedPtr message) {
    // status_ = move(message);  can remove
    RCLCPP_INFO(get_logger(), "statusTopicCallback...");
    if (message->data == "START") {
        RCLCPP_INFO(get_logger(), "Received START exploration command");
        explore_start = true;
        explore();
    }

    if (message->data == "STOP") { //not needed
        stop();
    }
}

    void poseTopicCallback(PoseWithCovarianceStamped::UniquePtr pose) {
        pose_ = move(pose);
        RCLCPP_INFO(get_logger(), "poseTopicCallback...");
    }

    void updateFullMap(OccupancyGrid::UniquePtr occupancyGrid) {
        if (pose_ == nullptr) { return; }
        RCLCPP_INFO(get_logger(), "updateFullMap...");
        const auto occupancyGridInfo = occupancyGrid->info;
        unsigned int size_in_cells_x = occupancyGridInfo.width;
        unsigned int size_in_cells_y = occupancyGridInfo.height;
        double resolution = occupancyGridInfo.resolution;
        double origin_x = occupancyGridInfo.origin.position.x;
        double origin_y = occupancyGridInfo.origin.position.y;

        RCLCPP_INFO(get_logger(), "received full new map, resizing to: %d, %d", size_in_cells_x,
                    size_in_cells_y);
        costmap_.resizeMap(size_in_cells_x,
                           size_in_cells_y,
                           resolution,
                           origin_x,
                           origin_y);

        // lock as we are accessing raw underlying map
        auto *mutex = costmap_.getMutex();
        lock_guard<Costmap2D::mutex_t> lock(*mutex);

        // fill map with data
        unsigned char *costmap_data = costmap_.getCharMap();
        size_t costmap_size = costmap_.getSizeInCellsX() * costmap_.getSizeInCellsY();
        RCLCPP_INFO(get_logger(), "full map update, %lu values", costmap_size);
        for (size_t i = 0; i < costmap_size && i < occupancyGrid->data.size(); ++i) {
            auto cell_cost = static_cast<unsigned char>(occupancyGrid->data[i]);
            costmap_data[i] = costTranslationTable_[cell_cost];
        }
        //explore();
        /*
        if (explore_start) {
            explore_start = false;
            explore();

        else
        }*/
    }

    void drawMarkers(const vector<Frontier> &frontiers) {
        for (const auto &frontier: frontiers) {
            RCLCPP_INFO(get_logger(), "visualising %f,%f ", frontier.centroid.x, frontier.centroid.y);
            ColorRGBA green;
            green.r = 0;
            green.g = 1.0;
            green.b = 0;
            green.a = 1.0;

            vector<Marker> &markers = markersMsg_.markers;
            Marker m;

            m.header.frame_id = "map";
            m.header.stamp = now();
            m.frame_locked = true;

            m.action = Marker::ADD;
            m.ns = "frontiers";
            m.id = ++markerId_;
            m.type = Marker::SPHERE;
            m.pose.position = frontier.centroid;
            m.scale.x = 0.3;
            m.scale.y = 0.3;
            m.scale.z = 0.3;
            m.color = green;
            markers.push_back(m);
            markerArrayPublisher_->publish(markersMsg_);
        }
    }

    void clearMarkers() {
        for (auto &m: markersMsg_.markers) {
            m.action = Marker::DELETE;
        }
        markerArrayPublisher_->publish(markersMsg_);
    }

    void stop() {
        RCLCPP_INFO(get_logger(), "Stopped...");
        poseSubscription_.reset(); //can remove
        mapSubscription_.reset(); //can remove
        poseNavigator_->async_cancel_all_goals();
        //saveMap(); //can remove 
        clearMarkers();  
    }
    void restart() {
        RCLCPP_INFO(get_logger(), "Restarting...");
        poseSubscription_ = create_subscription<PoseWithCovarianceStamped>(
                "/pose", 10, bind(&AutoMapper::poseTopicCallback, this, _1));

        mapSubscription_ = create_subscription<OccupancyGrid>(
                "/map", 10, bind(&AutoMapper::updateFullMap, this, _1));
    }

    void moveInRandomDirection() {
        // Create a twist message for movement
        geometry_msgs::msg::Twist move_cmd;

        // Set a random linear velocity (forward speed)
        move_cmd.linear.x = 0.2;  // Move forward with a small speed (adjust as needed)

        // Set a random angular velocity (left or right turn)
        move_cmd.angular.z = ((rand() % 2) * 2 - 1) * 0.5;  // Random turn (clockwise or counterclockwise)

        // Publish the command to the robot to start moving
        move_pub_->publish(move_cmd);

        RCLCPP_INFO(get_logger(), "Moving in a random direction...");

        // Move for 1 second
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Stop the robot by setting velocity to zero
        move_cmd.linear.x = 0.0;  // Stop linear motion (stop moving forward)
        move_cmd.angular.z = 0.0; // Stop angular motion (stop turning)

        // Publish the stop command
        move_pub_->publish(move_cmd);

        RCLCPP_INFO(get_logger(), "Stopped moving.");
    }

    void explore() {
        //added check for manual exploration start
        if (!explore_start) { 
            RCLCPP_INFO(get_logger(), "Explore stopped");
            return; 
        }
        if (isExploring_) { return; }
        auto frontiers = findFrontiers();
        // If exploration attempts failed more than 5 times, send a message to coordinator node to initiate lidar-based exploration
        if (exploration_attempts_failed > 5) {
            RCLCPP_WARN(get_logger(), "Exploration attempts failed more than 5 times. Requesting r2autonav");
            // Add code for sending a new message to send to the coordinator node here
            std_msgs::msg::String command;
            command.data = "r2autonav";
            statusPublisher_->publish(command);
            rclcpp::sleep_for(std::chrono::seconds(2));
            stop();
            restart();
            //updateFullMap(); //maybe add havent checked effect
            exploration_attempts_failed = 0;
            //explore();
            return;
        }
        if (frontiers.empty()) {
            exploration_attempts_failed++;
            RCLCPP_WARN(get_logger(), "NO BOUNDARIES FOUND!!");
            RCLCPP_WARN(get_logger(), "No frontiers found, retrying...");
            //moveInRandomDirection();
            rclcpp::sleep_for(std::chrono::seconds(1));
            stop();
            restart();
            explore();
            //stop();
            return;
            
        }
        // If failed goal increment gretaer than 3, stop and restart
        if (failed_goal_index_increment > 1) {
            RCLCPP_WARN(get_logger(), "Failed goal index greater than 1. Stopping and restarting...");
            stop();
            restart();
            failed_goal_index_increment = 0;
            return;
        }
        // If goal failed, increment the goal index so previous goal is not chosen
        const auto frontier = frontiers[0 + failed_goal_index_increment];
        drawMarkers(frontiers);
        auto goal = NavigateToPose::Goal();
        goal.pose.pose.position = frontier.centroid;
        goal.pose.pose.orientation.w = 1.;
        goal.pose.header.frame_id = "map";

        RCLCPP_INFO(get_logger(), "Sending goal %f,%f", frontier.centroid.x, frontier.centroid.y);

        auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
        send_goal_options.goal_response_callback = [this, &frontier](
                const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
            if (goal_handle) {
                RCLCPP_INFO(get_logger(), "Goal accepted by server, waiting for result");
                isExploring_ = true;
                goal_start_time_ = std::chrono::steady_clock::now();
            } else {
                RCLCPP_ERROR(get_logger(), "Goal was rejected by server");
            }
        };

        send_goal_options.feedback_callback = [this](
                const GoalHandleNavigateToPose::SharedPtr &,
                const std::shared_ptr<const NavigateToPose::Feedback> &feedback) {
            RCLCPP_INFO(get_logger(), "Distance remaining: %f", feedback->distance_remaining);
        };

        send_goal_options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult &result) {
            isExploring_ = false;
            auto goal_end_time = std::chrono::steady_clock::now();
            auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(goal_end_time - goal_start_time_).count();
            if (elapsed_time < 1) {
                RCLCPP_INFO(get_logger(), "Goal reached too quickly...Exploration failed");
                    //std_msgs::msg::String command;
                    //command.data = "r2autonav";
                    //statusPublisher_->publish(command);
                    exploration_attempts_failed++;
                    stop();
                    restart();
                    explore();
                return;
            }
            std_msgs::msg::String statusMessage;
            statusMessage.data = "GOAL_REACHED";
            //saveMap();
            clearMarkers();
            //explore(); might need to add back
            switch (result.code) {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(get_logger(), "Goal reached");
                    //failed_goal_index_increment = 0;
                    exploration_attempts_failed = 0;
                    explore_start = false;
                    //std_msgs::msg::String statusMessage;
                    //statusMessage.data = "GOAL_REACHED";
                    statusPublisher_->publish(statusMessage);
                    stop();
                    restart();
                    //poseNavigator_->async_cancel_all_goals();
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(get_logger(), "Goal was aborted");
                    // stop(); can try this
                    // restart();
                    //failed_goal_index_increment++;
                    exploration_attempts_failed++;
                    explore();
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_ERROR(get_logger(), "Goal was canceled");
                    //failed_goal_index_increment++;
                    exploration_attempts_failed++;
                    explore();
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Unknown result code");
                    //failed_goal_index_increment++;
                    exploration_attempts_failed++;
                    explore();
                    break;
            }
        };
        poseNavigator_->async_send_goal(goal, send_goal_options);
    }

    void saveMap() {
        auto mapSerializer = create_client<slam_toolbox::srv::SerializePoseGraph>(
                "/slam_toolbox/serialize_map");
        auto serializePoseGraphRequest =
                std::make_shared<slam_toolbox::srv::SerializePoseGraph::Request>();
        serializePoseGraphRequest->filename = mapPath_;
        auto serializePoseResult = mapSerializer->async_send_request(serializePoseGraphRequest);

        auto map_saver = create_client<slam_toolbox::srv::SaveMap>(
                "/slam_toolbox/save_map");
        auto saveMapRequest = std::make_shared<slam_toolbox::srv::SaveMap::Request>();
        saveMapRequest->name.data = mapPath_;
        auto saveMapResult = map_saver->async_send_request(saveMapRequest);
    }

    vector<unsigned int> nhood8(unsigned int idx) {
        unsigned int mx, my;
        vector<unsigned int> out;
        costmap_.indexToCells(idx, mx, my);
        const int x = mx;
        const int y = my;
        const pair<int, int> directions[] = {
                pair(-1, -1),
                pair(-1, 1),
                pair(1, -1),
                pair(1, 1),
                pair(1, 0),
                pair(-1, 0),
                pair(0, 1),
                pair(0, -1)
        };
        for (const auto &d: directions) {
            int newX = x + d.first;
            int newY = y + d.second;
            if (newX >= 0 && newX < static_cast<int>(costmap_.getSizeInCellsX()) &&
                newY >= 0 && newY < static_cast<int>(costmap_.getSizeInCellsY())) {
                out.push_back(costmap_.getIndex(newX, newY));
            }
        }
        return out;
    }

    bool isAchievableFrontierCell(unsigned int idx,
                                  const vector<bool> &frontier_flag) {
        auto map = costmap_.getCharMap();
        // check that cell is unknown and not already marked as frontier
        if (map[idx] != NO_INFORMATION || frontier_flag[idx]) {
            return false;
        }
        /*
        int obstacleCount = 0;
        for (unsigned int nbr: nhood8(idx)) {
            if (map[nbr] == LETHAL_OBSTACLE) {
                if (++obstacleCount >= MAX_OBSTACLE_THRESHOLD) {
                    return false;
                }
            }
        }
        */
        //check there's enough free space for robot to move to frontier
        int freeCount = 0;
        for (unsigned int nbr: nhood8(idx)) {
            if (map[nbr] == FREE_SPACE) {
                if (++freeCount >= MIN_FREE_THRESHOLD) {
                    return true;
                }
            }
        }

        return false;
    }

    Frontier buildNewFrontier(unsigned int neighborCell, vector<bool> &frontier_flag) {
        Frontier output;
        output.centroid.x = 0;
        output.centroid.y = 0;

        queue<unsigned int> bfs;
        bfs.push(neighborCell);

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            // try adding cells in 8-connected neighborhood to frontier
            for (unsigned int nbr: nhood8(idx)) {
                // check if neighbour is a potential frontier cell
                if (isAchievableFrontierCell(nbr, frontier_flag)) {
                    // mark cell as frontier
                    frontier_flag[nbr] = true;
                    unsigned int mx, my;
                    double wx, wy;
                    costmap_.indexToCells(nbr, mx, my);
                    costmap_.mapToWorld(mx, my, wx, wy);

                    Point point;
                    point.x = wx;
                    point.y = wy;
                    output.points.push_back(point);

                    // update centroid of frontier
                    output.centroid.x += wx;
                    output.centroid.y += wy;

                    bfs.push(nbr);
                }
            }
        }

        // average out frontier centroid
        output.centroid.x /= output.points.size();
        output.centroid.y /= output.points.size();
        return output;
    }

    vector<Frontier> findFrontiers() {
        vector<Frontier> frontier_list;
        const auto position = pose_->pose.pose.position;
        unsigned int mx, my;
        if (!costmap_.worldToMap(position.x, position.y, mx, my)) {
            RCLCPP_ERROR(get_logger(), "Robot out of costmap bounds, cannot search for frontiers");
            return frontier_list;
        }

        // make sure map is consistent and locked for duration of search
        lock_guard<Costmap2D::mutex_t> lock(*(costmap_.getMutex()));

        auto map_ = costmap_.getCharMap();
        auto size_x_ = costmap_.getSizeInCellsX();
        auto size_y_ = costmap_.getSizeInCellsY();

        // initialize flag arrays to keep track of visited and frontier cells
        vector<bool> frontier_flag(size_x_ * size_y_,
                                   false);
        vector<bool> visited_flag(size_x_ * size_y_,
                                  false);

        // initialize breadth first search
        queue<unsigned int> bfs;

        unsigned int pos = costmap_.getIndex(mx, my);
        bfs.push(pos);
        visited_flag[bfs.front()] = true;

        while (!bfs.empty()) {
            unsigned int idx = bfs.front();
            bfs.pop();

            for (unsigned nbr: nhood8(idx)) {
                // add to queue all free, unvisited cells, use descending search in case
                // initialized on non-free cell
                if (map_[nbr] == FREE_SPACE && !visited_flag[nbr]) {
                    visited_flag[nbr] = true;
                    bfs.push(nbr);
                    // check if cell is new frontier cell (unvisited, NO_INFORMATION, free
                    // neighbour)
                } else if (isAchievableFrontierCell(nbr, frontier_flag)) {
                    frontier_flag[nbr] = true;
                    const Frontier frontier = buildNewFrontier(nbr, frontier_flag);

                    double distance_squared = pow((double(frontier.centroid.x) - double(position.x)), 2.0) +
                                           pow((double(frontier.centroid.y) - double(position.y)), 2.0);
                    if (distance_squared < MIN_DISTANCE_TO_FRONTIER_SQUARED) { continue; }
                    if (frontier.points.size() * costmap_.getResolution() >=
                        MIN_FRONTIER_DENSITY) {
                        frontier_list.push_back(frontier);
                    }
                }
            }
        }

        return frontier_list;
    }

};

int main(int argc, char *argv[]) {
    init(argc, argv);
    spin(make_shared<AutoMapper>());
    shutdown();
    return 0;
}
