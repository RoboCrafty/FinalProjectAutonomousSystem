// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <std_msgs/msg/bool.hpp>
// #include <std_srvs/srv/empty.hpp>
// #include <cmath>

// using namespace std::chrono_literals;

// enum class MissionState {
//     APPROACH_CAVE,      // Hover, Approach, and Enter
//     EXPLORE_CAVE,       // Wait for map and start frontier exploration
//     HUNT_LANTERN,       // Activate hunter and wait for completion
//     MISSION_COMPLETED,
// };

// class StateMachineNode : public rclcpp::Node {
// public:
//     StateMachineNode() : Node("state_machine_node"), current_state_(MissionState::APPROACH_CAVE), sub_step_(0), has_odom_(false), target_sent_(false) {

//         target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
//         explore_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_exploration", 10);
//         octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");
//         hunt_pub_ = this->create_publisher<std_msgs::msg::Bool>("/enable_hunting", 10);
        
//         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/current_state_est", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
//                 current_pos_ = msg->pose.pose.position;
//                 has_odom_ = true;
//             });

//         explore_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//             "/exploration_complete", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
//                 if (msg->data) exploration_finished_received_ = true;
//             });

//         hunting_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//             "/hunting_done", 10, [this](const std_msgs::msg::Bool::SharedPtr msg) {
//                 if (msg->data) hunting_finished_received_ = true;
//             });

//         lantern_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
//             "/lantern/target_data", 10, [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
//                 // If a lantern is found (z != 2.0) and we are currently exploring
//                 if (msg->z != 2.0 && current_state_ == MissionState::EXPLORE_CAVE) {
//                     current_state_ = MissionState::HUNT_LANTERN;
//                     RCLCPP_INFO(this->get_logger(), "Lantern spotted! Pausing exploration.");
//                 }
//             });

//         mission_timer_ = this->create_wall_timer(500ms, std::bind(&StateMachineNode::stateMachineTick, this));
//         RCLCPP_INFO(this->get_logger(), "State Machine Ready");
//     }

// private:
//     void stateMachineTick() {
//         if (!has_odom_) return;

//         switch (current_state_) {
//             case MissionState::APPROACH_CAVE:
//                 handleApproachPhase();
//                 break;

//             case MissionState::EXPLORE_CAVE:
//                 handleExplorationPhase();
//                 break;

//             case MissionState::HUNT_LANTERN:
//                 handleHuntingPhase();
//                 break;

//             case MissionState::MISSION_COMPLETED:
//                 setExploration(false);
//                 RCLCPP_INFO_ONCE(this->get_logger(), "Mission completed. Hovering at start.");
//                 break;
//         }
//     }



//     //APPROACH STATE
//     void handleApproachPhase() {
//         setExploration(false); // Ensure explorer is OFF

//         // Climb to Hover point
//         if (sub_step_ == 0) {
//             if (!target_sent_) { sendTarget(-36.0, 10.0, 25.0, 0.0, 0.0, 1.0, 0.0); target_sent_ = true; RCLCPP_INFO(this->get_logger(), "Climbing..."); }
//             if (isReached(-36.0, 10.0, 25.0)) { sub_step_++; target_sent_ = false; }
//         }
//         //Approach Entrance
//         else if (sub_step_ == 1) {
//             if (!target_sent_) { sendTarget(-310.14, 8.38, 15.0); target_sent_ = true; RCLCPP_INFO(this->get_logger(), "Approaching Entrance..."); }
//             if (isReached(-310.14, 8.38, 15.0)) { 
//                 resetOctomap(); // Clear old map data before entering
//                 sub_step_++; target_sent_ = false; 
//             }
//         }
//         //Enter Cave
//         else if (sub_step_ == 2) {
//             if (!target_sent_) { sendTarget(-330.14, 8.38, 15.0); target_sent_ = true; RCLCPP_INFO(this->get_logger(), "Entering Cave..."); }
//             if (isReached(-330.14, 8.38, 15.0)) {
//                 start_wait_time_ = this->now();
//                 current_state_ = MissionState::EXPLORE_CAVE; // Move to next major Phase
//                 sub_step_ = 0; // Reset sub_step for next phase if needed
//                 target_sent_ = false;
//             }
//         }
//     }



//     //EXPLORATION STATE
//     void handleExplorationPhase() {
//         //Wait for map sensors to fill after the reset
//         if ((this->now() - start_wait_time_) < 4s) {
//             setExploration(false);
//             return;
//         }

//         // Once sensors are ready, start the actual explorer
//         if (exploration_finished_received_) {
//             current_state_ = MissionState::MISSION_COMPLETED;
//         } else {
//             setExploration(true);
//             RCLCPP_INFO_ONCE(this->get_logger(), "Frontier Explorer active.");
//         }
//     }



//     // HUNTING STATE
//     void handleHuntingPhase() {
//         setExploration(false); 
//         setHunting(true); 

//         if (hunting_finished_received_) {
//             RCLCPP_INFO(this->get_logger(), "Hunting complete. Resuming exploration.");
//             hunting_finished_received_ = false;
//             current_state_ = MissionState::EXPLORE_CAVE;
//         }
//     }


//     // HELPER FUNCTIONS
//     void setExploration(bool enable) {
//         std_msgs::msg::Bool msg;
//         msg.data = enable;
//         explore_pub_->publish(msg);
//     }


//     void setHunting(bool enable) {
//         std_msgs::msg::Bool msg;
//         msg.data = enable;
//         hunt_pub_->publish(msg);
//     }


//     void resetOctomap() {
//         if (octomap_reset_client_->wait_for_service(1s)) {
//             octomap_reset_client_->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
//             RCLCPP_INFO(this->get_logger(), "Octomap reset.");
//         }
//     }


//     void sendTarget(double x, double y, double z, double qx = 0.0, double qy = 0.0, double qz = 0.0, double qw = 0.0) {
//         auto msg = geometry_msgs::msg::PoseStamped();
//         msg.header.stamp = this->now();
//         msg.header.frame_id = "world";
//         msg.pose.position.x = x;
//         msg.pose.position.y = y;
//         msg.pose.position.z = z;
//         msg.pose.orientation.x = qx;
//         msg.pose.orientation.y = qy;
//         msg.pose.orientation.z = qz;
//         msg.pose.orientation.w = qw;
//         target_pub_->publish(msg);
//     }


//     bool isReached(double tx, double ty, double tz) {
//         double dist = std::sqrt(std::pow(tx - current_pos_.x, 2) + std::pow(ty - current_pos_.y, 2) + std::pow(tz - current_pos_.z, 2));
//         return dist < 1.5;
//     }


//     // Members
//     MissionState current_state_;
//     int sub_step_;
//     bool has_odom_, target_sent_;
//     bool exploration_finished_received_ = false;
//     bool hunting_finished_received_ = false;
//     geometry_msgs::msg::Point current_pos_;
//     rclcpp::Time start_wait_time_;
    
//     rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr explore_pub_;
//     rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr hunt_pub_;
//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr explore_finished_sub_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr hunting_done_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lantern_sub_;
//     rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
//     rclcpp::TimerBase::SharedPtr mission_timer_;
// };



// int main(int argc, char** argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<StateMachineNode>());
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_srvs/srv/empty.hpp>
#include <cmath>

using namespace std::chrono_literals;

// --- CONFIG ---
static constexpr int    TOTAL_LANTERNS       = 4;    // complete mission after finding this many
static constexpr double LANTERN_COOLDOWN_SEC = 8.0;  // seconds before the same detection counts again
static constexpr double LANTERN_MIN_AREA     = 50.0; // minimum pixel area to count as real detection
// --------------

enum class MissionState {
    APPROACH_CAVE,    // Hover, Approach, and Enter
    EXPLORE_CAVE,     // Frontier explore (runs throughout, even during hunting)
    HUNT_LANTERN,     // Hunter is active — exploration keeps running in parallel
    MISSION_COMPLETED
};

class StateMachineNode : public rclcpp::Node {
public:
    StateMachineNode()
    : Node("state_machine_node"),
      current_state_(MissionState::APPROACH_CAVE),
      sub_step_(0),
      has_odom_(false),
      target_sent_(false),
      lanterns_found_(0),
      lantern_visible_(false),
      last_lantern_time_(this->now() - rclcpp::Duration(10, 0))
    {
        target_pub_        = this->create_publisher<geometry_msgs::msg::PoseStamped>("/next_setpoint", 10);
        explore_pub_       = this->create_publisher<std_msgs::msg::Bool>("/enable_exploration", 10);
        hunt_pub_          = this->create_publisher<std_msgs::msg::Bool>("/enable_hunting", 10);
        lantern_count_pub_ = this->create_publisher<std_msgs::msg::Int32>("/lantern_count", 10);

        octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>("/octomap_server/reset");

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/current_state_est", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                current_pos_ = msg->pose.pose.position;
                has_odom_ = true;
            });

        explore_finished_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/exploration_complete", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data) exploration_finished_received_ = true;
            });

        // hunting_done: hunter finished chasing one lantern -> back to pure EXPLORE_CAVE
        hunting_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/hunting_done", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                if (msg->data && current_state_ == MissionState::HUNT_LANTERN) {
                    hunting_finished_received_ = true;
                }
            });

        // Lantern detector: counts rising-edge events, NEVER disables exploration
        lantern_sub_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/lantern/target_data", 10,
            [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
                handleLanternData(msg);
            });

        mission_timer_ = this->create_wall_timer(
            500ms, std::bind(&StateMachineNode::stateMachineTick, this));

        RCLCPP_INFO(this->get_logger(), "State Machine Ready.");
    }

private:

    // -----------------------------------------------------------------------
    //  Lantern detection callback
    //  - Counts each new lantern (rising-edge + cooldown guard)
    //  - Triggers HUNT_LANTERN but exploration is NEVER disabled here
    // -----------------------------------------------------------------------
    void handleLanternData(const geometry_msgs::msg::Vector3::SharedPtr msg) {
        if (current_state_ != MissionState::EXPLORE_CAVE &&
            current_state_ != MissionState::HUNT_LANTERN) return;

        bool in_frame = (msg->z != 2.0) && (msg->y > LANTERN_MIN_AREA);

        if (in_frame) {
            if (!lantern_visible_) {
                // Rising edge — new detection event
                double secs = (this->now() - last_lantern_time_).seconds();
                if (secs >= LANTERN_COOLDOWN_SEC) {
                    lanterns_found_++;
                    last_lantern_time_ = this->now();
                    lantern_visible_   = true;

                    std_msgs::msg::Int32 count_msg;
                    count_msg.data = lanterns_found_;
                    lantern_count_pub_->publish(count_msg);

                    RCLCPP_WARN(this->get_logger(),
                        "*** LANTERN %d / %d DETECTED ***", lanterns_found_, TOTAL_LANTERNS);

                    // Activate hunter only from EXPLORE_CAVE (already hunting = ignore)
                    if (current_state_ == MissionState::EXPLORE_CAVE) {
                        hunting_finished_received_ = false;
                        current_state_ = MissionState::HUNT_LANTERN;
                        RCLCPP_INFO(this->get_logger(),
                            "HUNT_LANTERN activated — frontier explorer stays ON.");
                    }
                }
            }
            // else: same continuous detection, skip
        } else {
            lantern_visible_ = false; // falling edge: ready to detect next lantern
        }
    }

    // -----------------------------------------------------------------------
    //  Main tick
    // -----------------------------------------------------------------------
    void stateMachineTick() {
        if (!has_odom_) return;

        switch (current_state_) {
            case MissionState::APPROACH_CAVE:   handleApproachPhase();    break;
            case MissionState::EXPLORE_CAVE:    handleExplorationPhase(); break;
            case MissionState::HUNT_LANTERN:    handleHuntingPhase();     break;
            case MissionState::MISSION_COMPLETED:
                setExploration(false);
                setHunting(false);
                RCLCPP_INFO_ONCE(this->get_logger(),
                    "Mission complete — %d lanterns found. Hovering.", lanterns_found_);
                break;
        }
    }

    // -----------------------------------------------------------------------
    //  APPROACH STATE — fly to cave entrance and enter
    // -----------------------------------------------------------------------
    void handleApproachPhase() {
        setExploration(false);
        setHunting(false);

        if (sub_step_ == 0) {
            if (!target_sent_) {
                sendTarget(-36.0, 10.0, 25.0, 0.0, 0.0, 1.0, 0.0);
                target_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "Climbing...");
            }
            if (isReached(-36.0, 10.0, 25.0)) { sub_step_++; target_sent_ = false; }
        }
        else if (sub_step_ == 1) {
            if (!target_sent_) {
                sendTarget(-310.14, 8.38, 15.0);
                target_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "Approaching Entrance...");
            }
            if (isReached(-310.14, 8.38, 15.0)) {
                resetOctomap();
                sub_step_++; target_sent_ = false;
            }
        }
        else if (sub_step_ == 2) {
            if (!target_sent_) {
                sendTarget(-330.14, 8.38, 15.0);
                target_sent_ = true;
                RCLCPP_INFO(this->get_logger(), "Entering Cave...");
            }
            if (isReached(-330.14, 8.38, 15.0)) {
                start_wait_time_ = this->now();
                current_state_   = MissionState::EXPLORE_CAVE;
                sub_step_        = 0;
                target_sent_     = false;
                RCLCPP_INFO(this->get_logger(), "Cave entered — starting exploration.");
            }
        }
    }

    // -----------------------------------------------------------------------
    //  EXPLORE STATE — pure frontier exploration, no hunting
    // -----------------------------------------------------------------------
    void handleExplorationPhase() {
        // Brief sensor warm-up after octomap reset
        if ((this->now() - start_wait_time_) < 4s) {
            setExploration(false);
            return;
        }

        // End conditions
        if (lanterns_found_ >= TOTAL_LANTERNS || exploration_finished_received_) {
            current_state_ = MissionState::MISSION_COMPLETED;
            return;
        }

        setExploration(true);
        RCLCPP_INFO_ONCE(this->get_logger(), "Frontier Explorer active.");
    }

    // -----------------------------------------------------------------------
    //  HUNT STATE
    //  THE KEY FIX: setExploration(true) — frontier explorer stays ON
    //  The hunter publishes to /command/trajectory (trajectory override)
    //  The explorer publishes to /next_setpoint (setpoint)
    //  Both run simultaneously — drone follows hunter trajectory while
    //  the explorer silently tracks progress for when hunting ends.
    // -----------------------------------------------------------------------
    void handleHuntingPhase() {
        setExploration(true);  // <-- frontier explorer stays ON
        setHunting(true);      // <-- hunter also ON

        // All lanterns found during hunting
        if (lanterns_found_ >= TOTAL_LANTERNS) {
            setHunting(false);
            current_state_ = MissionState::MISSION_COMPLETED;
            return;
        }

        // This hunt is done — back to pure exploration
        if (hunting_finished_received_) {
            RCLCPP_INFO(this->get_logger(),
                "Hunt complete (%d / %d). Resuming pure exploration.",
                lanterns_found_, TOTAL_LANTERNS);
            hunting_finished_received_ = false;
            setHunting(false);
            current_state_ = MissionState::EXPLORE_CAVE;
        }
    }

    // -----------------------------------------------------------------------
    //  Helpers
    // -----------------------------------------------------------------------
    void setExploration(bool enable) {
        std_msgs::msg::Bool msg;
        msg.data = enable;
        explore_pub_->publish(msg);
    }

    void setHunting(bool enable) {
        std_msgs::msg::Bool msg;
        msg.data = enable;
        hunt_pub_->publish(msg);
    }

    void resetOctomap() {
        if (octomap_reset_client_->wait_for_service(1s)) {
            octomap_reset_client_->async_send_request(
                std::make_shared<std_srvs::srv::Empty::Request>());
            RCLCPP_INFO(this->get_logger(), "Octomap reset.");
        }
    }

    void sendTarget(double x, double y, double z,
                    double qx = 0.0, double qy = 0.0,
                    double qz = 0.0, double qw = 0.0)
    {
        auto msg = geometry_msgs::msg::PoseStamped();
        msg.header.stamp    = this->now();
        msg.header.frame_id = "world";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;
        msg.pose.orientation.x = qx;
        msg.pose.orientation.y = qy;
        msg.pose.orientation.z = qz;
        msg.pose.orientation.w = qw;
        target_pub_->publish(msg);
    }

    bool isReached(double tx, double ty, double tz) {
        double dist = std::sqrt(
            std::pow(tx - current_pos_.x, 2) +
            std::pow(ty - current_pos_.y, 2) +
            std::pow(tz - current_pos_.z, 2));
        return dist < 1.5;
    }

    // -----------------------------------------------------------------------
    //  Members
    // -----------------------------------------------------------------------
    MissionState current_state_;
    int  sub_step_;
    bool has_odom_, target_sent_;
    bool exploration_finished_received_ = false;
    bool hunting_finished_received_     = false;

    int          lanterns_found_;
    bool         lantern_visible_;
    rclcpp::Time last_lantern_time_;

    geometry_msgs::msg::Point current_pos_;
    rclcpp::Time start_wait_time_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             explore_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr             hunt_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr            lantern_count_pub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         explore_finished_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         hunting_done_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr lantern_sub_;

    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr octomap_reset_client_;
    rclcpp::TimerBase::SharedPtr mission_timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StateMachineNode>());
    rclcpp::shutdown();
    return 0;
}