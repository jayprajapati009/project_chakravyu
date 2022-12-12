#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <vector>
using std::placeholders::_1;
using std_msgs::msg::String;
using RCL_NODE_PTR = std::shared_ptr<rclcpp::Node>;

class CallbackFunction;
using CB_FUNCT_PTR = std::shared_ptr<CallbackFunction>;
using SUBSCRIBER = rclcpp::Subscription<String>::SharedPtr;


class CallbackFunction {
public:
  CallbackFunction(RCL_NODE_PTR node, int id) :
    node_(node),
    count_(0),
    id(id)
  {
    printf ("CallbackFunction constructor\n");
  }

  void topic_callback(const std_msgs::msg::String& msg)
  {
    RCLCPP_INFO(node_->get_logger(), "CallbackFunction id=%d: I heard: '%s'", id, msg.data.c_str());

    // exit nicely so that coverage data can be saved properly
    if (++count_ > 3)
      exit(EXIT_SUCCESS);
  }

  RCL_NODE_PTR node_;
  size_t       count_;
  int          id;
};

class MinimalSubscriber : public rclcpp::Node {
public:

  MinimalSubscriber(const std::string& node_name      = "my_node",
                    const std::string& node_namespace = "/my_ns",
                    const std::string& topic_name     = "my_topic") :
    Node(node_name, node_namespace)
    // count_(0)
  {
    // auto callback = std::bind(&MinimalSubscriber::topic_callback, this, _1);
    // subscription_ = this->create_subscription<String>(topic_name, 10, callback);

    for (int idx = 0; idx < numNodes; idx++)
      {
        std::string nodeName = "my_node" + std::to_string(idx);
        cbFunts_[idx] = std::make_shared<CallbackFunction>(static_cast<RCL_NODE_PTR>(this), idx);

        auto callback = std::bind(&CallbackFunction::topic_callback, cbFunts_[idx], _1);
        subscriptions_[idx] = this->create_subscription<String>(topic_name, 10, callback);
      }
  }

private:

  // void topic_callback(const std_msgs::msg::String& msg)
  // {
  //   RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

  //   // exit nicely so that coverage data can be saved properly
  //   if (++count_ > 3)
  //     exit(EXIT_SUCCESS);
  // }

  // size_t count_;
  // SUBSCRIBER subscription_;

  int                       numNodes       = 5;
  std::vector<CB_FUNCT_PTR> cbFunts_       = std::vector<CB_FUNCT_PTR>(numNodes);
  std::vector<SUBSCRIBER>   subscriptions_ = std::vector<SUBSCRIBER>(numNodes);
  
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<MinimalSubscriber>("Listen_Node", "/", "topic"));
  rclcpp::shutdown();

  return 0;
}
