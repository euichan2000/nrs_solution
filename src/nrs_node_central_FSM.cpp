#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <std_msgs/String.h>
#include <string>
#include <cstdlib>
#include <signal.h>
#include <unistd.h>
#include <sys/wait.h>
#include <nrs_solution/HandGesture.h>

class CentralFSM
{
public:
  CentralFSM()
  {
    state = "standby";
    last_left = "";
    last_right = "";
    current_left = "";
    current_right = "";
    active_pid = -1;

    ros::NodeHandle nh;

    state_transitions["standby"] = {
        {{"paper", "paper"}, "scan_mode"},
        {{"paper", "pointing"}, "discrete_teach_mode"},
        {{"paper", "scissors"}, "continuous_teach_mode"},
        {{"paper", "rock"}, "follow_mode"},
        {{"paper", "okay"}, "path_planning_mode"}};

    state_transitions["scan"] = {
        {{"", "rock"}, "terminate scan"},
        {{"", "pointing"}, "scan"},
        {{"", "scissors"}, "registration & reconstruction"}};

    state_transitions["discrete_teach"] = {
        {{"", "rock"}, "terminate discrete_teach"},
        {{"", "pointing"}, "save discrete_teaching waypoints"}};

    state_transitions["continuous_teach"] = {
        {{"", "rock"}, "terminate continuous_teach"},
        {{"", "pointing"}, "save continuous_teaching waypoints"}};

    state_transitions["follow"] = {
        {{"", "rock"}, "terminate follow"}};

    state_transitions["path_planning"] = {
        {{"", "rock"}, "terminate path_planning"},
        {{"", "pointing"}, "generate path using discrete_waypoints"},
        {{"", "scissors"}, "generate path using continuous waypoints"}};

    state_timer = nh.createTimer(
        ros::Duration(1.0),
        &CentralFSM::stateTimerCallback,
        this);

    gesture_sub = nh.subscribe("/hand_gesture", 10, &CentralFSM::gestureCallback, this);

    last_gesture_time = ros::Time::now();
    gestureHoldThreshold = ros::Duration(3.0); // 3초 이상 유지되어야 동작
    ROS_INFO("[FSM] Central FSM node initialized. Current state: standby");
  }

private:
  ros::Subscriber gesture_sub;
  ros::Timer state_timer;
  ros::Publisher command_pub;
  std::string state;
  pid_t active_pid;
  std::string node_to_kill;
  // 실시간 파싱된 제스처
  std::string current_left, current_right;

  // 제스처 지속 확인용 변수
  std::string last_left, last_right;
  ros::Time last_gesture_time;
  ros::Duration gestureHoldThreshold;

  std::map<std::string,
           std::map<std::pair<std::string, std::string>, std::string>>
      state_transitions;

  void stateTimerCallback(const ros::TimerEvent &)
  {
    // 1) 옵션 문자열 만들기
    std::string map_str;
    auto it = state_transitions.find(state);
    if (it != state_transitions.end())
    {
      for (const auto &kv : it->second)
      {
        const auto &gest = kv.first;
        const auto &mode = kv.second;
        if (!map_str.empty())
          map_str += " | ";
        map_str += "(" + gest.first + "," + gest.second + ")->" + mode;
      }
    }
    else
    {
      map_str = "no mappings";
    }

    // 2) 로깅
    ROS_INFO("[FSM] State: %s | Gesture: L:%s, R:%s | Mappings: %s",
             state.c_str(),
             current_left.c_str(),
             current_right.c_str(),
             map_str.c_str());
  }

  void gestureCallback(const std_msgs::String::ConstPtr &msg)
  {
    // parse Left/Right
    std::string data = msg->data;
    std::string left, right;
    size_t pL = data.find("Left:");
    size_t pC = data.find(",Right:");
    if (pL != std::string::npos && pC != std::string::npos)
    {
      left = data.substr(pL + 5, pC - (pL + 5));
      right = data.substr(pC + 7);
    }

    current_left = left;
    current_right = right;
    // ROS_INFO("[FSM] Received gesture -> Left:%s, Right:%s",
    //          left.c_str(),
    //          right.c_str());

    // persistent check
    if (!checkPersistentGesture(left, right))
      return;

    /* ------------------------ Standby Mode ------------------------ */
    if (state == "standby")
    {
      // Launch scan mode when both hands show 'paper'
      if (left == "paper" && right == "paper")
      {
        ROS_INFO("[FSM] Gesture: scan mode trigger");
        callService("/scan_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_scan.launch", "scan");
      }
      // Launch discrete teaching mode with left 'paper', right 'point'
      else if (left == "paper" && right == "pointing")
      {
        ROS_INFO("[FSM] Gesture: discrete teaching trigger");
        callService("/following_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_discrete_teach.launch", "discrete_teach");
      }
      // Launch continuous teaching mode with left 'paper', right 'scissor'
      else if (left == "paper" && right == "scissors")
      {
        ROS_INFO("[FSM] Gesture: continuous teaching trigger");
        callService("/following_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_continuous_teach.launch", "continuous_teach");
      }
      // Launch follow mode with left 'paper', right 'rock'
      else if (left == "paper" && right == "rock")
      {
        ROS_INFO("[FSM] Gesture: follow mode trigger");
        callService("/following_mode");
        callService("/visual_servoing_on");
        launch("nrs_solution", "nrs_follow.launch", "follow");
      }

      // Launch path planning mode with left 'paper', right 'okay'
      else if (left == "paper" && right == "okay")
      {
        ROS_INFO("[FSM] Gesture: path planning mode trigger");
        callService("/visual_servoing_off");
        killNode("rviz");
        killNode("robot_state_publisher");
        launch("nrs_path", "path_planning.launch", "path_planning");
      }

      else
      {
        // ROS_INFO("[FSM] Gesture in standby: no matching transition");
      }
    }

    /* ------------------------ Scan Mode ------------------------ */
    else if (state == "scan")
    {

      // Terminate scan mode with right 'rock'
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: terminating scan mode");
        callService("/visual_servoing_off");
        terminateMode();
      }
      // Trigger scene save with right 'pointing'
      else if (left == "" && right == "pointing")
      {
        callService("/scan");
        last_left = "";
        last_right = "";
      }
      // Trigger registration & reconstruction with right 'okay'
      else if (left == "" && right == "scissors")
      {
        callService("/visual_servoing_off");
        callService("/registration");
        callService("/reconstruction");
        terminateMode();
      }
    }

    /* ------------------------ Continuous Teaching Mode ------------------------ */
    else if (state == "continuous_teach")
    {

      // Exit to standby with right 'rock'
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending continuous teaching mode");
        callService("/continuous_teaching_end");
        callService("/visual_servoing_off");
        terminateMode();
      }
      // Save waypoint with right 'point'
      else if (left == "" && right == "pointing")
      {
        callService("/continuous_teaching_save");
      }
      // Follow hand with right 'paper'
      // else if (left == "" && right == "paper")
      // {
      //   publishCommand("continuous_teaching_follow");
      // }
    }

    /* ------------------------ Discrete Teaching Mode ------------------------ */
    else if (state == "discrete_teach")
    {

      // Exit to standby with right 'rock'
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending discrete teaching mode");
        callService("/discrete_teaching_end");
        callService("/visual_servoing_off");
        terminateMode();
      }
      // Save waypoint with right 'point'
      else if (left == "" && right == "pointing")
      {
        callService("/discrete_teaching_save");
        last_left = "";
        last_right = "";
      }
      // Follow hand with right 'scissors'
      // else if (left == "" && right == "scissors")
      // {
      //   callService("/visual_servoing_off");
      //   launch("nrs_path", "path_planning.launch", "path_planning");
      // }
    }

    /* ------------------------ Follow Mode ------------------------ */
    else if (state == "follow")
    {
      // Exit to standby with right 'rock'
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending follow mode");
        callService("/visual_servoing_off");
        terminateMode();
      }
      // Follow hand with right 'paper'
      // else if (left == "" && right == "paper")
      // {
      //   publishCommand("follow_follow");
      // }
    }

    /* ------------------------ path_planning Mode ------------------------ */
    else if (state == "path_planning")
    {

      // Exit to standby with right 'rock'
      if (left == "" && right == "rock")
      {
        ROS_WARN("[FSM] Gesture: ending path_planning mode");
        terminateMode();
        launch("nrs_robot", "nrs_robot.launch", "standby");
      }
      // discrete path_planning with right 'pointing'
      else if (left == "" && right == "pointing")
      {
        callService("/straight");
        callService("/interpolate");
      }
      // continuous path_planning with right 'scissors'
      else if (left == "" && right == "scissors")
      {
        callService("/straight");
        callService("/interpolate");
      }
      // Follow hand with right 'okay'
      // else if (left == "" && right == "okay")
      // {
      //   callService("/straight");
      // }
    }
  }

  bool checkPersistentGesture(const std::string &l, const std::string &r)
  {
    ros::Time now = ros::Time::now();
    if (l != last_left || r != last_right)
    {
      last_left = l;
      last_right = r;
      last_gesture_time = now;
      return false;
    }
    return (now - last_gesture_time >= gestureHoldThreshold);
  }

  bool launch(const std::string &pkg, const std::string &file, const std::string &mode)
  {
    if (state != "standby")
      return false;

    state = mode;
    ROS_INFO("[FSM] Launching %s mode...", mode.c_str());

    pid_t pid = fork();
    if (pid == 0)
    {
      execlp("roslaunch", "roslaunch", pkg.c_str(), file.c_str(), (char *)nullptr);
      _exit(EXIT_FAILURE);
    }

    active_pid = pid;
    return true;
  }

  void killNode(const std::string &node_name)
  {
    // Execute the rosnode kill command
    std::string cmd = "rosnode kill " + node_name;
    int ret = system(cmd.c_str());
    if (ret != 0)
    {
      ROS_ERROR("Failed to kill node %s (return code %d)", node_name.c_str(), ret);
    }
    else
    {
      ROS_INFO("Killed node %s", node_name.c_str());
    }
  }

  void killLaunch(const std::string &pkg, const std::string &file)
  {
    // build the pattern string, e.g. "roslaunch nrs_solution nrs_scan.launch"
    std::string pattern = "roslaunch " + pkg + " " + file;
    // pkill -f pattern will send SIGTERM to all matching processes
    std::string cmd = "pkill -f \"" + pattern + "\"";
    int ret = system(cmd.c_str());
    if (ret != 0)
    {
      ROS_ERROR("Failed to kill launch [%s %s] (ret=%d)", pkg.c_str(), file.c_str(), ret);
    }
    else
    {
      ROS_INFO("Killed launch [%s %s]", pkg.c_str(), file.c_str());
    }
  }

  void terminateMode()
  {
    if (active_pid > 0)
    {
      kill(active_pid, SIGINT);
      active_pid = -1;
      state = "standby";
      ROS_INFO("[FSM] Returned to standby mode");
    }
  }

  void publishCommand(const std::string &cmd_str)
  {
    std_srvs::Empty srv;
    ros::ServiceClient client = ros::NodeHandle().serviceClient<std_srvs::Empty>("/nrs_command/" + cmd_str);
    if (client.call(srv))
    {
      ROS_INFO("[FSM] Command triggered via service: %s", cmd_str.c_str());
    }
    else
    {
      ROS_ERROR("[FSM] Failed to call command service for: %s", cmd_str.c_str());
    }
  }

  void callService(const std::string &service_name)
  {
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>(service_name);
    std_srvs::Empty srv;
    if (client.call(srv))
    {
      ROS_INFO("[FSM] Visual servoing service called successfully: %s", service_name.c_str());
    }
    else
    {
      ROS_ERROR("[FSM] Failed to call %s", service_name.c_str());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "nrs_node_central_FSM");
  CentralFSM fsm;
  ros::spin();
  return 0;
}
