# services/path_planner.py
import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib

# 初始化 ROS 节点
rospy.init_node('drone_path_planner', anonymous=True)

# 创建 MoveBase action 客户端
move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)

def plan_path(start_position, target_position):
    # 等待 move_base 服务器启动
    move_base.wait_for_server()

    # 创建目标点
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target_position['x']
    goal.target_pose.pose.position.y = target_position['y']
    goal.target_pose.pose.position.z = target_position['z']
    
    goal.target_pose.pose.orientation.w = 1.0  # 保持默认的姿态

    # 发布目标点
    move_base.send_goal(goal)

    # 等待结果
    move_base.wait_for_result()

    # 获取路径规划的结果
    result = move_base.get_state()
    if result == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("Path plan successful")
        return True
    else:
        rospy.loginfo("Path plan failed")
        return False
