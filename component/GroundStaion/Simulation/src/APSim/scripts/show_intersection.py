#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose, Point
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class showIntersection:
    
    def __init__(self) -> None:
        rospy.init_node("showIntersection_Gazebo")
        
        # 初始化IntersectionBall模型的状态
        self.SphereState = ModelState()
        self.SphereState.model_name = 'IntersectionBall'  # 设置你想要控制的模型的名称
        self.SphereState.pose = Pose()  # 初始化pose属性
        
        # 订阅/myPose话题，指定回调函数为changeIntersectionPosition
        self.sub_Pose = rospy.Subscriber("/ap/intersection", Pose, self.changeIntersectionPosition)
        
    def set_state(self):
        rospy.wait_for_service("/gazebo/set_model_state")  
        try:
            set_model_state = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            resp = set_model_state(self.SphereState)
            if not resp.success:
                rospy.logwarn("Failed to update model state: " + resp.status_message)
            else:
                rospy.loginfo("SET SUCESS: {:.2f} {:.2f} {:.2f}".format(self.SphereState.pose.position.x,self.SphereState.pose.position.y,self.SphereState.pose.position.z))
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % str(e))
        
    def changeIntersectionPosition(self, pose_msg: Pose):
        # 更新IntersectionBall模型的位置
        self.SphereState.pose = pose_msg
        
        # 调用set_state方法设置IntersectionBall模型的位置
        self.set_state()
        
    def spin(self):
        rospy.spin()

# 创建showIntersection对象并运行程序
if __name__ == "__main__":
    show_intersection = showIntersection()
    show_intersection.spin()
