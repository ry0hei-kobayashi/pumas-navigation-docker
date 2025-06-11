#!/usr/bin/env python3
import rospy
from path_planner.srv import GetPlanWithVia
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Header

def create_pose_stamped(x, y, frame="map"):
    pose = PoseStamped()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = frame
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = 0.0
    pose.pose.orientation.w = 1.0
    return pose

def main():
    rospy.init_node("test_viapoint_client", anonymous=True)
    while not rospy.is_shutdown():
        path_pub = rospy.Publisher("/test_path", Path, queue_size=1, latch=True)  # latch=trueで保持

        rospy.wait_for_service("/path_planner/plan_path")
        try:
            get_plan = rospy.ServiceProxy("/path_planner/plan_path", GetPlanWithVia)

            # 経路取得（viaあり）
            via1 = create_pose_stamped(2.5, 0.0)
            via2 = create_pose_stamped(2.3, 4.0)
            start = create_pose_stamped(0.0, 0.0)
            goal  = create_pose_stamped(0.0, 3.0)

            resp = get_plan(start=start, goal=goal, via_points=[via1, via2])

            print(f"→ Path length: {len(resp.plan.poses)} points")

            # Publish path to RViz
            resp.plan.header.stamp = rospy.Time.now()
            resp.plan.header.frame_id = "map"
            path_pub.publish(resp.plan)

        except rospy.ServiceException as e:
            print("Service call failed:", e)

if __name__ == "__main__":
    main()

