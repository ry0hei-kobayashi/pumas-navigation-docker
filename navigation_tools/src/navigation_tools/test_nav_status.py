
import rospy
from actionlib_msgs.msg import GoalStatus

class CheckNavStatus():
    def __init__(self):
        rospy.init_node('check_navigation_status')
        self.sub = rospy.Subscriber('/navigation/status', GoalStatus, self.callback)

    def callback(self, msg):
        status = msg.status
        text   = msg.text
        rospy.loginfo(msg)

        #if status == 1:
        #    #rospy.loginfo('start')

        ###SUCCESS###
        if status == 1:
            if text == 'Near goal point':
                rospy.loginfo('#######Near Goal')

        if status == 3:
                rospy.loginfo('#########Goal Success')



        ###FAIL###
        if status == 4:
            if text == 'Cannot calculate path from start to goal point':
                rospy.loginfo('#########Path Plan Fail')
            if text == 'Cancelling current movement'

        



if __name__ == '__main__':
    CheckNavStatus()
    rospy.spin()

