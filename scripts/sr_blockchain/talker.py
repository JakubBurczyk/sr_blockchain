from tracemalloc import start
import rospy
from std_msgs.msg import String
from sr_blockchain.srv import getTransactionHistory
from urllib3 import Retry
from sr_blockchain.subscriber import subFoo


def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

def getHistory(request):
    return(f" {type(request)} | {request} | History")

def serviceServer():
    print("Starting talker srv server")
    rospy.init_node('talkerServiceServer', anonymous=True)
    service = rospy.Service("getTransactionHistory", getTransactionHistory, getHistory)
    rospy.spin()

if __name__ == '__main__':
    a = subFoo()
    try:
        serviceServer()
        #talker()
    except rospy.ROSInterruptException:
        pass
