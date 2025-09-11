import rospy
from dynamic_reconfigure.server import Server
from ocam.cfg import CamConfig

def callback(config, level):
    # 매개변수 변경 시 호출될 함수
    rospy.loginfo("""Reconfigure Request: {exposure}, {gain}, 
          {wb_blue}, {wb_red}, {auto_exposure}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("cam_node", anonymous = False)

    srv = Server(CamConfig, callback)
    rospy.spin()
