#!/usr/bin/env python
import rospy
import csv
import os
import math
from geometry_msgs.msg import PoseStamped
from uwb.msg import UwbData  # UwbData 메시지 타입으로 변경

# CSV 파일 경로
csv_file_path = '/workspace/uamtest/3_12/measurement_data/data.csv'
# 파일이 이미 존재하면 삭제
if os.path.exists(csv_file_path):
    os.remove(csv_file_path)

# 데이터를 저장할 딕셔너리
data = {}
# CSV 파일 쓰기 준비
with open(csv_file_path, mode='w', newline='', encoding='utf-8') as file:
    writer = csv.writer(file)
    # CSV 헤더 작성
    writer.writerow([
        'Timestamp', 'Source',
        'PositionX', 'PositionY', 'PositionZ',
        'OrientationX', 'OrientationY', 'OrientationZ', 'OrientationW',
        'Address', 'Range', 'RxPower'
    ])
    # writer.writerow([1,Anchor 2,-1.4152213134765625,2.762179443359375,0.021282070159912108,-0.0033169810191344623,0.018775828771640565,-0.7125776444671046,0.701334133606887,'','',''])
    # writer.writerow([1,Anchor 1,-1.2601585693359374,-2.563399169921875,0.02825090980529785,0.002941351162320118,0.0015370985083729802,0.7079343201066746,0.7062704640269822,'','',''])
    # writer.writerow([1,Anchor 3,2.718420654296875,-0.09661665344238281,0.030677114486694336,0.000615881946956106,-0.0331436428789627,0.7127251279873863,0.7006596969889161,'','',''])

    # 콜백 함수 정의
    def pose_callback(msg, source):
        # CSV에 쓸 데이터 준비
        row = [
            rospy.get_time(), source,
            msg.pose.position.x if not math.isnan(msg.pose.position.x) else '',
            msg.pose.position.y if not math.isnan(msg.pose.position.y) else '',
            msg.pose.position.z if not math.isnan(msg.pose.position.z) else '',
            msg.pose.orientation.x if not math.isnan(msg.pose.orientation.x) else '',
            msg.pose.orientation.y if not math.isnan(msg.pose.orientation.y) else '',
            msg.pose.orientation.z if not math.isnan(msg.pose.orientation.z) else '',
            msg.pose.orientation.w if not math.isnan(msg.pose.orientation.w) else '',
            '', '', ''  # UWB 데이터가 없으므로 빈 문자열
        ]
        # count +=1
        writer.writerow(row)
        # if (source == "MAVROS Vision"):
        #     writer.writerow(row)
        # elif(count < 100):
        #     writer.writerow(row)
            
    def uwb_callback(msg):
        # CSV에 쓸 데이터 준비
        row = [
            rospy.get_time(), 'UWB',
            '', '', '',  # Pose 데이터가 없으므로 빈 문자열
            '', '', '', '',
            msg.Address, msg.range, msg.rxPower
        ]
        writer.writerow(row)

    rospy.init_node('data_to_csv')

    # PoseStamped 메시지를 위한 구독자
    # rospy.Subscriber('/qualisys/anchor1/pose', PoseStamped, pose_callback, 'Anchor 1')
    # rospy.Subscriber('/qualisys/anchor2/pose', PoseStamped, pose_callback, 'Anchor 2')
    # rospy.Subscriber('/qualisys/anchor3/pose', PoseStamped, pose_callback, 'Anchor 3')
    rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, pose_callback, 'MAVROS Vision')

    # UwbData 메시지를 위한 구독자
    rospy.Subscriber('/UwbData', UwbData, uwb_callback)

    rospy.spin()
    # Rate = rospy.rate(10)
    # Rate.sleep()



