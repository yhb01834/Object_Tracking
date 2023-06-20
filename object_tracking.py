#!/usr/bin/env python3
# encoding: utf-8
# @data:2022/11/07
# @author:aiden

import cv2
import rospy
import numpy as np
import jetauto_sdk.pid as pid
import jetauto_sdk.fps as fps
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


#
#/home/jetauto_ws/src/jetauto_example/scripts/tracker/object_tracking.py
#

class MedianFlowNode:
    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.pid_d = pid.PID(1.8, 0, 0)
        #self.pid_d = pid.PID(0, 0, 0)
        
        self.pid_angular = pid.PID(0.005, 0, 0)
        self.go_speed, self.turn_speed = 0.007, 0.04
        self.linear_x, self.angular = 0, 0

        self.fps = fps.FPS()
        
        self.DISTANCE = 100 # 정지 거리 단위 cm
        self.distance = self.DISTANCE # 선택 시 정지 거리 단위 cm
        self.mouse_click = False
        self.selection = None   # 마우스의 추적 영역을 실시간으로 추적
        self.track_window = None   # 감지할 물체의 영역
        self.drag_start = None   # 플래그, 마우스 드래그 시작 여부
        self.start_circle = True
        self.start_click = False
        self.depth_frame = None

        #######################################################
        self.tracker = 
        ###################################################### 
        fs = cv2.FileStorage("custom_csrt.xml", cv2.FILE_STORAGE_READ)
        fn = fs.getFirstTopLevelNode()
        #self.tracker.save('default_csrt.xml')
        self.tracker.read(fn)

        cv2.namedWindow(name, 1)
        cv2.setMouseCallback(name, self.onmouse)
        camera = rospy.get_param('/depth_camera/camera_name', 'camera')
        self.image_sub = rospy.Subscriber('/%s/rgb/image_raw'%camera, Image, self.image_callback, queue_size=1)  
        self.depth_image_sub = rospy.Subscriber('/%s/depth/image_raw'%camera, Image, self.depth_image_callback, queue_size=1)
        self.mecanum_pub = rospy.Publisher('/jetauto_controller/cmd_vel', Twist, queue_size=1)
        rospy.sleep(0.2)
        self.mecanum_pub.publish(Twist())

    def depth_image_callback(self, depth_image):
        self.depth_frame = np.ndarray(shape=(depth_image.height, depth_image.width), dtype=np.uint16, buffer=depth_image.data)
        
    def image_callback(self, ros_image: Image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8, buffer=ros_image.data) 
        
        try:
            ##################################################
            result_image = self.          (np.copy(rgb_image))
            ##################################################
        except BaseException as e:
            print(e)
            result_image = rgb_image
        cv2.imshow(self.name, cv2.cvtColor(result_image, cv2.COLOR_RGB2BGR))
        key = cv2.waitKey(1)
        if key != -1:
            self.mecanum_pub.publish(Twist())
            rospy.signal_shutdown('shutdown')
     
    # 마우스 클릭 이벤트 콜백 함수
    def onmouse(self, event, x, y, flags, param): 
        # 왼쪽 마우스 클릭 시 
        if event == cv2.EVENT_LBUTTONDOWN:       
            self.mouse_click = True
            self.drag_start = (x, y)       # 마우스 시작 위치
            self.track_window = None
        if self.drag_start:       # 마우스 끌기 시작 및 마우스 위치 기록 여부
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)

        # 왼쪽 마우스 떼면 선택 영역 녹화
        if event == cv2.EVENT_LBUTTONUP:     
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
            roi = self.depth_frame[self.track_window[1]:self.track_window[3], self.track_window[0]:self.track_window[2]]
            try:
                roi_h, roi_w = roi.shape
                if roi_h > 0 and roi_w > 0:
                    roi_cut = [1/3, 1/3]
                    if roi_w < 100:
                        roi_cut[1] = 0
                    if roi_h < 100:
                        roi_cut[0] = 0
                    roi_distance = roi[int(roi_h*roi_cut[0]):(roi_h - int(roi_h*roi_cut[0])), 
                            int(roi_w*roi_cut[1]):(roi_w - int(roi_w*roi_cut[1]))]
                    self.distance = int(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<30000)])/10)
                else:
                    self.distance = self.DISTANCE
            except:
                self.distance = self.distance

        # 오른쪽 버튼 누르면 추적 중지
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None   # 마우스의 추적 영역 삭제
            self.track_window = None   # 감지할 물체의 영역
            self.drag_start = None   # 플래그, 마우스 드래그 시작 여부
            self.start_circle = True
            self.start_click = False
            self.mecanum_pub.publish(Twist())
            #########################################################
            self.tracker = 
            #########################################################
            fs = cv2.FileStorage("custom_csrt.xml", cv2.FILE_STORAGE_READ)
            fn = fs.getFirstTopLevelNode()
            #tracker.save('default_csrt.xml')
            self.tracker.read(fn)

    def image_proc(self, image):

        # 추적 시작 (추적 대상에 네모난 창 그려주기 전)
        if self.start_circle:
            # 마우스로 상자를 드래그하여 영역 지정            
            if self.track_window:      # 추적 대상을 추적하는 창을 그린 후 추적 대상을 실시간으로 표시
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]), (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:      # 마우스를 드래그하면 실시간으로 표적을 추적하는 창이 표시됩니다.
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]), (0, 0, 255), 2)
            if self.mouse_click:
                self.start_click = True
            ############ 계속 추적 #############    
            if self.______  :
                if not self.mouse_click:
                    self.______ = False
            ##################################
            
            # 선택한 추적기 track_window를 bbox에 추가하여 이미지와 함께 tracker에 초기화 시켜준다 
            if not self.start_circle:
                print('start tracking')
                #### bbox = (x,y,w,h) : x,y는 왼쪽 상단의 모서리 좌표, w,h는 너비와 높이
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0], self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
        # 시작 외
        else:
            # 속도 조절
            twist = Twist()
            # !!!!!!!!!! tracker에 그 다음 이미지 업데이트해주고 다음 장면에 추적물체 결과값 return 
            ######################################
            ok, ______ = self.tracker.update(image)
            ######################################

            if ok and min(bbox) > 0:
                # 대상의 bbox로부터 중심(center) 얻어오기
                p1 = (int(bbox[0]), int(bbox[1]))
                p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
                center = [(p1[0] + p2[0])/2, (p1[1] + p2[1])/2]

                ##################### 대상과 로봇 사이 간격 계산 #############################
                roi = self.depth_frame[p1[1]:p2[1], p1[0]:p2[0]]
                roi_h, roi_w = roi.shape
                if roi_h > 0 and roi_w > 0:
                    roi_cut = [1/3, 1/3]
                    if roi_w < 100:
                        roi_cut[1] = 0
                    if roi_h < 100:
                        roi_cut[0] = 0
                    roi_distance = roi[int(roi_h*roi_cut[0]):(roi_h - int(roi_h*roi_cut[0])), 
                            int(roi_w*roi_cut[1]):(roi_w - int(roi_w*roi_cut[1]))]
                    try:
                        distance = int(np.mean(roi_distance[np.logical_and(roi_distance>0, roi_distance<30000)])/10)
                    except:
                        distance = self.DISTANCE
                    ####################################################################
                    # 이미지에 거리 및 추적 사각형 그리기
                    cv2.putText(image, 'Distance: ' + str(distance) + 'cm', (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1) 
                    #cv2.drawMarker(image, (int(center[0]), int(center[1])), (0, 255, 0), markerType=0, thickness=2, line_type=cv2.LINE_AA)
                    cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
                    h, w = image.shape[:2]
                    

                    ################ 로봇 제어 ###################
                    # 1. 대상 센터의 좌표, PID 컨트롤러 지속적 업데이트 

                    #### PID distance 
                    self.pid_d.SetPoint = self.distance
                    if abs(distance - self.distance) < 10:
                        distance = self.distance
                    self.pid_d.update(distance)  # PID 업데이트
                    #######################################
                    tmp = self.________ - self.pid_d.output
                    #######################################
                    self.linear_x = tmp
                    if tmp > 0.3:
                        self.linear_x = 0.3
                    if tmp < -0.3:
                        self.linear_x = -0.3
                    if abs(tmp) < 0.008:
                        self.linear_x = 0
                    twist.linear.x = self.linear_x
                    
                    #### PID angular 
                    self.pid_angular.SetPoint = w/2
                    if abs(center[0] - w/2) < 10:
                        center[0] = w/2
                    self.pid_angular.update(center[0])  # PID 업데이트
                    tmp = self.turn_speed + self.pid_angular.output
                    self.angular = tmp
                    if tmp > 1:
                        self.angular = 1
                    if tmp < -1:
                        self.angular = -1
                    if abs(tmp) < 0.05:
                        self.angular = 0
                    twist.angular.z = self.angular

                else:
                    cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            else:
                # Tracking failure
                cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 1)
            self.mecanum_pub.publish(twist)
        self.fps.update()
        result_image = self.fps.show_fps(image)
        
        return result_image

def main():
    medianflow_node = MedianFlowNode('MedianFlow_tracking')
    try:
        rospy.spin()
    except Exception as e:
        medianflow_node.mecanum_pub.publish(Twist())
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()
