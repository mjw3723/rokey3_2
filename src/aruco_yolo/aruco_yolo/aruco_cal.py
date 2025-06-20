import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import pickle
import tf_transformations # 쿼터니언 변환을 위해 추가 (pip install transforms3d 필요)
from aruco_msgs.msg import MarkerArray, Marker
from ament_index_python.packages import get_package_share_directory
import yaml
# 만약 aruco_msgs/msg/MarkerArray 등을 발행하려면 아래 주석 해제
# from aruco_msgs.msg import MarkerArray, Marker
# from geometry_msgs.msg import Pose, Point, Quaternion

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        # 캘리브레이션 데이터 로드
        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_matrix, self.dist_coeffs = self.load_camera_parameters('calibration_params.yaml')


        # ArUco 검출기 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        
        # 마커 크기 설정 (미터 단위) - 실제 마커의 크기와 정확히 일치해야 합니다!
        self.marker_size = 0.05  # 예: 5cm = 0.05m
        
        # ROS 2 이미지 토픽 구독
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # v4l2_camera 노드의 기본 이미지 토픽 경로
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.get_logger().info('ArUco Detector Node has started and subscribed to /image_raw.')

        self.marker_publisher = self.create_publisher(MarkerArray, '/detected_markers', 10)

    def load_camera_parameters(self,yaml_file):
        package_share_directory = get_package_share_directory('aruco_yolo')
        calibration_file = os.path.join(package_share_directory, 'config', yaml_file)

        with open(calibration_file, 'r') as f:
            data = yaml.safe_load(f)
            camera_matrix = np.array(data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
            dist_coeffs = np.array(data["distortion_coefficients"]["data"], dtype=np.float32)
            
        return camera_matrix, dist_coeffs
    def image_callback(self, msg):
        # ROS Image 메시지를 OpenCV 이미지로 변환
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
            return
            
        # 이미지 왜곡 보정
        # 프레임 크기 가져오기
        h, w = frame.shape[:2]
        # 최적의 카메라 행렬 구하기 (ROI 포함)
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(self.camera_matrix, self.dist_coeffs, (w,h), 1, (w,h))
        # 왜곡 보정 수행
        frame_undistorted = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs, None, newcameramtx)
        # ROI로 이미지 자르기
        x, y, w_roi, h_roi = roi
        if w_roi > 0 and h_roi > 0: # 유효한 ROI가 있을 때만 자르기
            frame_undistorted = frame_undistorted[y:y+h_roi, x:x+w_roi]

        # 마커 검출
        corners, ids, rejected = self.detector.detectMarkers(frame_undistorted)
        
        # 마커가 검출되면 표시 및 포즈 추정
        marker_array_msg = MarkerArray() # (선택 사항) 발행하려면 주석 해제
        marker_array_msg.header.stamp = msg.header.stamp
        marker_array_msg.header.frame_id = msg.header.frame_id
        
        if ids is not None:
            # 검출된 마커 표시
            cv2.aruco.drawDetectedMarkers(frame_undistorted, corners, ids)

            rvecs_list = []
            tvecs_list = []

            # 마커의 3D 월드 좌표 (사각형 마커)
            _marker_points = np.array([
                [-self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, self.marker_size / 2, 0],
                [self.marker_size / 2, -self.marker_size / 2, 0],
                [-self.marker_size / 2, -self.marker_size / 2, 0]
            ], dtype=np.float32)

            # 각 마커에 대해 포즈 추정 및 처리
            for i in range(len(ids)):
                # 각 마커의 2D 이미지 좌표
                _corners = corners[i][0]
                
                # cv2.solvePnP를 사용하여 rvec 및 tvec 계산
                ret_pnp, rvec, tvec = cv2.solvePnP(
                    _marker_points, 
                    _corners, 
                    self.camera_matrix, 
                    self.dist_coeffs, 
                    flags=cv2.SOLVEPNP_IPPE_SQUARE
                )
                
                if ret_pnp:
                    rvecs_list.append(rvec)
                    tvecs_list.append(tvec)

                    
                    # 마커의 3D 위치 표시
                    # pos_x = tvec[0][0]
                    # pos_y = tvec[0][1]
                    # pos_z = tvec[0][2]
                    pos_x = float(tvec[0][0])
                    pos_y = float(tvec[1][0])
                    pos_z = float(tvec[2][0])
                    
                    # 회전 벡터를 오일러 각도로 변환 (tf_transformations 사용)
                    rotation_matrix = np.array(cv2.Rodrigues(rvec)[0])
                    quaternion = tf_transformations.quaternion_from_matrix(
                        np.array([
                            [rotation_matrix[0,0], rotation_matrix[0,1], rotation_matrix[0,2], 0],
                            [rotation_matrix[1,0], rotation_matrix[1,1], rotation_matrix[1,2], 0],
                            [rotation_matrix[2,0], rotation_matrix[2,1], rotation_matrix[2,2], 0],
                            [0, 0, 0, 1]
                        ])
                    )
                    euler_angles = tf_transformations.euler_from_quaternion(quaternion, axes='sxyz')
                    euler_angles_deg = np.degrees(euler_angles)
                    
                    # 마커 정보 표시 위치 계산
                    corner_mean = np.mean(corners[i][0], axis=0).astype(int)
                    center_x, center_y = corner_mean[0], corner_mean[1]
                    

                    marker_msg = Marker()
                    marker_msg.header.stamp = msg.header.stamp
                    marker_msg.header.frame_id = msg.header.frame_id
                    marker_msg.id = int(ids[i][0])
                    marker_msg.pose.pose.position.x = pos_x
                    marker_msg.pose.pose.position.y = pos_y
                    marker_msg.pose.pose.position.z = pos_z
                    marker_msg.pose.pose.orientation.x = quaternion[0]
                    marker_msg.pose.pose.orientation.y = quaternion[1]
                    marker_msg.pose.pose.orientation.z = quaternion[2]
                    marker_msg.pose.pose.orientation.w = quaternion[3]
                    marker_array_msg.markers.append(marker_msg)

                else:
                    self.get_logger().warn(f"Failed to estimate pose for ArUco marker ID {ids[i][0]}")

            # (선택 사항) MarkerArray 발행
            if self.marker_publisher:
                self.get_logger().info(f"detect =================")
                self.marker_publisher.publish(marker_array_msg)
        
    
def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node) # 노드 실행
    except KeyboardInterrupt: # Ctrl+C 예외 처리
        pass
    finally:
        node.destroy_node() # 노드 자원 해제
        cv2.destroyAllWindows() # OpenCV 창 닫기
        rclpy.shutdown() # ROS 시스템 종료

if __name__ == "__main__":
    main()