from human_detector.human_detector_parameters import human_detector_parameters
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from rclpy.lifecycle import LifecycleNode
from cv_bridge.core import CvBridge
from typing import NamedTuple
import mediapipe as mp
import cv2

def mm_to_m(mm):
    return mm / 1000

class HumanDetector(LifecycleNode):
    def __init__(self):
        super().__init__("human_detector")
        param_listener = human_detector_parameters.ParamListener(self)
        self.parameters = param_listener.get_params()
        self.log_parameters()
        self.depth_image: Image = None
        self.image: Image = None
        self.detected_landmarks = None
        self.detected_human_position_world = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.camera_info_is_stored: bool = False
        self.cv_bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.person_pose_estimator = mp.solutions.pose.Pose(min_detection_confidence=0.8, min_tracking_confidence=0.5)

    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.image_subscription = self.create_subscription(
            Image, "/camera/third_person_camera/color/image_raw", self.store_image, 10
        )
        self.depth_image_subscription = self.create_subscription(
            Image, "/camera/third_person_camera/depth/image_rect_raw", self.store_depth_image, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera/third_person_camera/depth/camera_info", self.store_camera_info, 10
        )
        if self.is_publishing_image_with_detected_human_needed():
            self.image_with_detected_human_pub = self.create_publisher(
                Image, "/camera/third_person_camera/color/person_selected", 10
            )
        self.timer = self.create_timer(1/self.parameters.detected_human_transform_frequency, self.timer_callback)
        self.timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        return super().on_deactivate(previous_state)

    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        return TransitionCallbackReturn.SUCCESS

    def log_parameters(self):
        self.get_logger().info(f"Human detector uses: {self.parameters.camera_frame_id} as camera link.")
        self.get_logger().info(f"Human detector uses: {self.parameters.detected_human_frame_id} as frame with human.")
        self.get_logger().info(
            f"Human detector publishes transform to detected human with \
            {self.parameters.detected_human_transform_frequency} Hz."
        )

        if self.is_publishing_image_with_detected_human_needed():
            self.get_logger().info(
                f"Human detector publishes image with detected human on \
                {self.parameters.publish_image_with_detected_human_topic} topic.")

    def is_publishing_image_with_detected_human_needed(self):
        return self.parameters.publish_image_with_detected_human_topic is not None

    def store_image(self, image_msg: Image):
        self.image = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(image_msg), cv2.COLOR_BGR2RGB)
        self.store_human_pose()

    def store_depth_image(self, depth_image_msg: Image):
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image_msg, desired_encoding="16UC1")
        self.store_human_pose()

    def store_camera_info(self, info: CameraInfo):
        self.model.fromCameraInfo(info)
        if not self.camera_info_is_stored:
            self.camera_info_is_stored = True

    def store_human_pose(self):
        if not self.camera_info_is_stored or self.image is None or self.depth_image is None:
            self.get_logger().error("No camera info or image or depth image are not stored")
            return

        hight, width, _ = self.image.shape
        self.detected_landmarks = self.person_pose_estimator.process(self.image).pose_landmarks
        x_pos_of_detected_person, y_pos_of_detected_person = \
            self.get_position_of_human_in_the_image(self.detected_landmarks, width, hight)

        if x_pos_of_detected_person <= 0 or y_pos_of_detected_person <= 0:
            return

        depth_of_given_pixel = self.depth_image[y_pos_of_detected_person, x_pos_of_detected_person]
        ray = self.model.projectPixelTo3dRay((x_pos_of_detected_person, y_pos_of_detected_person))
        ray_3d = [ray_element / ray[2] for ray_element in ray]
        point_xyz = [ray_element * depth_of_given_pixel for ray_element in ray_3d]
        self.detected_human_position_world = {
            'x': mm_to_m(point_xyz[2]),
            'y': -mm_to_m(point_xyz[0]),
            'z': mm_to_m(point_xyz[1])
        }

    def get_position_of_human_in_the_image(self, detected_landmarks: NamedTuple, width: int, height: int):
        x, y = 0, 0
        if detected_landmarks:
            landmarks = mp.solutions.pose.PoseLandmark
            left_hip_landmark = detected_landmarks.landmark[landmarks.LEFT_HIP]
            right_hip_landmark = detected_landmarks.landmark[landmarks.RIGHT_HIP]
            x = int(min((left_hip_landmark.x * width + right_hip_landmark.x * width) / 2, width - 1))
            y = int(min((left_hip_landmark.y * height + right_hip_landmark.y * height) / 2, height - 1))

        return x, y

    def timer_callback(self):
        if self.detected_human_position_world['z'] > 0:
            self.broadcast_timer_callback()
        self.publish_image_with_detected_human()

    def broadcast_timer_callback(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parameters.camera_frame_id
        transform.child_frame_id = self.parameters.detected_human_frame_id
        transform.transform.translation.x = self.detected_human_position_world['x']
        transform.transform.translation.y = self.detected_human_position_world['y']
        transform.transform.translation.z = self.detected_human_position_world['z']
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def draw_person_pose(self, image, landmarks):
        if landmarks is not None:
            mp_drawing = mp.solutions.drawing_utils
            mp_drawing.draw_landmarks(
                image,
                landmarks,
                mp.solutions.pose.POSE_CONNECTIONS,
                mp.solutions.drawing_styles.get_default_pose_landmarks_style()
            )

    def publish_image_with_detected_human(self):
        if not self.is_publishing_image_with_detected_human_needed():
            return
        if self.detected_landmarks is not None:
            self.draw_person_pose(self.image, self.detected_landmarks)
            modified_image_msg = self.cv_bridge.cv2_to_imgmsg(cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR))
            self.image_with_detected_human_pub.publish(modified_image_msg)
        elif self.image is not None:
            image_msg = self.cv_bridge.cv2_to_imgmsg(cv2.cvtColor(self.image, cv2.COLOR_RGB2BGR))
            self.image_with_detected_human_pub.publish(image_msg)


def main():
    pass

if __name__ == '__main__':
    main()
