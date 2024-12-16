import cv2
from cv_bridge.core import CvBridge
from geometry_msgs.msg import TransformStamped
from human_detector.human_detector_parameters import human_detector_parameters
from image_geometry import PinholeCameraModel
import mediapipe as mp
import rclpy
from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.transform_broadcaster import TransformBroadcaster
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.qos_overriding_options import QoSOverridingOptions
from rclpy.qos import qos_profile_sensor_data


def mm_to_m(mm):
    return mm / 1000


class HumanDetector(LifecycleNode):
    def __init__(self):
        super().__init__("human_detector")
        self.param_listener = human_detector_parameters.ParamListener(self)
        self.depth_image: Image = None
        self.image = None
        self.detected_landmarks = None
        self.detected_human_position_world = {"x": 0.0, "y": 0.0}
        self.cv_bridge = CvBridge()
        self.model = PinholeCameraModel()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.person_pose_estimator = None
        self.time_approximation_slope = 0.05
        self.camera_info = None

    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.parameters = self.param_listener.get_params()
        self.log_parameters()
        self.person_pose_estimator = mp.solutions.pose.Pose(
            min_detection_confidence=self.parameters.min_detection_confidence,
            min_tracking_confidence=self.parameters.min_tracking_confidence,
        )
        self.initialize_sync_subscribers()

        if self.parameters.publish_image_with_detected:
            self.image_with_detected_human_pub = self.create_publisher(Image, "image_with_detected_human", 10)
        self.timer = self.create_timer(1 / self.parameters.detected_human_transform_frequency, self.timer_callback)
        self.timer.cancel()

        return TransitionCallbackReturn.SUCCESS

    def initialize_sync_subscribers(self):
        sync_topics = [
            Subscriber(
                self,
                Image,
                "/camera/color/image_raw",
                qos_profile=qos_profile_sensor_data,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                Image,
                "/camera/depth/image_rect_raw",
                qos_profile=qos_profile_sensor_data,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
            Subscriber(
                self,
                CameraInfo,
                "/camera/depth/camera_info",
                qos_profile=qos_profile_sensor_data,
                qos_overriding_options=QoSOverridingOptions.with_default_policies(),
            ),
        ]

        self.image_approx_time_sync = ApproximateTimeSynchronizer(
            sync_topics,
            queue_size=5,
            slop=self.time_approximation_slope,
        )
        self.image_approx_time_sync.registerCallback(self.on_image_data)

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.timer.reset()
        return super().on_activate(previous_state)

    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.timer.cancel()
        return super().on_deactivate(previous_state)

    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        self.destroy_resources()
        return TransitionCallbackReturn.SUCCESS

    def destroy_resources(self):
        self.destroy_timer(self.timer)

    def log_parameters(self):
        self.get_logger().info(f"Human detector uses: {self.parameters.camera_frame_id} as camera link.")
        self.get_logger().info(f"Human detector uses: {self.parameters.detected_human_frame_id} as frame with human.")
        self.get_logger().info(
            "Human detector publishes transform to detected human with "
            f"{self.parameters.detected_human_transform_frequency} Hz."
        )
        self.get_logger().info(
            f"Mediapipe will use {self.parameters.min_detection_confidence} " "as min_detection_confidence"
        )

        self.get_logger().info(
            f"Mediapipe will use {self.parameters.min_tracking_confidence} " "as min_tracking_confidence"
        )
        if self.parameters.publish_image_with_detected:
            self.get_logger().info("Human detector will publish image with detected human.")

    def on_image_data(self, image: Image, depth_image: Image, info: CameraInfo):
        self.camera_info = info
        self.model.fromCameraInfo(self.camera_info)
        self.image = cv2.cvtColor(self.cv_bridge.imgmsg_to_cv2(image), cv2.COLOR_BGR2RGB)
        self.depth_image = self.cv_bridge.imgmsg_to_cv2(depth_image, desired_encoding="16UC1")
        self.store_human_pose()

    def are_rgb_image_same_size_as_depth_image(self):
        rgb_image_height, rgb_image_width, _ = self.image.shape
        depth_image_height, depth_image_width = self.depth_image.shape

        return rgb_image_height == depth_image_height and rgb_image_width == depth_image_width

    def should_detect_human(self):
        if self.camera_info is None or self.image is None or self.depth_image is None:
            self.get_logger().error(
                "No camera info or image or depth image are not stored. Human will not be detected."
            )
            return False

        if not self.are_rgb_image_same_size_as_depth_image():
            self.get_logger().error(
                "Dimensions of rgb image and depth image are not equal. Human will not be detected."
            )
            return False

        return True

    def store_human_pose(self):
        if not self.should_detect_human():
            return

        self.detected_landmarks = self.person_pose_estimator.process(self.image).pose_landmarks
        x_pos_of_detected_person, y_pos_of_detected_person = self.get_position_of_human_in_the_image(
            self.detected_landmarks
        )

        self.get_3d_human_position(x_pos_of_detected_person, y_pos_of_detected_person)

    def get_3d_human_position(self, x_pos_of_detected_person, y_pos_of_detected_person):
        if x_pos_of_detected_person <= 0 or y_pos_of_detected_person <= 0:
            return

        depth_of_given_pixel = self.depth_image[y_pos_of_detected_person, x_pos_of_detected_person]
        ray = self.model.projectPixelTo3dRay((x_pos_of_detected_person, y_pos_of_detected_person))
        ray_3d = [ray_element / ray[2] for ray_element in ray]
        point_xyz = [ray_element * depth_of_given_pixel for ray_element in ray_3d]
        self.detected_human_position_world = {"x": mm_to_m(point_xyz[2]), "y": -mm_to_m(point_xyz[0])}

    def get_position_of_human_in_the_image(self, landmarks):
        x, y = 0, 0
        if self.detected_landmarks:
            landmarks = mp.solutions.pose.PoseLandmark
            left_hip_landmark = self.detected_landmarks.landmark[landmarks.LEFT_HIP]
            right_hip_landmark = self.detected_landmarks.landmark[landmarks.RIGHT_HIP]
            x, y = self.extract_hip_midpoint(left_hip_landmark, right_hip_landmark)

        return x, y

    def extract_hip_midpoint(self, left_hip_landmark, right_hip_landmark):
        height, width, _ = self.image.shape
        x = int(min((left_hip_landmark.x * width + right_hip_landmark.x * width) / 2, width - 1))
        y = int(min((left_hip_landmark.y * height + right_hip_landmark.y * height) / 2, height - 1))
        return x, y

    def timer_callback(self):
        if self.detected_human_position_world["x"] > 0.0:
            self.broadcast_timer_callback()
        self.publish_image_with_detected_human()

    def broadcast_timer_callback(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.parameters.camera_frame_id
        transform.child_frame_id = self.parameters.detected_human_frame_id
        transform.transform.translation.x = self.detected_human_position_world["x"]
        transform.transform.translation.y = self.detected_human_position_world["y"]
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def draw_person_pose(self, image):
        mp_drawing = mp.solutions.drawing_utils
        mp_drawing.draw_landmarks(
            image,
            self.detected_landmarks,
            mp.solutions.pose.POSE_CONNECTIONS,
            mp.solutions.drawing_styles.get_default_pose_landmarks_style(),
        )
        return image

    def publish_image_with_detected_human(self):
        if not self.parameters.publish_image_with_detected:
            return

        if self.detected_landmarks is not None:
            modified_image_msg = self.cv_bridge.cv2_to_imgmsg(self.draw_person_pose(self.image.copy()))
            self.image_with_detected_human_pub.publish(modified_image_msg)
        elif self.image is not None:
            image_msg = self.cv_bridge.cv2_to_imgmsg(self.image)
            self.image_with_detected_human_pub.publish(image_msg)


def main(args=None):
    rclpy.init(args=args)
    pose_detector = HumanDetector()
    rclpy.spin(pose_detector)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
