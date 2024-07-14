from human_detector.human_detector_parameters import human_detector_parameters
from rclpy.lifecycle.node import LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import LifecycleNode
from sensor_msgs.msg import Image, CameraInfo
from image_geometry import PinholeCameraModel
from cv_bridge.core import CvBridge
from rclpy.lifecycle import LifecycleNode
import mediapipe as mp



class HumanDetector(LifecycleNode):
    def __init__(self):
        super().__init__("human_detector")
        param_listener = human_detector_parameters.ParamListener(self)
        self.parameters = param_listener.get_params()
        self.log_parameters()
        self.depth_image: Image = None
        self.image: Image = None
        self.detected_landmarks = None
        self.detected_x: int = 0
        self.detected_y: int = 0
        self.detected_human_pose_world_x: float = 0.0
        self.detected_human_pose_world_y: float = 0.0
        self.detected_human_pose_world_z: float = 0.0
        self.camera_info_is_stored: bool = False
        self.cv_bridge = CvBridge()
        self.model = PinholeCameraModel()
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
        self.timer = self.create_timer(1/self.parameters.detected_human_transform_frequency, self.publish_transform)
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

    def store_image(self):
        pass

    def store_depth_image(self):
        pass

    def store_camera_info(self):
        pass

    def publish_transform(self):
        pass


def main():
    pass

if __name__ == '__main__':
    main()
