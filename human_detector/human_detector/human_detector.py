from human_detector.human_detector_parameters import human_detector_parameters
from rclpy.lifecycle import LifecycleNode

class HumanDetector(LifecycleNode):
    def __init__(self):
        super().__init__("human_detector")
        param_listener = human_detector_parameters.ParamListener(self)
        self.parameters = param_listener.get_params()
        self.log_parameters()

    def log_parameters(self):
        self.get_logger().info(f"Human detector uses: {self.parameters.camera_frame_id} as camera link.")
        self.get_logger().info(f"Human detector uses: {self.parameters.detected_human_frame_id} as frame with human.")
        self.get_logger().info(
            f"Human detector publishes transform to detected human with \
            {self.parameters.detected_human_transform_frequency} Hz."
        )

        if self.parameters.publish_image_with_detected_human_topic is not None:
            self.get_logger().info(
                f"Human detector publishes image with detected human on \
                {self.parameters.publish_image_with_detected_human_topic} topic.")



def main():
    pass

if __name__ == '__main__':
    main()
