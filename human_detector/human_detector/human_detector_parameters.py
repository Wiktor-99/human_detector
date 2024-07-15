# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
from generate_parameter_library_py.python_validators import ParameterValidators



class human_detector_parameters:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        camera_frame_id = "camera_link"
        detected_human_frame_id = "detected_human"
        detected_human_transform_frequency = 10
        publish_image_with_detected_human_topic = ""



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = human_detector_parameters.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("human_detector_parameters." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters


        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "camera_frame_id":
                    updated_params.camera_frame_id = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detected_human_frame_id":
                    updated_params.detected_human_frame_id = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "detected_human_transform_frequency":
                    updated_params.detected_human_transform_frequency = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "publish_image_with_detected_human_topic":
                    updated_params.publish_image_with_detected_human_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))



            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "camera_frame_id"):
                descriptor = ParameterDescriptor(description="Name of frame containing the camera.", read_only = True)
                parameter = updated_params.camera_frame_id
                self.node_.declare_parameter(self.prefix_ + "camera_frame_id", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detected_human_frame_id"):
                descriptor = ParameterDescriptor(description="Name of frame containing detected human.", read_only = True)
                parameter = updated_params.detected_human_frame_id
                self.node_.declare_parameter(self.prefix_ + "detected_human_frame_id", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "detected_human_transform_frequency"):
                descriptor = ParameterDescriptor(description="Frequency of publishing transform to detected human.", read_only = True)
                parameter = updated_params.detected_human_transform_frequency
                self.node_.declare_parameter(self.prefix_ + "detected_human_transform_frequency", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "publish_image_with_detected_human_topic"):
                descriptor = ParameterDescriptor(description="Name of topic for publishing image with detected human.", read_only = True)
                parameter = updated_params.publish_image_with_detected_human_topic
                self.node_.declare_parameter(self.prefix_ + "publish_image_with_detected_human_topic", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "camera_frame_id")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.camera_frame_id = param.value
            param = self.node_.get_parameter(self.prefix_ + "detected_human_frame_id")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detected_human_frame_id = param.value
            param = self.node_.get_parameter(self.prefix_ + "detected_human_transform_frequency")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.detected_human_transform_frequency = param.value
            param = self.node_.get_parameter(self.prefix_ + "publish_image_with_detected_human_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.publish_image_with_detected_human_topic = param.value


            self.update_internal_params(updated_params)
