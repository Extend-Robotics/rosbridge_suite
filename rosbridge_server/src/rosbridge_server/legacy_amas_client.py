import itertools
import json

import rospy

from rosbridge_library.capabilities.advertise import Advertise
from rosbridge_library.capabilities.call_service import CallService, trim_servicename
from rosbridge_library.capabilities.publish import Publish
from rosbridge_library.capabilities.subscribe import Subscribe
from rosbridge_library.protocol import Protocol

FORWARDED_SERVICES = frozenset(["/rosapi/get_param", "/load_file", "/file_server/get_file"])
# https://github.com/Extend-Robotics/extend_unity_release/blob/3dad4e59a90c59d9a04ccb9af6ac68d85cb85d67/Assets/_Project/Scripts/DigitalTwin/JointRealignment.cs#L87-L110
FORWARDED_TOPIC = "/tf2_buffer_server/goal"

GET_PARAM_SERVICE = "/rosapi/get_param"
SYSTEM_INFORMATION_SERVICE = "/system_information"
CORTEX_CONFIGURATION_PROVIDER_SERVICE = "/cortex_configuration_provider"
LOAD_FILE_SERVICE = "/load_file"
LOAD_CONFIG_FILE_SERVICE = "/load_config_file"
RAISE_OPERATOR_ERROR_SERVICE = "/raise_operator_error"

# https://github.com/Extend-Robotics/extend_unity_release/blob/62cbcd5ffc1f1e3f9e2cf59f32c2370a7fedbe7f/Assets/Cloud%20Utilities/ConnectedDeviceManager.cs#L60-L82
ROBOKIT_MODEL_PARAM = "/robokit_model"
LEGACY_AMAS_ROBOKIT_MODEL_PARAM_VALUE = '"XARM6"'
MACHINE_ID_COMMAND = "cat /etc/machine-id"
LEGACY_AMAS_MACHINE_ID = "legacyamasupdaterequired"
# https://github.com/Extend-Robotics/extend_unity_release/blob/62cbcd5ffc1f1e3f9e2cf59f32c2370a7fedbe7f/Assets/_Project/Scripts/Robokit/RobokitMaster.cs#L347-L352
DOCKER_IMAGE_COMMAND = "echo $DOCKER_IMAGE"
# https://github.com/Extend-Robotics/extend_unity_release/blob/ac246536774fcb127b3ed164128cf08d49a31823/Assets/Firmware%20Version%20Manager/FirmwareVersionManager.cs#L50-L70
FIRMWARE_VERSION_ANSWER = "this robot runs new architecture firmware and needs a newer AMAS"

# https://github.com/Extend-Robotics/extend_unity_release/blob/62cbcd5ffc1f1e3f9e2cf59f32c2370a7fedbe7f/Assets/CortexConnectionManager.cs#L111-L293
LEGACY_AMAS_CORTEX_NAME = "AMAS_Update_Required"
LEGACY_AMAS_DEVICE_NAME = "<color=#FF5252>Update AMAS to connect</color>"
LEGACY_AMAS_DEVICE_TYPE = "robokit"
LEGACY_AMAS_ROBOKIT_INFO_PATH = "configs/{}/{}.json".format(LEGACY_AMAS_DEVICE_TYPE, LEGACY_AMAS_DEVICE_NAME)
LEGACY_AMAS_ROBOKIT_ENV_PATH_FORMAT = "firmware_configs/" + LEGACY_AMAS_DEVICE_TYPE + "/{}.env"
EMPTY_ROBOKIT_ENV_CONTENTS = ""
# https://github.com/Extend-Robotics/extend_unity_release/blob/62cbcd5ffc1f1e3f9e2cf59f32c2370a7fedbe7f/Assets/Custom_scripts/ArmSelectionHandler.cs#L205-L233
XARM6_ROBOT_MODEL = 2
# https://github.com/Extend-Robotics/extend_unity_release/blob/62cbcd5ffc1f1e3f9e2cf59f32c2370a7fedbe7f/Assets/_Project/Scripts/Management/InitialDeviceConnectionManager.cs#L368-L371
BACKTICK = "`"

# https://github.com/Extend-Robotics/extend_unity_release/blob/3dad4e59a90c59d9a04ccb9af6ac68d85cb85d67/Assets/URDFLoader.cs#L204-L207
# https://github.com/Extend-Robotics/extend_unity_release/blob/3dad4e59a90c59d9a04ccb9af6ac68d85cb85d67/Assets/_Project/Scripts/Error/RaiseOperatorErrorServiceProvider.cs#L52-L126
UPDATE_REQUIRED_CALL_ID_PREFIX = "legacy_amas_update_required:"
UPDATE_REQUIRED_RESEND_PERIOD = 60.0  # seconds
UPDATE_REQUIRED_OPERATOR_ERROR = {
    "source": "Robot",
    "code": "AMV0001",
    "definition": "This AMAS is too old for this robot: update AMAS. Control is disabled.",
    "resolution": ("Install the current AMAS release on this headset and reconnect. "
                   "Until then this headset is view-only; the robot ignores its commands."),
    "resolved": False,
}


def legacy_amas_robokit_env_path(bridge_port):
    return LEGACY_AMAS_ROBOKIT_ENV_PATH_FORMAT.format(bridge_port)


def system_information_values(request_cli):
    if request_cli == MACHINE_ID_COMMAND:
        return {"cli": LEGACY_AMAS_MACHINE_ID}
    if request_cli == DOCKER_IMAGE_COMMAND:
        return None
    return {"cli": FIRMWARE_VERSION_ANSWER}


def legacy_amas_cortex_configuration_values(connection_ip, bridge_port):
    return {
        "cortexName": LEGACY_AMAS_CORTEX_NAME,
        "cortexConfiguration": [{
            "name": LEGACY_AMAS_DEVICE_NAME,
            "deviceType": LEGACY_AMAS_DEVICE_TYPE,
            "connectionIP": connection_ip,
            "rosPort": str(bridge_port),
        }],
    }


def backtick_quoted_json(values):
    return json.dumps(values, sort_keys=True).replace('"', BACKTICK)


def loaded_file_values(contents):
    return {"contents": contents, "success": True, "error": 0}


def legacy_amas_robokit_info_values(bridge_port):
    return loaded_file_values(backtick_quoted_json({
        "name": LEGACY_AMAS_DEVICE_NAME,
        "model": XARM6_ROBOT_MODEL,
        "rosPort": str(bridge_port),
    }))


def locally_answered_service_values(resolved_service, request_args, bridge_port):
    if resolved_service == GET_PARAM_SERVICE:
        if rospy.resolve_name(request_args["name"]) == ROBOKIT_MODEL_PARAM:
            return {"value": LEGACY_AMAS_ROBOKIT_MODEL_PARAM_VALUE}
        return None
    if resolved_service == SYSTEM_INFORMATION_SERVICE:
        return system_information_values(request_args["request_cli"])
    if resolved_service == CORTEX_CONFIGURATION_PROVIDER_SERVICE:
        return legacy_amas_cortex_configuration_values(request_args["connectionIP"], bridge_port)
    if resolved_service == LOAD_FILE_SERVICE and request_args["path"] == LEGACY_AMAS_ROBOKIT_INFO_PATH:
        return legacy_amas_robokit_info_values(bridge_port)
    if resolved_service == LOAD_CONFIG_FILE_SERVICE and request_args["path"] == legacy_amas_robokit_env_path(bridge_port):
        return loaded_file_values(EMPTY_ROBOKIT_ENV_CONTENTS)
    return None


class LegacyAmasProtocol(Protocol):
    def __init__(self, client_id, bridge_port, parameters=None):
        self.parameters = parameters
        Protocol.__init__(self, client_id)
        self.bridge_port = bridge_port
        subscribe_capability = Subscribe(self)
        self.advertise_capability = Advertise(self)
        self.publish_capability = Publish(self)
        self.call_service_capability = CallService(self)
        self.capabilities = [subscribe_capability, self.advertise_capability, self.publish_capability,
                             self.call_service_capability]
        self.operations = {
            "subscribe": subscribe_capability.subscribe,
            "unsubscribe": subscribe_capability.unsubscribe,
            "advertise": self.advertise_forwarded_topic,
            "publish": self.publish_forwarded_topic,
            "call_service": self.call_service,
            "advertise_service": self.advertise_service,
            "service_response": self.service_response,
            "unadvertise": self.drop,
            "unadvertise_service": self.drop,
            "fragment": self.drop,
        }
        self.reported_dropped_operations = set()
        self.raise_operator_error_service_as_sent = None
        self.update_required_call_numbers = itertools.count(1)
        self.update_required_resend_timer = None

    def drop(self, message):
        operation_and_name = (message["op"], message.get("topic", message.get("service")))
        if operation_and_name not in self.reported_dropped_operations:
            self.reported_dropped_operations.add(operation_and_name)
            self.log("warn", "Dropping %s %s from a legacy AMAS client: it is read-only" % operation_and_name)

    def advertise_forwarded_topic(self, message):
        if rospy.resolve_name(message["topic"]) == FORWARDED_TOPIC:
            self.advertise_capability.advertise(message)
        else:
            self.drop(message)

    def publish_forwarded_topic(self, message):
        if rospy.resolve_name(message["topic"]) == FORWARDED_TOPIC:
            self.publish_capability.publish(message)
        else:
            self.drop(message)

    def call_service(self, message):
        self.call_service_capability.basic_type_check(message, CallService.call_service_msg_fields)
        resolved_service = rospy.resolve_name(trim_servicename(message["service"]))
        local_values = locally_answered_service_values(resolved_service, message.get("args", {}), self.bridge_port)
        if local_values is not None:
            self.send_service_response(message, local_values)
        elif resolved_service in FORWARDED_SERVICES:
            self.call_service_capability.call_service(message)
        else:
            self.drop(message)

    def send_service_response(self, request_message, values):
        service_response = {
            "op": "service_response",
            "service": request_message["service"],
            "values": values,
            "result": True,
        }
        if "id" in request_message:
            service_response["id"] = request_message["id"]
        self.send(service_response)

    def advertise_service(self, message):
        if rospy.resolve_name(message["service"]) != RAISE_OPERATOR_ERROR_SERVICE:
            self.drop(message)
            return
        self.raise_operator_error_service_as_sent = message["service"]
        self.send_update_required_operator_error()
        if self.update_required_resend_timer is None:
            self.update_required_resend_timer = rospy.Timer(rospy.Duration(UPDATE_REQUIRED_RESEND_PERIOD),
                                                            self.send_update_required_operator_error)

    def send_update_required_operator_error(self, _timer_event=None):
        self.send({
            "op": "call_service",
            "id": "{}{}".format(UPDATE_REQUIRED_CALL_ID_PREFIX, next(self.update_required_call_numbers)),
            "service": self.raise_operator_error_service_as_sent,
            "args": UPDATE_REQUIRED_OPERATOR_ERROR,
        })

    def service_response(self, message):
        if not str(message.get("id", "")).startswith(UPDATE_REQUIRED_CALL_ID_PREFIX):
            self.drop(message)
            return
        self.log("info", "Legacy AMAS client answered update-required operator error %s: result %s, values %s" % (
            message["id"], message.get("result"), message.get("values")))

    def finish(self):
        if self.update_required_resend_timer is not None:
            self.update_required_resend_timer.shutdown()
        Protocol.finish(self)
