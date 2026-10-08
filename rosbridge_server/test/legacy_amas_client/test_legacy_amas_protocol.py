#!/usr/bin/env python
import json
import time
import unittest

import rospy
import rostest

from rosbridge_library.protocol import Protocol
from rosbridge_server import legacy_amas_client
from rosbridge_server.legacy_amas_client import LegacyAmasProtocol

BRIDGE_PORT = 9090
ROSSHARP_FRAGMENT_SIZE = 2147483647
HEADSET_CONNECTION_IP = '192.168.1.42'
OLD_ARCHITECTURE_FIRMWARE_VERSION_ANSWER = 'this robot runs new architecture firmware and needs a newer AMAS'
V8_3_FIRMWARE_VERSION_WORD_INDEX = 5
RESEND_TEST_PERIOD = 0.2  # seconds
RESEND_TEST_TIMEOUT = 5.0  # seconds


def get_param_call(call_id, service_as_sent, param_name):
    return {'op': 'call_service', 'id': call_id, 'service': service_as_sent,
            'args': {'name': param_name, 'default': 'default'},
            'fragment_size': ROSSHARP_FRAGMENT_SIZE, 'compression': 'none'}


class ForwardedMessageRecorder(object):
    def __init__(self):
        self.forwarded_messages = []

    def forward(self, message):
        self.forwarded_messages.append(message)


class TestLegacyAmasProtocol(unittest.TestCase):
    def setUp(self):
        self.protocol = LegacyAmasProtocol('test_legacy_amas_client', BRIDGE_PORT)
        self.sent_frames = []
        self.protocol.outgoing = self.sent_frames.append
        self.forwarded_service_calls = ForwardedMessageRecorder()
        self.forwarded_advertisements = ForwardedMessageRecorder()
        self.forwarded_publications = ForwardedMessageRecorder()
        self.protocol.call_service_capability.call_service = self.forwarded_service_calls.forward
        self.protocol.advertise_capability.advertise = self.forwarded_advertisements.forward
        self.protocol.publish_capability.publish = self.forwarded_publications.forward

    def tearDown(self):
        self.protocol.finish()

    def receive_binary_frames(self, *messages):
        for message in messages:
            self.protocol.incoming(json.dumps(message).encode('utf-8'))

    def sent_messages(self):
        return [json.loads(frame) for frame in self.sent_frames]

    def assert_nothing_forwarded(self):
        self.assertEqual([], self.forwarded_service_calls.forwarded_messages)
        self.assertEqual([], self.forwarded_advertisements.forwarded_messages)
        self.assertEqual([], self.forwarded_publications.forwarded_messages)
        self.assertEqual({}, Protocol.external_service_list)

    def test_only_read_only_operations_are_routed(self):
        self.assertEqual(
            sorted(['subscribe', 'unsubscribe', 'advertise', 'publish', 'call_service', 'advertise_service',
                    'service_response', 'unadvertise', 'unadvertise_service', 'fragment']),
            sorted(self.protocol.operations.keys()))

    def test_only_the_tf2_buffer_server_goal_is_advertised_and_published(self):
        tf2_goal_advertisement = {'op': 'advertise', 'topic': '/tf2_buffer_server/goal',
                                  'type': 'tf2_msgs/LookupTransformActionGoal'}
        tf2_goal_publication = {'op': 'publish', 'topic': 'tf2_buffer_server/goal', 'msg': {}}
        self.receive_binary_frames(
            tf2_goal_advertisement, tf2_goal_publication,
            {'op': 'advertise', 'topic': '/extend_control_command', 'type': 'std_msgs/String'},
            {'op': 'publish', 'topic': '/extend_control_command', 'msg': {'data': 'move'}},
            {'op': 'publish', 'topic': 'tf2_buffer_server//cancel', 'msg': {}},
            {'op': 'unadvertise', 'topic': '/tf2_buffer_server/goal'})
        self.assertEqual([tf2_goal_advertisement], self.forwarded_advertisements.forwarded_messages)
        self.assertEqual([tf2_goal_publication], self.forwarded_publications.forwarded_messages)
        self.assertEqual([], self.forwarded_service_calls.forwarded_messages)
        self.assertEqual([], self.sent_messages())

    def test_only_listed_services_are_forwarded_whatever_their_spelling(self):
        forwarded_calls = [
            {'op': 'call_service', 'id': 'a', 'service': 'rosapi/get_param', 'args': {'name': '/robot_description'}},
            {'op': 'call_service', 'id': 'b', 'service': '/rosapi//get_param', 'args': {'name': 'robot_name'}},
            {'op': 'call_service', 'id': 'c', 'service': '/load_file', 'args': {'path': 'configs/AMAS_targets.json'}},
            {'op': 'call_service', 'id': 'd', 'service': '/file_server/get_file', 'args': {'name': 'package://a.dae'}},
        ]
        refused_calls = [
            {'op': 'call_service', 'id': 'e', 'service': 'rosapi/set_param', 'args': {'name': '/a', 'value': '1'}},
            {'op': 'call_service', 'id': 'f', 'service': '/rosapi/delete_param', 'args': {'name': '/a'}},
            {'op': 'call_service', 'id': 'g', 'service': 'save_file', 'args': {'path': 'a', 'contents': 'b'}},
            {'op': 'call_service', 'id': 'h', 'service': '/delete_file', 'args': {'path': 'a'}},
            {'op': 'call_service', 'id': 'i', 'service': '~load_file', 'args': {'path': 'a'}},
            {'op': 'call_service', 'id': 'j', 'service': '/rosapi/get_param_names', 'args': {}},
            {'op': 'call_service', 'id': 'k', 'service': 'load_config_file', 'args': {'path': 'firmware_configs/a.env'}},
            {'op': 'call_service', 'id': 'l', 'service': '/extend_control_manager', 'args': {}},
        ]
        self.receive_binary_frames(*(forwarded_calls + refused_calls))
        self.assertEqual(forwarded_calls, self.forwarded_service_calls.forwarded_messages)
        self.assertEqual([], self.sent_messages())

    def test_other_operations_are_dropped_without_reply(self):
        self.receive_binary_frames(
            {'op': 'advertise_service', 'service': '/extend_control_manager', 'type': 'std_srvs/Trigger'},
            {'op': 'unadvertise_service', 'service': 'raise_operator_error'},
            {'op': 'service_response', 'id': 'call_service:/x:1', 'service': '/x', 'values': {}, 'result': True},
            {'op': 'fragment', 'id': 'f', 'data': '{', 'num': 0, 'total': 2},
            {'op': 'set_level', 'level': 'info'})
        self.assert_nothing_forwarded()
        self.assertEqual([], self.sent_messages())

    def test_update_required_operator_error_is_resent_while_connected(self):
        original_resend_period = legacy_amas_client.UPDATE_REQUIRED_RESEND_PERIOD
        legacy_amas_client.UPDATE_REQUIRED_RESEND_PERIOD = RESEND_TEST_PERIOD
        try:
            self.receive_binary_frames({'op': 'advertise_service', 'service': 'raise_operator_error',
                                        'type': 'extend_msgs/RaiseOperatorError'})
            deadline = time.time() + RESEND_TEST_TIMEOUT
            while len(self.sent_frames) < 3 and time.time() < deadline:
                time.sleep(RESEND_TEST_PERIOD / 4)
        finally:
            legacy_amas_client.UPDATE_REQUIRED_RESEND_PERIOD = original_resend_period
        self.assertEqual(['legacy_amas_update_required:1', 'legacy_amas_update_required:2',
                          'legacy_amas_update_required:3'],
                         [message['id'] for message in self.sent_messages()[:3]])
        self.protocol.finish()
        frames_sent_before_disconnect = len(self.sent_frames)
        time.sleep(RESEND_TEST_PERIOD * 3)
        self.assertEqual(frames_sent_before_disconnect, len(self.sent_frames))

    def test_v2026_07_and_urdf_test_load_the_robot_and_are_told_to_update(self):
        forwarded_config_and_urdf_calls = [
            get_param_call('rosapi/get_param:1', 'rosapi/get_param',
                           '/returned_frontend_setup_config/iotDeviceConfiguration/deviceName'),
            get_param_call('rosapi/get_param:2', 'rosapi/get_param', '/returned_frontend_setup_config/subrobots'),
            {'op': 'call_service', 'id': '/load_file:3', 'service': '/load_file',
             'args': {'path': 'configs/AMAS_targets.json'},
             'fragment_size': ROSSHARP_FRAGMENT_SIZE, 'compression': 'none'},
            get_param_call('rosapi/get_param:4', 'rosapi/get_param', '/returned_frontend_setup_config/sensors'),
            get_param_call('/rosapi/get_param:5', '/rosapi/get_param', '/robot/name'),
            get_param_call('/rosapi/get_param:6', '/rosapi/get_param', '/robot_description'),
            {'op': 'call_service', 'id': '/file_server/get_file:7', 'service': '/file_server/get_file',
             'args': {'name': 'package://xarm_description/meshes/link1.stl'}, 'fragment_size': ROSSHARP_FRAGMENT_SIZE,
             'compression': 'none'},
        ]
        tf2_goal_advertisement = {'op': 'advertise', 'id': None, 'topic': 'tf2_buffer_server/goal',
                                  'type': 'tf2_msgs/LookupTransformActionGoal'}
        tf2_goal_publication = {'op': 'publish', 'id': None, 'topic': 'tf2_buffer_server/goal', 'msg': {}}
        self.receive_binary_frames(*forwarded_config_and_urdf_calls)
        self.receive_binary_frames(
            {'op': 'advertise_service', 'id': None, 'service': 'raise_operator_error',
             'type': 'extend_msgs/RaiseOperatorError'},
            {'op': 'subscribe', 'id': None, 'topic': 'joint_states', 'type': 'sensor_msgs/JointState'},
            tf2_goal_advertisement, tf2_goal_publication,
            {'op': 'advertise', 'id': None, 'topic': 'tf2_buffer_server/cancel', 'type': 'actionlib_msgs/GoalID'},
            {'op': 'advertise', 'id': None, 'topic': 'extend_control_command', 'type': 'extend_msgs/ArmControl'},
            {'op': 'publish', 'id': None, 'topic': 'extend_control_command', 'msg': {}},
            {'op': 'advertise', 'id': None, 'topic': 'extend_vr_hand_state_info', 'type': 'extend_msgs/VRHandStates'},
            {'op': 'publish', 'id': None, 'topic': 'extend_vr_hand_state_info', 'msg': {}},
            {'op': 'advertise', 'id': None, 'topic': '/cmd_vel', 'type': 'geometry_msgs/Twist'},
            {'op': 'publish', 'id': None, 'topic': '/cmd_vel', 'msg': {}},
            {'op': 'service_response', 'id': 'legacy_amas_update_required:1', 'service': 'raise_operator_error',
             'values': {'ack': True}, 'result': True},
            {'op': 'unadvertise_service', 'id': None, 'service': 'raise_operator_error',
             'type': 'extend_msgs/RaiseOperatorError'})
        self.assertEqual(forwarded_config_and_urdf_calls, self.forwarded_service_calls.forwarded_messages)
        self.assertEqual([tf2_goal_advertisement], self.forwarded_advertisements.forwarded_messages)
        self.assertEqual([tf2_goal_publication], self.forwarded_publications.forwarded_messages)
        self.assertEqual([{
            'op': 'call_service',
            'id': 'legacy_amas_update_required:1',
            'service': 'raise_operator_error',
            'args': {
                'source': 'Robot',
                'code': 'AMV0001',
                'definition': 'This AMAS is too old for this robot: update AMAS. Control is disabled.',
                'resolution': ('Install the current AMAS release on this headset and reconnect. '
                               'Until then this headset is view-only; the robot ignores its commands.'),
                'resolved': False,
            },
        }], self.sent_messages())
        self.assertEqual({}, Protocol.external_service_list)

    def assert_old_architecture_robokit_connection_shows_mismatched_firmware(self, version_query):
        self.receive_binary_frames(
            {'op': 'call_service', 'id': 'rosapi/get_param:0', 'service': 'rosapi/get_param',
             'args': {'name': '/robokit_model', 'default': 'default'}},
            {'op': 'call_service', 'id': 'system_information:0', 'service': 'system_information',
             'args': {'request_cli': 'cat /etc/machine-id'}},
            version_query)
        self.assertEqual([
            {'op': 'service_response', 'id': 'rosapi/get_param:0', 'service': 'rosapi/get_param', 'result': True,
             'values': {'value': '"XARM6"'}},
            {'op': 'service_response', 'id': 'system_information:0', 'service': 'system_information',
             'result': True, 'values': {'cli': 'legacyamasupdaterequired'}},
            {'op': 'service_response', 'id': version_query['id'], 'service': version_query['service'],
             'result': True, 'values': {'cli': OLD_ARCHITECTURE_FIRMWARE_VERSION_ANSWER}},
        ], self.sent_messages())
        self.assert_nothing_forwarded()

    def test_v8_3_to_v10_3_add_robot_wizard_reads_a_model_it_knows(self):
        self.receive_binary_frames(
            {'op': 'call_service', 'id': 'rosapi/get_param:0', 'service': 'rosapi/get_param',
             'args': {'name': 'robokit_model', 'default': 'default'}},
            {'op': 'call_service', 'id': 'rosapi/get_param:1', 'service': 'rosapi/get_param',
             'args': {'name': 'rgbd_server/camera_type', 'default': 'default'}})
        self.assertEqual([{'op': 'service_response', 'id': 'rosapi/get_param:0', 'service': 'rosapi/get_param',
                           'result': True, 'values': {'value': '"XARM6"'}}], self.sent_messages())
        self.assertEqual(['rosapi/get_param:1'],
                         [message['id'] for message in self.forwarded_service_calls.forwarded_messages])

    def test_old_architecture_firmware_version_answer_survives_v8_3_parsing_and_never_matches(self):
        self.assertGreater(len(OLD_ARCHITECTURE_FIRMWARE_VERSION_ANSWER.split(' ')), V8_3_FIRMWARE_VERSION_WORD_INDEX)
        self.assertFalse(any(character.isdigit() for character in OLD_ARCHITECTURE_FIRMWARE_VERSION_ANSWER))

    def test_v8_3_robokit_connection_shows_mismatched_firmware(self):
        self.assert_old_architecture_robokit_connection_shows_mismatched_firmware(
            {'op': 'call_service', 'id': '/system_information:0', 'service': '/system_information',
             'args': {'request_cli': 'snap list | grep extend-robokit-xarm'}})

    def test_v9_3_and_v10_3_robokit_connection_shows_mismatched_firmware(self):
        self.assert_old_architecture_robokit_connection_shows_mismatched_firmware(
            {'op': 'call_service', 'id': 'system_information:1', 'service': 'system_information',
             'args': {'request_cli': 'rosversion bio_ik_service_examples'}})

    def test_v2025_cortex_handshake_lists_one_device_named_update_amas(self):
        self.receive_binary_frames(
            {'op': 'call_service', 'id': 'cortex_configuration_provider:1', 'service': 'cortex_configuration_provider',
             'args': {'connectionIP': HEADSET_CONNECTION_IP}},
            {'op': 'call_service', 'id': 'load_file:2', 'service': 'load_file',
             'args': {'path': 'configs/robokit/<color=#FF5252>Update AMAS to connect</color>.json'}},
            {'op': 'call_service', 'id': 'load_config_file:3', 'service': 'load_config_file',
             'args': {'path': 'firmware_configs/robokit/9090.env'}})
        self.assertEqual([
            {'op': 'service_response', 'id': 'cortex_configuration_provider:1',
             'service': 'cortex_configuration_provider', 'result': True,
             'values': {'cortexName': 'AMAS_Update_Required', 'cortexConfiguration': [{
                 'name': '<color=#FF5252>Update AMAS to connect</color>', 'deviceType': 'robokit',
                 'connectionIP': HEADSET_CONNECTION_IP, 'rosPort': '9090'}]}},
            {'op': 'service_response', 'id': 'load_file:2', 'service': 'load_file', 'result': True,
             'values': {'contents': ('{`model`: 2, `name`: `<color=#FF5252>Update AMAS to connect</color>`, '
                                     '`rosPort`: `9090`}'),
                        'success': True, 'error': 0}},
            {'op': 'service_response', 'id': 'load_config_file:3', 'service': 'load_config_file', 'result': True,
             'values': {'contents': '', 'success': True, 'error': 0}},
        ], self.sent_messages())
        self.assert_nothing_forwarded()

    def test_v2025_device_connection_never_learns_its_docker_image(self):
        self.receive_binary_frames(
            {'op': 'call_service', 'id': 'rosapi/get_param:1', 'service': 'rosapi/get_param',
             'args': {'name': '/robokit_model', 'default': 'default'}},
            {'op': 'call_service', 'id': 'system_information:2', 'service': 'system_information',
             'args': {'request_cli': 'cat /etc/machine-id'}},
            {'op': 'call_service', 'id': 'system_information:3', 'service': 'system_information',
             'args': {'request_cli': 'echo $DOCKER_IMAGE'}})
        self.assertEqual([
            {'op': 'service_response', 'id': 'rosapi/get_param:1', 'service': 'rosapi/get_param', 'result': True,
             'values': {'value': '"XARM6"'}},
            {'op': 'service_response', 'id': 'system_information:2', 'service': 'system_information',
             'result': True, 'values': {'cli': 'legacyamasupdaterequired'}},
        ], self.sent_messages())
        self.assert_nothing_forwarded()


PKG = 'rosbridge_server'
NAME = 'test_legacy_amas_protocol'

if __name__ == '__main__':
    rospy.init_node(NAME)
    rostest.rosrun(PKG, NAME, TestLegacyAmasProtocol)
