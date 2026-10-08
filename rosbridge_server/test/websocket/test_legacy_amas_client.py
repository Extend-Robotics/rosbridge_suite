#!/usr/bin/env python
import base64
import json
import os
import socket
import struct
import time
import unittest

import rosgraph
import rospy
import rostest

REQUIRED_PROTOCOL_PORT_PARAM = '/rosbridge_websocket/actual_port'
NO_REQUIRED_PROTOCOL_PORT_PARAM = '/rosbridge_websocket_without_required_amas_protocol/actual_port'
VISIBLE_PARAM = '/legacy_amas_test/visible_param'
VISIBLE_PARAM_VALUE = 'readable by legacy AMAS'
LEGACY_CONTROL_TOPIC = '/extend_control_command'
CURRENT_AMAS_CONTROL_TOPIC = '/extend_control_command_from_current_amas'
UNRESTRICTED_BRIDGE_CONTROL_TOPIC = '/extend_control_command_without_required_amas_protocol'
SOCKET_TIMEOUT = 5.0  # seconds
NO_REPLY_WAIT = 2.0  # seconds
PUBLISHER_REGISTRATION_TIMEOUT = 5.0  # seconds
SWITCHING_PROTOCOLS = 101
TEXT_OPCODE = 0x1
BINARY_OPCODE = 0x2
FINAL_FRAME_BIT = 0x80
MASK_BIT = 0x80
PAYLOAD_LENGTH_MASK = 0x7f
OPCODE_MASK = 0x0f
SIXTEEN_BIT_LENGTH_MARKER = 126
SIXTY_FOUR_BIT_LENGTH_MARKER = 127
MAXIMUM_SIXTEEN_BIT_LENGTH = 0xffff
END_OF_HTTP_HEADERS = b'\r\n\r\n'


class WebSocketSharpLikeClient(object):
    def __init__(self, port, resource):
        self.connection = socket.create_connection(('127.0.0.1', port), timeout=SOCKET_TIMEOUT)
        self.unread_bytes = b''
        handshake_request = (
            'GET {} HTTP/1.1\r\n'
            'User-Agent: websocket-sharp/1.0\r\n'
            'Upgrade: websocket\r\n'
            'Connection: Upgrade\r\n'
            'Host: 127.0.0.1:{}\r\n'
            'Sec-WebSocket-Key: {}\r\n'
            'Sec-WebSocket-Version: 13\r\n'
            '\r\n').format(resource, port, base64.b64encode(os.urandom(16)).decode('ascii'))
        self.connection.sendall(handshake_request.encode('ascii'))
        while END_OF_HTTP_HEADERS not in self.unread_bytes:
            self.receive_more_bytes()
        handshake_response, self.unread_bytes = self.unread_bytes.split(END_OF_HTTP_HEADERS, 1)
        self.handshake_status_code = int(handshake_response.decode('utf-8').split(' ', 2)[1])

    def receive_more_bytes(self):
        received_bytes = self.connection.recv(4096)
        if not received_bytes:
            raise AssertionError('The bridge closed the connection')
        self.unread_bytes += received_bytes

    def take_bytes(self, byte_count):
        while len(self.unread_bytes) < byte_count:
            self.receive_more_bytes()
        taken_bytes, self.unread_bytes = self.unread_bytes[:byte_count], self.unread_bytes[byte_count:]
        return taken_bytes

    def send_binary_json(self, message):
        payload = bytearray(json.dumps(message).encode('utf-8'))
        mask = bytearray(os.urandom(4))
        header = bytearray([FINAL_FRAME_BIT | BINARY_OPCODE])
        if len(payload) < SIXTEEN_BIT_LENGTH_MARKER:
            header.append(MASK_BIT | len(payload))
        elif len(payload) <= MAXIMUM_SIXTEEN_BIT_LENGTH:
            header.append(MASK_BIT | SIXTEEN_BIT_LENGTH_MARKER)
            header.extend(struct.pack('!H', len(payload)))
        else:
            header.append(MASK_BIT | SIXTY_FOUR_BIT_LENGTH_MARKER)
            header.extend(struct.pack('!Q', len(payload)))
        masked_payload = bytearray(byte ^ mask[index % len(mask)] for index, byte in enumerate(payload))
        self.connection.sendall(bytes(header + mask + masked_payload))

    def receive_json(self, timeout=SOCKET_TIMEOUT):
        self.connection.settimeout(timeout)
        first_byte, second_byte = bytearray(self.take_bytes(2))
        payload_length = second_byte & PAYLOAD_LENGTH_MASK
        if payload_length == SIXTEEN_BIT_LENGTH_MARKER:
            payload_length = struct.unpack('!H', self.take_bytes(2))[0]
        elif payload_length == SIXTY_FOUR_BIT_LENGTH_MARKER:
            payload_length = struct.unpack('!Q', self.take_bytes(8))[0]
        payload = self.take_bytes(payload_length)
        opcode = first_byte & OPCODE_MASK
        if opcode not in (TEXT_OPCODE, BINARY_OPCODE):
            raise AssertionError('Expected a data frame, got opcode {}'.format(opcode))
        return json.loads(payload.decode('utf-8'))

    def assert_no_message_within(self, seconds):
        try:
            unexpected_message = self.receive_json(timeout=seconds)
        except socket.timeout:
            return
        raise AssertionError('Expected no message, got {}'.format(unexpected_message))

    def close(self):
        self.connection.close()


def topic_publishers(topic):
    publishers, _subscribers, _services = rosgraph.Master(NAME).getSystemState()
    return dict(publishers).get(topic, [])


def advertised_services():
    _publishers, _subscribers, services = rosgraph.Master(NAME).getSystemState()
    return [service for service, _providers in services]


class TestLegacyAmasClient(unittest.TestCase):
    def connect(self, port_param, resource):
        client = WebSocketSharpLikeClient(rospy.get_param(port_param), resource)
        self.addCleanup(client.close)
        self.assertEqual(SWITCHING_PROTOCOLS, client.handshake_status_code)
        return client

    def send_get_param(self, client, call_id):
        client.send_binary_json({'op': 'call_service', 'id': call_id, 'service': 'rosapi/get_param',
                                 'args': {'name': VISIBLE_PARAM, 'default': 'default'}})

    def assert_get_param_answered(self, client, call_id):
        self.assertEqual({'op': 'service_response', 'id': call_id, 'service': 'rosapi/get_param',
                          'values': {'value': json.dumps(VISIBLE_PARAM_VALUE)}, 'result': True},
                         client.receive_json())

    def advertise_and_publish_control_command(self, client, topic):
        client.send_binary_json({'op': 'advertise', 'topic': topic, 'type': 'std_msgs/String'})
        client.send_binary_json({'op': 'publish', 'topic': topic, 'msg': {'data': 'move the robot'}})

    def wait_for_publisher_registration(self, topic):
        deadline = time.time() + PUBLISHER_REGISTRATION_TIMEOUT
        while not topic_publishers(topic) and time.time() < deadline:
            time.sleep(0.1)
        return topic_publishers(topic)

    def test_legacy_client_publish_creates_no_publisher(self):
        client = self.connect(REQUIRED_PROTOCOL_PORT_PARAM, '/')
        self.advertise_and_publish_control_command(client, LEGACY_CONTROL_TOPIC)
        self.send_get_param(client, 'after_publish')
        self.assert_get_param_answered(client, 'after_publish')
        self.assertEqual([], self.wait_for_publisher_registration(LEGACY_CONTROL_TOPIC))

    def test_legacy_client_get_param_is_forwarded_and_set_param_is_dropped(self):
        client = self.connect(REQUIRED_PROTOCOL_PORT_PARAM, '/?amas_protocol=1')
        client.send_binary_json({'op': 'call_service', 'id': 'set_param', 'service': 'rosapi/set_param',
                                 'args': {'name': VISIBLE_PARAM, 'value': json.dumps('changed by legacy AMAS')}})
        self.send_get_param(client, 'get_param')
        self.assert_get_param_answered(client, 'get_param')
        client.assert_no_message_within(NO_REPLY_WAIT)
        self.assertEqual(VISIBLE_PARAM_VALUE, rospy.get_param(VISIBLE_PARAM))

    def test_legacy_client_is_told_to_update_instead_of_advertising_raise_operator_error(self):
        client = self.connect(REQUIRED_PROTOCOL_PORT_PARAM, '/')
        client.send_binary_json({'op': 'advertise_service', 'id': None, 'service': 'raise_operator_error',
                                 'type': 'extend_msgs/RaiseOperatorError'})
        self.assertEqual({
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
        }, client.receive_json())
        self.assertNotIn('/raise_operator_error', advertised_services())

    def test_legacy_client_gets_a_fake_cortex_whose_device_is_named_update_amas(self):
        client = self.connect(REQUIRED_PROTOCOL_PORT_PARAM, '/')
        client.send_binary_json({'op': 'call_service', 'id': 'cortex_configuration_provider:1',
                                 'service': 'cortex_configuration_provider',
                                 'args': {'connectionIP': '127.0.0.1'}})
        self.assertEqual({
            'op': 'service_response',
            'id': 'cortex_configuration_provider:1',
            'service': 'cortex_configuration_provider',
            'values': {'cortexName': 'AMAS_Update_Required', 'cortexConfiguration': [{
                'name': '<color=#FF5252>Update AMAS to connect</color>', 'deviceType': 'robokit',
                'connectionIP': '127.0.0.1', 'rosPort': str(rospy.get_param(REQUIRED_PROTOCOL_PORT_PARAM))}]},
            'result': True,
        }, client.receive_json())

    def test_legacy_client_never_gets_its_docker_image(self):
        client = self.connect(REQUIRED_PROTOCOL_PORT_PARAM, '/')
        client.send_binary_json({'op': 'call_service', 'id': 'system_information:1', 'service': 'system_information',
                                 'args': {'request_cli': 'echo $DOCKER_IMAGE'}})
        client.send_binary_json({'op': 'call_service', 'id': 'system_information:2', 'service': 'system_information',
                                 'args': {'request_cli': 'cat /etc/machine-id'}})
        self.assertEqual({'op': 'service_response', 'id': 'system_information:2', 'service': 'system_information',
                          'values': {'cli': 'legacyamasupdaterequired'}, 'result': True},
                         client.receive_json())
        client.assert_no_message_within(NO_REPLY_WAIT)

    def test_current_amas_client_is_not_restricted(self):
        client = self.connect(REQUIRED_PROTOCOL_PORT_PARAM, '/?amas_protocol=2')
        self.advertise_and_publish_control_command(client, CURRENT_AMAS_CONTROL_TOPIC)
        self.assertEqual(['/rosbridge_websocket'], self.wait_for_publisher_registration(CURRENT_AMAS_CONTROL_TOPIC))

    def test_any_client_is_unrestricted_when_no_protocol_is_required(self):
        client = self.connect(NO_REQUIRED_PROTOCOL_PORT_PARAM, '/')
        self.advertise_and_publish_control_command(client, UNRESTRICTED_BRIDGE_CONTROL_TOPIC)
        self.assertEqual(['/rosbridge_websocket_without_required_amas_protocol'],
                         self.wait_for_publisher_registration(UNRESTRICTED_BRIDGE_CONTROL_TOPIC))


PKG = 'rosbridge_server'
NAME = 'test_legacy_amas_client'

if __name__ == '__main__':
    rospy.init_node(NAME)
    rospy.set_param(VISIBLE_PARAM, VISIBLE_PARAM_VALUE)
    rospy.wait_for_service('/rosapi/get_param')

    while not rospy.is_shutdown() and not (rospy.has_param(REQUIRED_PROTOCOL_PORT_PARAM) and
                                           rospy.has_param(NO_REQUIRED_PROTOCOL_PORT_PARAM)):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestLegacyAmasClient)
