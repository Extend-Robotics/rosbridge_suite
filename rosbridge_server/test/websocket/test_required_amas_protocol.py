#!/usr/bin/env python
import base64
import os
import rostest
import socket
import unittest

import rospy

REQUIRED_PROTOCOL_PORT_PARAM = '/rosbridge_websocket/actual_port'
NO_REQUIRED_PROTOCOL_PORT_PARAM = '/rosbridge_websocket_without_required_amas_protocol/actual_port'
HANDSHAKE_TIMEOUT = 5.0  # seconds
SWITCHING_PROTOCOLS = 101
FORBIDDEN = 403


def websocket_sharp_handshake_status(port, resource):
    handshake_request = (
        'GET {} HTTP/1.1\r\n'
        'User-Agent: websocket-sharp/1.0\r\n'
        'Upgrade: websocket\r\n'
        'Connection: Upgrade\r\n'
        'Host: 127.0.0.1:{}\r\n'
        'Sec-WebSocket-Key: {}\r\n'
        'Sec-WebSocket-Version: 13\r\n'
        '\r\n').format(resource, port, base64.b64encode(os.urandom(16)).decode('ascii'))
    connection = socket.create_connection(('127.0.0.1', port), timeout=HANDSHAKE_TIMEOUT)
    try:
        connection.sendall(handshake_request.encode('ascii'))
        status_line = connection.makefile('rb').readline().decode('utf-8')
    finally:
        connection.close()
    _http_version, status_code, reason = status_line.rstrip('\r\n').split(' ', 2)
    return int(status_code), reason


class TestRequiredAmasProtocol(unittest.TestCase):
    def assert_rejected_with_update_hint(self, resource):
        status_code, reason = websocket_sharp_handshake_status(rospy.get_param(REQUIRED_PROTOCOL_PORT_PARAM), resource)
        self.assertEqual(FORBIDDEN, status_code)
        self.assertIn('update AMAS', reason)

    def assert_accepted(self, port_param, resource):
        status_code, _reason = websocket_sharp_handshake_status(rospy.get_param(port_param), resource)
        self.assertEqual(SWITCHING_PROTOCOLS, status_code)

    def test_missing_protocol_is_rejected(self):
        self.assert_rejected_with_update_hint('/')

    def test_older_protocol_is_rejected(self):
        self.assert_rejected_with_update_hint('/?amas_protocol=1')

    def test_non_integer_protocol_is_rejected(self):
        self.assert_rejected_with_update_hint('/?amas_protocol=two')

    def test_repeated_protocol_is_rejected(self):
        self.assert_rejected_with_update_hint('/?amas_protocol=2&amas_protocol=3')

    def test_required_protocol_is_accepted(self):
        self.assert_accepted(REQUIRED_PROTOCOL_PORT_PARAM, '/?amas_protocol=2')

    def test_newer_protocol_is_accepted(self):
        self.assert_accepted(REQUIRED_PROTOCOL_PORT_PARAM, '/?amas_protocol=3')

    def test_client_without_protocol_is_accepted_when_no_protocol_is_required(self):
        self.assert_accepted(NO_REQUIRED_PROTOCOL_PORT_PARAM, '/')


PKG = 'rosbridge_server'
NAME = 'test_required_amas_protocol'

if __name__ == '__main__':
    rospy.init_node(NAME)

    while not rospy.is_shutdown() and not (rospy.has_param(REQUIRED_PROTOCOL_PORT_PARAM) and
                                           rospy.has_param(NO_REQUIRED_PROTOCOL_PORT_PARAM)):
        rospy.sleep(1.0)

    rostest.rosrun(PKG, NAME, TestRequiredAmasProtocol)
