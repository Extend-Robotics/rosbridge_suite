#!/usr/bin/env python
from __future__ import unicode_literals

import json
import unittest

import rostest

from rosbridge_library.capabilities.defragmentation import Defragment
from rosbridge_library.protocol import Protocol


class RecordingProtocol(Protocol):
    def __init__(self):
        Protocol.__init__(self, "test_protocol_incoming")
        self.dispatched_messages = []
        self.logged_messages = []
        self.operations["echo"] = self.dispatched_messages.append

    def log(self, level, message, lid=None):
        self.logged_messages.append((level, message, lid))


def echo_json(message_id, **extra_fields):
    return json.dumps(dict(op="echo", id=message_id, **extra_fields), ensure_ascii=False)


def fragment_json(fragmented_message_id, fragment_number, total_fragments, fragment_data):
    return json.dumps(dict(op="fragment", id=fragmented_message_id, num=fragment_number,
                           total=total_fragments, data=fragment_data))


class TestProtocolIncoming(unittest.TestCase):
    def setUp(self):
        self.protocol = RecordingProtocol()

    def dispatched_ids(self):
        return [message["id"] for message in self.protocol.dispatched_messages]

    def logged_errors(self):
        return [message for level, message, _ in self.protocol.logged_messages if level == "error"]

    def test_text_frame_is_dispatched(self):
        self.protocol.incoming(echo_json("text"))

        self.assertEqual(["text"], self.dispatched_ids())
        self.assertEqual("", self.protocol.buffer)

    def test_binary_frame_is_decoded_as_utf8_and_dispatched(self):
        self.protocol.incoming(echo_json("binary", data="héllo").encode("utf-8"))

        self.assertEqual(["binary"], self.dispatched_ids())
        self.assertEqual("héllo", self.protocol.dispatched_messages[0]["data"])
        self.assertEqual("", self.protocol.buffer)

    def test_invalid_utf8_binary_frame_is_logged_and_does_not_poison_buffer(self):
        self.protocol.incoming(b"\xff" + echo_json("invalid").encode("utf-8"))

        self.assertEqual([], self.dispatched_ids())
        self.assertEqual(1, len(self.logged_errors()))
        self.assertEqual("", self.protocol.buffer)

        self.protocol.incoming(echo_json("after_invalid"))
        self.assertEqual(["after_invalid"], self.dispatched_ids())

    def test_two_objects_in_one_text_frame_are_both_dispatched_in_order(self):
        self.protocol.incoming(echo_json("first") + echo_json("second"))

        self.assertEqual(["first", "second"], self.dispatched_ids())
        self.assertEqual("", self.protocol.buffer)

    def test_two_objects_in_one_binary_frame_are_both_dispatched_in_order(self):
        self.protocol.incoming((echo_json("first") + echo_json("second")).encode("utf-8"))

        self.assertEqual(["first", "second"], self.dispatched_ids())
        self.assertEqual("", self.protocol.buffer)

    def test_text_json_split_across_calls_is_dispatched_once_complete(self):
        whole_message = echo_json("split")
        self.protocol.incoming(whole_message[:10])
        self.assertEqual([], self.dispatched_ids())

        self.protocol.incoming(whole_message[10:])
        self.assertEqual(["split"], self.dispatched_ids())
        self.assertEqual("", self.protocol.buffer)

    def test_binary_json_split_across_calls_is_dispatched_once_complete(self):
        whole_message = echo_json("split").encode("utf-8")
        self.protocol.incoming(whole_message[:10])
        self.assertEqual([], self.dispatched_ids())

        self.protocol.incoming(whole_message[10:])
        self.assertEqual(["split"], self.dispatched_ids())
        self.assertEqual("", self.protocol.buffer)

    def test_message_reassembled_from_binary_fragments_is_dispatched(self):
        Defragment(self.protocol)
        whole_message = echo_json("reassembled")
        fragmented_message_id = "test_protocol_incoming_fragments"

        self.protocol.incoming(fragment_json(fragmented_message_id, 0, 2, whole_message[:10]).encode("utf-8"))
        self.assertEqual([], self.dispatched_ids())

        self.protocol.incoming(fragment_json(fragmented_message_id, 1, 2, whole_message[10:]).encode("utf-8"))
        self.assertEqual(["reassembled"], self.dispatched_ids())


PKG = "rosbridge_library"
NAME = "test_protocol_incoming"
if __name__ == "__main__":
    rostest.unitrun(PKG, NAME, TestProtocolIncoming)
