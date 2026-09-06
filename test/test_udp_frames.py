"""Black-box HOLD framing regression; run only in a disposable test environment.

Compile the production header without ROS or socket mocks. Refuse to start if
the robot endpoint is already occupied, and send test traffic exclusively to loopback.
"""
import itertools
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import tempfile
import unittest


def port(robot):
    value = 2166136261
    for byte in robot.encode():
        value = ((value ^ byte) * 16777619) & 0xffffffff
    return 20000 + value % 20000


ENDPOINT = r'''
#include "xgc_chassis_hold/udp.hpp"
#include <iostream>
int main(int argc, char** argv) {
  xgc_chassis_hold::Gate target(argc > 1 ? argv[1] : "ugv1");
  auto& hub = xgc_chassis_hold::Hub::instance();
  hub.add(&target);
  std::cout << "ready" << std::endl;
  std::string line;
  while (std::getline(std::cin, line) && line != "quit") {
    std::cout << target.held() << std::endl;
  }
  hub.remove(&target);
}
'''


class UdpFrameTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        temporary = tempfile.TemporaryDirectory(prefix="hold-frame-test-")
        cls.addClassCleanup(temporary.cleanup)
        directory = Path(temporary.name)
        source = directory / "endpoint.cpp"
        source.write_text(ENDPOINT)
        cls.endpoint = directory / "endpoint"
        include = Path(__file__).resolve().parents[1] / "include"
        subprocess.run(
            [os.environ.get("CXX", "g++"), "-std=c++11", "-Wall", "-Wextra",
             "-Werror", "-pthread", "-I" + str(include), str(source),
             "-o", str(cls.endpoint)], check=True, timeout=30)

    def setUp(self):
        # Never join an existing production/test listener through SO_REUSEADDR.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(("0.0.0.0", port("ugv1")))
        self.process = subprocess.Popen(
            [str(self.endpoint)], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True)
        self.addCleanup(self.stop_endpoint)
        self.assertEqual(self.read_line(), "ready")
        self.client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addCleanup(self.client.close)
        self.client.bind(("127.0.0.1", 0))
        self.client.settimeout(2)
        self.sequence = itertools.count(1)

    def stop_endpoint(self):
        try:
            self.process.communicate("quit\n", timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.communicate()
            self.fail("HOLD endpoint did not stop")
        self.assertEqual(self.process.returncode, 0)

    def read_line(self):
        self.assertTrue(select.select([self.process.stdout], [], [], 3)[0],
                        "HOLD endpoint response timed out")
        return self.process.stdout.readline().strip()

    @staticmethod
    def frame(sequence, held, robot=b"ugv1"):
        return struct.pack("<IBBHI32s", 0x58474348, 1, held, 0, sequence, robot)

    def exchange(self, packet):
        barrier = next(self.sequence)
        self.client.sendto(packet, ("127.0.0.1", port("ugv1")))
        self.client.sendto(self.frame(barrier, False, b"audit_barrier"),
                           ("127.0.0.1", port("ugv1")))
        acknowledgements = []
        # An unmatched-ID request to the SAME endpoint is an ordered no-op. This
        # avoids sleep-based assertions that a malformed frame was ignored.
        while True:
            ack, sender = self.client.recvfrom(1024)
            self.assertEqual(sender, ("127.0.0.1", port("ugv1")))
            self.assertEqual(len(ack), 12)
            self.assertEqual(ack[:5], struct.pack("<IB", 0x58474348, 1))
            request = struct.unpack_from("<I", ack, 8)[0]
            if request == barrier:
                self.assertEqual(ack[6], 1)
                break
            self.assertEqual(ack[6], 0)
            acknowledgements.append((request, bool(ack[5])))
        self.process.stdin.write("state\n")
        self.process.stdin.flush()
        return acknowledgements, self.read_line() == "1"

    def test_exact_length_and_rejection_without_state_change(self):
        for held in (False, True):
            for size in (0, 1, 43, 45, 4096):
                with self.subTest(held=held, invalid_size=size):
                    request = next(self.sequence)
                    self.assertEqual(self.exchange(self.frame(request, held)),
                                     ([(request, held)], held))
                    request = next(self.sequence)
                    malformed = self.frame(request, not held)
                    malformed = (malformed + b"x" * size)[:size]
                    self.assertEqual(self.exchange(malformed), ([], held))


if __name__ == "__main__":
    unittest.main()
