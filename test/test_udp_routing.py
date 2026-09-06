"""Real production-header multi-process routing. Run in an isolated network namespace."""
import contextlib
import resource
import select
import socket
import struct
import subprocess
import tempfile
import unittest
from pathlib import Path
from test_udp_frames import ENDPOINT, port

class UdpRoutingTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temp.cleanup)
        source = Path(cls.temp.name) / 'endpoint.cpp'
        source.write_text(ENDPOINT)
        cls.binary = str(source.with_suffix(''))
        include = Path(__file__).resolve().parents[1] / 'include'
        subprocess.run(['g++','-std=c++11','-Wall','-Wextra','-Werror','-pthread','-I'+str(include),str(source),'-o',cls.binary],check=True)

    @contextlib.contextmanager
    def endpoint(self, name):
        proc = subprocess.Popen([self.binary,name],stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,
                                preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE,(0,0)))
        try:
            self.assertTrue(select.select([proc.stdout],[],[],3)[0])
            self.assertEqual(proc.stdout.readline().strip(),'ready')
            yield proc
        finally:
            proc.communicate('quit\n',timeout=3)
            self.assertEqual(proc.returncode,0)

    def state(self, proc):
        proc.stdin.write('state\n');proc.stdin.flush()
        self.assertTrue(select.select([proc.stdout],[],[],3)[0])
        return proc.stdout.readline().strip() == '1'

    def apply(self, destination, identity, held):
        with socket.socket(socket.AF_INET,socket.SOCK_DGRAM) as sock:
            sock.settimeout(2)
            sock.sendto(struct.pack('<IBBHI32s',0x58474348,1,held,0,71,identity.encode()),('127.0.0.1',port(destination)))
            ack,_=sock.recvfrom(100)
            self.assertEqual(len(ack),12)
            self.assertEqual(struct.unpack_from('<I',ack,8)[0],71)
            return ack[6]

    def test_process_order_restart_mismatched_id_and_exclusive_binding(self):
        for first,second in [('ugv1','ugv2'),('ugv2','ugv1')]:
            with self.subTest(first=first), self.endpoint(first) as a:
                for _ in range(2):
                    with self.endpoint(second) as b:
                        self.assertEqual(self.apply(first,first,False),0)
                        for held in (True,False,True):
                            self.assertEqual(self.apply(second,second,held),0)
                            self.assertEqual(self.state(b),held)
                            self.assertFalse(self.state(a))
                            self.assertEqual(self.apply(second,first,True),1)
                            self.assertFalse(self.state(a))
                        rejected=subprocess.run([self.binary,first],input='quit\n',capture_output=True,text=True,timeout=3,
                            preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE,(0,0)))
                        self.assertNotEqual(rejected.returncode,0)
                        self.assertIn('is unavailable',rejected.stderr)
                    self.assertEqual(self.apply(first,first,True),0)
                    self.assertTrue(self.state(a))

if __name__ == '__main__': unittest.main()
