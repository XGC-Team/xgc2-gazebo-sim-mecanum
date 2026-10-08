"""Bound simulation-v1 operations for isolated physical regression fixtures."""
import json
from types import SimpleNamespace
from urllib.parse import quote
import rospy
from geometry_msgs.msg import Pose, Twist
from xgc2_xrpc.runtime import Runtime
from xgc2_scene_runtime.simulation_client import SimulationClient

class NativeSimulation:
    def __init__(self):
        self.runtime = Runtime(blocking_workers=1, max_calls=4)
        self.native = SimulationClient(json.loads(rospy.get_param('~simulation_service_ref_json')),
            runtime=self.runtime, local_target=rospy.get_param('~target_id'), timeout=10)
    def close(self):
        self.native.close()
        self.runtime.close()
    def query(self, route):
        return self.native.client.json(route, method='GET', timeout=2)
    def entity(self, name):
        return self.query('/v1/entities/'+quote(name, safe=''))['entities'][0]
    def model_state(self, name, frame='world'):
        if frame!='world': raise ValueError('native regression accepts only world frame')
        state=self.entity(name)['state']; pose=Pose(); twist=Twist()
        pose.position.x,pose.position.y,pose.position.z=state['pose']['position']
        pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w=state['pose']['orientation']
        twist.linear.x,twist.linear.y,twist.linear.z=state['twist']['linear']
        twist.angular.x,twist.angular.y,twist.angular.z=state['twist']['angular']
        return SimpleNamespace(success=True, status_message='native snapshot', pose=pose, twist=twist)
    def pause(self): self.native.operation('/v1/world/pause', {})
    def resume(self): self.native.operation('/v1/world/resume', {})
    def hold(self, name, held):
        status=self.query('/v1/chassis/hold')
        result=self.native.client.json('/v1/chassis/hold', {'expected_revision':status['revision'],
            'changes':[{'robot_id':name, 'held':bool(held)}]}, timeout=5)
        if result.get('stage')!='applied': raise RuntimeError('native HOLD did not apply: '+str(result))
        if held and not result['changes'][0]['zero_applied']: raise RuntimeError('native zero sink did not complete')
        return result
    def set_model(self, state):
        entity=self.entity(state.model_name)
        p,q,t=state.pose.position,state.pose.orientation,state.twist
        self.native.operation('/v1/entities/'+quote(state.model_name,safe='')+'/state',
            {'generation':entity['ref']['generation'], 'state':{'pose':{'position':[p.x,p.y,p.z],
            'orientation':[q.x,q.y,q.z,q.w]},'twist':{'linear':[t.linear.x,t.linear.y,t.linear.z],
            'angular':[t.angular.x,t.angular.y,t.angular.z]}}})
        return SimpleNamespace(success=True)
    def spawn_model(self, name, source, namespace, pose, frame):
        if frame!='world': raise ValueError('native regression accepts only world frame')
        p,q=pose.position,pose.orientation
        entity={'id':name,'role':'robot',
            'pose':{'position':[p.x,p.y,p.z],'orientation':[q.x,q.y,q.z,q.w]},
            'asset':{'id':name,'realization':{'media_type':'application/sdf+xml','content':source}}}
        if namespace: entity['parameters']={'ros_namespace':namespace}
        self.native.operation('/v1/entities', {'entity':entity})
        return SimpleNamespace(success=True)
    def delete_model(self, name):
        entity=self.entity(name)
        self.native.operation('/v1/entities/'+quote(name,safe=''), {'generation':entity['ref']['generation']}, method='DELETE')
        return SimpleNamespace(success=True)
