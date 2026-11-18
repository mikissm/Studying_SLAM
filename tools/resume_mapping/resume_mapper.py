#!/usr/bin/env python3
"""Saved PCD + FAST-LIO odometry continuation. Does not restore a PGO graph."""
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from std_msgs.msg import Header, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster
from message_filters import Subscriber, ApproximateTimeSynchronizer
from core import MapStore, align, pose_matrix, transform


class ResumeMapper(Node):
    def __init__(self):
        super().__init__('resume_mapper')
        defaults={'map_path':'','output_path':'/home/ssm/maps/resumed/map.pcd',
                  'resolution':.1,'max_voxels':2000000,'update_hz':2.,
                  'initial_x':0.,'initial_y':0.,'initial_z':0.,'initial_yaw':0.,
                  'cloud_topic':'/fastlio2/body_cloud','odom_topic':'/fastlio2/lio_odom',
                  'map_frame':'map','odom_frame':'odom','body_frame':'body',
                  'min_overlap':.55,'max_rmse':.15,'max_scan_age':1.}
        for k,v in defaults.items():self.declare_parameter(k,v)
        self.p={k:self.get_parameter(k).value for k in defaults}
        if self.p['update_hz']<=0 or not 0<self.p['min_overlap']<=1 or self.p['max_rmse']<=0 or self.p['max_scan_age']<=0:
            raise ValueError('잘못된 주기/정합 파라미터')
        self.store=MapStore(self.p['map_path'],self.p['resolution'],self.p['max_voxels'])
        self.target=self.store.points();self.offset=None;self.latest=None;self.last_stamp=None
        self.initial=pose_matrix(*[self.p['initial_'+k] for k in ['x','y','z','yaw']])
        self.diagnostics={};self.error='초기 위치 확인 후 /resume_mapping/start 호출 필요'
        self.pub=self.create_publisher(PointCloud2,'/resume_mapping/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.status_pub=self.create_publisher(String,'/resume_mapping/status',10)
        self.tf=TransformBroadcaster(self)
        self.cloud_sub=Subscriber(self,PointCloud2,self.p['cloud_topic'],qos_profile=10)
        self.odom_sub=Subscriber(self,Odometry,self.p['odom_topic'],qos_profile=10)
        self.sync=ApproximateTimeSynchronizer([self.cloud_sub,self.odom_sub],10,.03)
        self.sync.registerCallback(self.receive)
        self.pose_sub=self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial_pose,10)
        self.start_srv=self.create_service(Trigger,'/resume_mapping/start',self.start)
        self.pause_srv=self.create_service(Trigger,'/resume_mapping/pause',self.pause)
        self.save_srv=self.create_service(Trigger,'/resume_mapping/save',self.save)
        self.active=False
        self.timer=self.create_timer(1/self.p['update_hz'],self.tick)
        self.publish_map();self.get_logger().info('Saved map loaded; set initial pose, then call /resume_mapping/start. PGO must be stopped.')

    def initial_pose(self,msg):
        if msg.header.frame_id!=self.p['map_frame']:
            self.error='초기 위치 frame은 '+self.p['map_frame']+'이어야 합니다';return
        self.active=False;self.offset=None
        p=msg.pose.pose
        q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        if not np.isfinite(q+[p.position.x,p.position.y,p.position.z]).all() or np.linalg.norm(q)<1e-6:
            self.error='유효하지 않은 초기 위치';return
        self.initial=np.eye(4);self.initial[:3,:3]=Rotation.from_quat(q).as_matrix()
        self.initial[:3,3]=[p.position.x,p.position.y,p.position.z]
        self.error='초기 위치 설정됨; start 호출 필요'

    def receive(self,cloud,odom):
        if cloud.header.frame_id!=self.p['body_frame'] or odom.header.frame_id!=self.p['odom_frame'] or odom.child_frame_id!=self.p['body_frame']:
            self.error='점군/odom frame 불일치';self.latest=None;return
        p=odom.pose.pose;q=[p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w]
        if not np.isfinite(q+[p.position.x,p.position.y,p.position.z]).all() or np.linalg.norm(q)<1e-6:return
        pose=np.eye(4);pose[:3,:3]=Rotation.from_quat(q).as_matrix();pose[:3,3]=[p.position.x,p.position.y,p.position.z]
        data=point_cloud2.read_points_numpy(cloud,field_names=('x','y','z'),skip_nans=True).reshape(-1,3)
        self.latest=(data,pose,cloud.header.stamp)

    def fresh(self):
        if self.latest is None:raise ValueError('동기화된 라이다/odom이 없습니다')
        stamp=self.latest[2];age=(self.get_clock().now().nanoseconds-(stamp.sec*1000000000+stamp.nanosec))/1e9
        if age>self.p['max_scan_age'] or age<-.1:raise ValueError('점군 시간이 오래됐거나 ROS 시계가 일치하지 않습니다')
        return self.latest

    def start(self,req,res):
        try:
            scan,odom,stamp=self.fresh()
            if self.offset is None:
                # Initial guess is the BODY pose in the saved-map frame now.
                target=self.target[np.linalg.norm(self.target-self.initial[:3,3],axis=1)<15]
                body,self.diagnostics=align(scan,target,self.initial,min_overlap=self.p['min_overlap'],max_rmse=self.p['max_rmse'])
                self.offset=body@np.linalg.inv(odom)
            self.active=True;self.error='';res.success=True;res.message='정합 승인; 누적 시작 '+json.dumps(self.diagnostics)
        except Exception as e:res.success=False;res.message=str(e);self.error=str(e)
        return res

    def pause(self,req,res):
        self.active=False;res.success=True;res.message='누적 일시 정지';return res

    def save(self,req,res):
        try:res.message=self.store.save(self.p['output_path']);res.success=True
        except Exception as e:res.message=str(e);res.success=False
        return res

    def publish_map(self):
        header=Header();header.frame_id=self.p['map_frame'];header.stamp=self.get_clock().now().to_msg()
        self.pub.publish(point_cloud2.create_cloud_xyz32(header,self.store.points()))

    def tick(self):
        try:
            if self.offset is not None:
                _,_,stamp=self.fresh()
                tf=TransformStamped();tf.header.frame_id=self.p['map_frame'];tf.child_frame_id=self.p['odom_frame'];tf.header.stamp=stamp
                q=Rotation.from_matrix(self.offset[:3,:3]).as_quat();t=self.offset[:3,3]
                tf.transform.translation.x=float(t[0]);tf.transform.translation.y=float(t[1]);tf.transform.translation.z=float(t[2])
                tf.transform.rotation.x=float(q[0]);tf.transform.rotation.y=float(q[1]);tf.transform.rotation.z=float(q[2]);tf.transform.rotation.w=float(q[3]);self.tf.sendTransform(tf)
            if self.active:
                scan,odom,stamp=self.fresh();key=(stamp.sec,stamp.nanosec)
                if key!=self.last_stamp:
                    self.store.add(transform(scan,self.offset@odom));self.last_stamp=key;self.publish_map()
        except Exception as e:self.active=False;self.error=str(e)
        status=String();status.data=json.dumps({'active':self.active,'aligned':self.offset is not None,'voxels':len(self.store.cells),'error':self.error,**self.diagnostics});self.status_pub.publish(status)


def main():
    rclpy.init();node=None
    try:node=ResumeMapper();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        if node is not None:node.destroy_node()
        rclpy.try_shutdown()

if __name__=='__main__':main()
