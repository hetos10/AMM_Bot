#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import CollisionObject, AttachedCollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from std_srvs.srv import Trigger


class MagneticGripperNode(Node):
    def __init__(self):
        super().__init__('magnetic_gripper')
        
        # Planning scene client
        self.planning_scene_client = self.create_client(
            ApplyPlanningScene, 
            '/apply_planning_scene'
        )
        
        while not self.planning_scene_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for planning scene service...')
        
        # End effector link
        self.eef_link = "mag_eef"
        
        # Currently attached object
        self.attached_object = None
        
        # Services
        self.attach_srv = self.create_service(Trigger, 'magnet_on', self.magnet_on_callback)
        self.detach_srv = self.create_service(Trigger, 'magnet_off', self.magnet_off_callback)
        
        self.get_logger().info(f"Magnetic gripper node ready on link: {self.eef_link}")

    def magnet_on_callback(self, request, response):
        """Attach crate to gripper in planning scene"""
        object_name = "crate"
        
        try:
            # Create attached collision object message
            attached_object = AttachedCollisionObject()
            attached_object.link_name = self.eef_link
            attached_object.object.id = object_name
            attached_object.object.operation = CollisionObject.ADD
            
            # Define crate as a box
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.2, 0.2, 0.2]  # 20cm cube
            
            # Pose relative to end effector
            pose = PoseStamped()
            pose.header.frame_id = self.eef_link
            pose.pose.position.z = 0.1  # 10cm below gripper
            pose.pose.orientation.w = 1.0
            
            attached_object.object.primitives = [primitive]
            attached_object.object.primitive_poses = [pose.pose]
            
            # Apply to planning scene
            from moveit_msgs.msg import PlanningScene
            scene_msg = PlanningScene()
            scene_msg.robot_state.attached_collision_objects = [attached_object]
            scene_msg.is_diff = True
            
            req = ApplyPlanningScene.Request()
            req.scene = scene_msg
            
            future = self.planning_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.attached_object = object_name
                self.get_logger().info(f"✅ Magnet ON → {object_name} attached to {self.eef_link}")
                response.success = True
                response.message = f"Attached {object_name}"
            else:
                self.get_logger().error("❌ Failed to attach object")
                response.success = False
                response.message = "Failed to attach"
                
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            response.success = False
            response.message = str(e)
        
        return response

    def magnet_off_callback(self, request, response):
        """Detach crate from gripper"""
        if self.attached_object is None:
            response.success = False
            response.message = "No object attached"
            return response
        
        try:
            # Remove attached object
            attached_object = AttachedCollisionObject()
            attached_object.link_name = self.eef_link
            attached_object.object.id = self.attached_object
            attached_object.object.operation = CollisionObject.REMOVE
            
            from moveit_msgs.msg import PlanningScene
            scene_msg = PlanningScene()
            scene_msg.robot_state.attached_collision_objects = [attached_object]
            scene_msg.is_diff = True
            
            req = ApplyPlanningScene.Request()
            req.scene = scene_msg
            
            future = self.planning_scene_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.get_logger().info(f"✅ Magnet OFF → {self.attached_object} detached")
                response.success = True
                response.message = f"Detached {self.attached_object}"
                self.attached_object = None
            else:
                self.get_logger().error("❌ Failed to detach object")
                response.success = False
                response.message = "Failed to detach"
                
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            response.success = False
            response.message = str(e)
        
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MagneticGripperNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()