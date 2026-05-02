#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import (
    CollisionObject,
    PlanningScene,
    AttachedCollisionObject
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class PlanningSceneManager(Node):
    def __init__(self):
        super().__init__('planning_scene_manager')

        self.publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

    #function to create a standard box - used as a helper and for floor/wall objects
    def create_box(self, name, size, position, orientation=(0.0, 0.0, 0.0, 1.0)):
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = list(map(float, size))

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, position)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = orientation

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    #function tocreate open containers with 1 inch thick walls
    def create_open_container(self, name, outer_size, base_pos, t=0.0254):
        lx, ly, lz = outer_size
        bx, by, bz = base_pos

        objs = []

        # Bottom
        objs.append(self.create_box(
            f"{name}_bottom",
            [lx, ly, t],
            [bx, by, bz + t/2]
        ))

        wall_z = bz + t + lz/2
        inner_ly = ly - 2*t

        # Side walls
        objs.append(self.create_box(
            f"{name}_left",
            [lx, t, lz],
            [bx, by + (ly/2 - t/2), wall_z]
        ))
        objs.append(self.create_box(
            f"{name}_right",
            [lx, t, lz],
            [bx, by - (ly/2 - t/2), wall_z]
        ))

        # Front/back
        objs.append(self.create_box(
            f"{name}_front",
            [t, inner_ly, lz],
            [bx + (lx/2 - t/2), by, wall_z]
        ))
        objs.append(self.create_box(
            f"{name}_back",
            [t, inner_ly, lz],
            [bx - (lx/2 - t/2), by, wall_z]
        ))

        return objs

    #function to attach a claw object to the tool plate, a cylinder with 
    #fixed height and radius
    def create_attached_claw(self, length, radius):
        attached = AttachedCollisionObject()
        attached.link_name = "tool0" 

        attached.object.header.frame_id = "tool0"
        attached.object.id = "claw"

        shape = SolidPrimitive()
        shape.type = SolidPrimitive.CYLINDER
        shape.dimensions = [float(length), float(radius)]

        pose = Pose()
        pose.position.z = length / 2.0
        pose.orientation.w = 1.0

        attached.object.primitives.append(shape)
        attached.object.primitive_poses.append(pose)
        attached.object.operation = CollisionObject.ADD

        attached.touch_links = ["tool0", "wrist_3_link"]

        return attached

    #function to publish the collision objects within the planning scene
    def publish_scene(self, world_objects=None, attached_objects=None):
        scene = PlanningScene()
        scene.is_diff = True

        if world_objects:
            scene.world.collision_objects.extend(world_objects)

        if attached_objects:
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects.extend(attached_objects)

        self.publisher.publish(scene)


#main, call functions to create specific sized objects and print confirmation to terminal
def main():
    rclpy.init()
    node = PlanningSceneManager()

    rclpy.spin_once(node, timeout_sec=1.0)

    print("\nAdding world objects...\n")

    world = []

    # Wall
    world.append(node.create_box(
        "back_wall",
        [0.1, 3.0, 2.5],
        [-0.36, 0.0, 1.25]
    ))

    # Floor
    world.append(node.create_box(
        "floor",
        [2.0, 2.1, 0.01],
        [0.0, 0.95, -0.01]
    ))

    # Containers
    world.extend(node.create_open_container(
        "fluidized_bed",
        [0.59, 0.97, 0.76],
        [0.25, -0.8, -0.215]
    ))

    world.extend(node.create_open_container(
        "substrate_tank",
        [0.31, 0.62, 0.4064],
        [0.16, 0.61, 0.0004]
    ))

    # Publish world first
    node.publish_scene(world_objects=world)

    # Give MoveIt time to register world
    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.2)

    print("Attaching claw...\n")

    claw = node.create_attached_claw(
        length=0.229,
        radius=0.0762
    )

    # Publish attached claw object separately to prevent timing issues
    node.publish_scene(attached_objects=[claw])

    for _ in range(5):
        rclpy.spin_once(node, timeout_sec=0.2)

    print("Complete, check RViz planning window to see collision objects")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()