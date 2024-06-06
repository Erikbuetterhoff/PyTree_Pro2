#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
import subtrees_full
import subtrees_full.subtree_april
import subtrees_full.subtree_gesture
import subtrees_full.subtree_collision
import subtrees_full.subtree_landing
import subtrees_full.subtree_lidar
import subtrees_full.subtree_start
import subtrees_full.subtree_waypoint
import subtrees_full.subtree_zone



def create_main_root() -> py_trees.behaviour.Behaviour:
    
    subtree_start = subtrees_full.subtree_start.subtree_start()
    subtree_landing = subtrees_full.subtree_landing.subtree_landing()
    subtree_zone = subtrees_full.subtree_zone.subtree_zone()
    subtree_collision = subtrees_full.subtree_collision.subtree_collision()
    subtree_waypoint = subtrees_full.subtree_waypoint.subtree_waypoint()
    subtree_lidar = subtrees_full.subtree_lidar.subtree_lidar()
    subtree_gesture = subtrees_full.subtree_gesture.subtree_gesture()
    subtree_april = subtrees_full.subtree_april.subtree_april()



    return subtree_start

def main():
    rclpy.init()

    root =create_main_root()
    tree = py_trees_ros.trees.BehaviourTree(
        root=root,
        unicode_tree_debug=True
    )
    try:
        tree.setup(timeout=15.0)
    except py_trees_ros.exceptions.TimedOutError as e:
        console.logerror(console.red + "failed to setup the tree, aborting [{}]".format(str(e)) + console.reset)
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)
    except KeyboardInterrupt:
        # not a warning, nor error, usually a user-initiated shutdown
        console.logerror("tree setup interrupted")
        tree.shutdown()
        rclpy.try_shutdown()
        sys.exit(1)


    tree.tick_tock(period_ms=1000.0)

    try:
        rclpy.spin(tree.node)
    except KeyboardInterrupt:
        tree.shutdown()
        rclpy.shutdown()
    


if __name__ == '__main__':
    main()