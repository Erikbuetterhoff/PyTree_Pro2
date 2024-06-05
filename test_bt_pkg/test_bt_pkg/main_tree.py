#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
import subtrees
import subtrees.subtree1
import subtrees.subtree_landing


def create_main_root() -> py_trees.behaviour.Behaviour:
    
    subtree_landing = subtrees.subtree_landing.create_subtree_landing()



    return subtree_landing

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