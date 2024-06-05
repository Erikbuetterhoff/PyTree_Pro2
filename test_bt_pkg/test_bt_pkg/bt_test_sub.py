#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
from std_msgs.msg import Bool

import action_pkg.action as actions



def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    
    root = py_trees.composites.Sequence(name="Sequece Dronecheck")
   
    drone_not_ok = py_trees_ros.action_clients.FromConstant(
        name="Return Home",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    drone_ok = py_trees_ros.subscribers.CheckData(
        name="Drohne okay?", 
        topic_name="bt_test_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    root.add_child(drone_ok)
    root.add_child(drone_not_ok)

    return root

def main():
    rclpy.init()

    root = tutorial_create_root()
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