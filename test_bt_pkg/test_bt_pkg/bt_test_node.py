#!/usr/bin/env python3

import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
#import extend_tree as test_extend



def tutorial_create_root() -> py_trees.behaviour.Behaviour:
    
    root = py_trees.composites.Parallel(name="Parallel Dronecheck")

    drone_not_ok = py_trees.composites.Parallel(name= "Parallel Dronecheck fail")
    drone_ok = py_trees.behaviours.Success("Drohne okay?")
    hpl_check = py_trees.composites.Sequence("Sequence HPL")
    
    hpl_ok = py_trees.behaviours.Success("HPL möglich")
    hotpoint_landing = py_trees.behaviours.Success("hotpoint")
    para_landing = py_trees.behaviours.Success("para_landing")
    
    #extend = test_extend.extend_tree_1.extend_tree_1()
    #root.add_child(extend)

    root.add_child(drone_ok)
    root.add_child(drone_not_ok)
    drone_not_ok.add_child(hpl_check)
    drone_not_ok.add_child(para_landing)
    hpl_check.add_child(hpl_ok)
    hpl_check.add_child(hotpoint_landing)

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