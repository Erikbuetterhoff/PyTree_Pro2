import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
from std_msgs.msg import Bool
import action_pkg.action as actions

def extend_tree_3() -> py_trees.behaviour.Behaviour:


    root = py_trees.composites.Sequence("Check battery",memory=True)
    landing = py_trees.composites.Sequence("Choose landing",memory=True)

    battery_status = py_trees.behaviours.Success(name="Battery okay?") ### Wie durchgehend prüfen oder Threshold ausreichend ###

##Return to Home Landing

    RTH_landing = py_trees.composites.Sequence(name="RTH landing",memory=False)
    RTH_possible = py_trees_ros.subscribers.CheckData(  name="Drohne okay?", 
                                                        topic_name="bt_test_topic",
                                                        topic_type=Bool, variable_name="rth_available", ## Variable muss Status wissen?! ##
                                                        expected_value= True, 
                                                        fail_if_bad_comparison=True, 
                                                        qos_profile=2, 
                                                        clearing_policy=2
                                                        )

    RTH_do = py_trees_ros.action_clients.FromConstant(  name="Return to Home",
                                                        action_type=actions.Empty,
                                                        action_name="XXXXXXX",
                                                        action_goal=actions.Empty.Goal(),
                                                        generate_feedback_message=lambda msg: actions.Empty.Feedback()
                                                        )

    

##Hot-Point Landing

    HPL_landing = py_trees.composites.Sequence(name="HPL landing",memory=False)
    HPL_possible = py_trees_ros.subscribers.CheckData(  name="Drohne okay?", 
                                                        topic_name="bt_test_topic",
                                                        topic_type=Bool, variable_name="hpl_available", 
                                                        expected_value= True, 
                                                        fail_if_bad_comparison=True, 
                                                        qos_profile=2, 
                                                        clearing_policy=2
                                                        )

    HPL_do = py_trees_ros.action_clients.FromConstant(  name="Hot-Point Landing",
                                                        action_type=actions.Empty,
                                                        action_name="XXXXX",
                                                        action_goal=actions.Empty.Goal(),
                                                        generate_feedback_message=lambda msg: actions.Empty.Feedback()
    )

##Parachute Landing

    para_landing = py_trees_ros.action_clients.FromConstant(    name="Emergency Landing with parachute",
                                                                action_type=actions.Empty,
                                                                action_name="XXXXX",
                                                                action_goal=actions.Empty.Goal(),
                                                                generate_feedback_message=lambda msg: actions.Empty.Feedback()
    )


    root.add_children([battery_status,landing])
    landing.add_children([RTH_landing,HPL_landing,para_landing])

    RTH_landing.add_children([RTH_possible,RTH_do])
    HPL_landing.add_children([HPL_possible,HPL_do])

    return root



def main():
    rclpy.init()

    root = extend_tree_3()
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