import rclpy
import py_trees
import py_trees_ros.trees
import sys
import py_trees.console as console
from std_msgs.msg import Bool

import action_pkg.action as actions


def start_sequence() -> py_trees.behaviour.Behaviour:
    
    blackboard = py_trees.blackboard.Client(name="Client")
    
    root = py_trees.composites.Sequenz(name="Start Sequenz")
    
    start_startup_selector = py_trees.behaviours.Selector("Start überprüfen")
    
    start_wpm_action= py_trees_ros.action_clients.FromConstant( #Action die die WPM ausführt
        name="WPM ausführen",
        action_type= actions.WPM,
        action_name= "wpm_action",
        action_goal=actions.Wait.Goal(Timer=5),
        generate_feedback_message=lambda msg: actions.WPM.Feedback()
    )

    check_start_on_bb = py_trees.decorators.EternalGuard(
        name="Startsequenz abgeschlossen?",
        condition= blackboard.Systemcheck_erfolgreich == True, 
        blackboard_keys={"Systemcheck_erfolgreich", "EDGE_verbunden", "WPM_geladen"}, 
        child = startsequenz
    )

    startsequenz= py_trees.behaviours.Sequenz("Startsequenz")
    
    check_systemcheck_on_bb = py_trees.decorators.EternalGuard(
        name="Systemcheck erfolgreich?",
        condition= blackboard.Systemcheck_erfolgreich == True, 
        blackboard_keys={"Systemcheck_erfolgreich"},
        child = systemcheck
    )

    systemcheck= py_trees_ros.action_clients.FromConstant( 
        name="Systemcheck durchführen",
        action_type= actions.Wait,
        action_name= "wait_action",
        action_goal=actions.Wait.Goal(Timer=5),
        generate_feedback_message=lambda msg: actions.Wait.Feedback()
    )

    check_EDGE_on_bb = py_trees.decorators.EternalGuard(
        name="Mit EDGE verbunden?",
        condition= True, 
        blackboard_keys={"EDGE_verbunden"},
        child = start_edge_action
    )

    start_edge_action = py_trees_ros.action_clients.FromConstant( 
        name="Mit EDGE verbinden",
        action_type= actions.Wait,
        action_name= "wait_action",
        action_goal=actions.Wait.Goal(Timer=5),
        generate_feedback_message=lambda msg: actions.Wait.Feedback()
    )

    check_WPM_on_bb = py_trees.decorators.EternalGuard(
        name="WPM geladen ?",
        condition= True, 
        blackboard_keys={"WPM_geladen"},
        child = start_wpmload_action
    )

    start_wpmload_action = py_trees_ros.action_clients.FromConstant( 
        name="WPM laden",
        action_type= actions.Wait,
        action_name= "wait_action",
        action_goal=actions.Wait.Goal(Timer=5),
        generate_feedback_message=lambda msg: actions.Wait.Feedback()
    )

    root.add_children([check_start_on_bb, start_wpm_action])
    startsequenz.add_children([check_systemcheck_on_bb, check_EDGE_on_bb, check_WPM_on_bb]) #hier fehlen zuweisungen aber bereits in Ethernalguard entahlten
   

    return root

def main():
    rclpy.init()

    root = start_sequence()
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