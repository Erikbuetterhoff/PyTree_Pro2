import py_trees
import py_trees_ros.trees
import py_trees.console 
from std_msgs.msg import Bool
import action_pkg.action as actions

def subtree_start() -> py_trees.behaviour.Behaviour:

    start_sequence = py_trees.composites.Sequence("Startsequenz einleiten",memory=False)

    start_startup_selector = py_trees.composites.Selector("Startsequenz prüfen",memory=False)

    start_startup_condition = py_trees_ros.subscribers.CheckData(
        name="Startsequenz abgeschlossen?", 
        topic_name="start_sequence_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    start_sub_sequence = py_trees.composites.Sequence("Systemcheck, EDGE und WPM ready?",memory=False)

    start_check_selector = py_trees.composites.Selector("Systemcheck",memory=False)

    start_systemcheck_condition = py_trees_ros.subscribers.CheckData(
        name="Systemcheck erfolgreich?", 
        topic_name="start_check_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    start_systemcheck_action = py_trees_ros.action_clients.FromConstant(  
        name="Systemcheck durchführen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    start_edge_selector = py_trees.composites.Selector("EDGE Verbindung",memory=False)

    start_edge_condition = py_trees_ros.subscribers.CheckData(
        name="Mit EDGE verbunden?", 
        topic_name="start_edge_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    start_edge_action = py_trees_ros.action_clients.FromConstant(  
        name="Mit EDGE verbinden",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    start_wpmload_selector = py_trees.composites.Selector("WPM",memory=False)

    start_wpmload_condition = py_trees_ros.subscribers.CheckData(
        name="WPM geladen?", 
        topic_name="start_wpm_ok_topic", 
        topic_type=Bool, 
        variable_name="data", 
        expected_value=True, 
        fail_if_bad_comparison=True, 
        qos_profile=2, 
        clearing_policy=2
    )

    start_wpmload_action = py_trees_ros.action_clients.FromConstant(  
        name="WPM laden",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    start_wpm_action = py_trees_ros.action_clients.FromConstant(  
        name="WPM ausführen",
        action_type=actions.Wait,
        action_name="wait_action",
        action_goal=actions.Wait.Goal(timer=5),
        generate_feedback_message=lambda msg: msg.feedback.part_result
    )

    start_sequence.add_children([start_startup_selector,start_wpm_action])
    start_startup_selector.add_children([start_startup_condition,start_sub_sequence])
    start_sub_sequence.add_children([start_check_selector,start_edge_selector,start_wpmload_selector])
    start_check_selector.add_children([start_systemcheck_condition,start_systemcheck_action])
    start_edge_selector.add_children([start_edge_condition,start_edge_action])
    start_wpmload_selector.add_children([start_wpmload_condition,start_wpmload_action])

    return start_sequence