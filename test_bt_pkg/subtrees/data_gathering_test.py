import py_trees
import py_trees_ros
from std_msgs.msg import Bool

def create_data_gatherer():

    sys_check= py_trees_ros.subscribers.ToBlackboard(topic_name="systemcheck_test",
                                          topic_type=Bool,
                                          qos_profile=2,
                                          blackboard_variables={"Systemcheck_erfolgreich": "data"}
                                          )
    
    edge_check= py_trees_ros.subscribers.ToBlackboard(topic_name="edge_test",
                                          topic_type=Bool,
                                          qos_profile=2,
                                          blackboard_variables={"EDGE_verbunden": "data"}
                                          )
    
    wpm_check = py_trees_ros.subscribers.ToBlackboard(topic_name="wpm_test",
                                          topic_type=Bool,
                                          qos_profile=2,
                                          blackboard_variables={"WPM_geladen": "data"}
                                          )
    
    root = py_trees.composites.Parallel(name="Data Gatherer")

    root.add_children([sys_check, edge_check, wpm_check])

    return root