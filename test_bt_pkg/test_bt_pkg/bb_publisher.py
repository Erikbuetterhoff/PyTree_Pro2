import rclpy
from rclpy.node import Node
from std_msgs.msg import String  # Beispiel-Nachrichtentyp importieren
from py_trees.blackboard import Client as BlackboardClient
from py_trees.behaviour import Behaviour
from py_trees.common import Status
import py_trees.trees 

class PublishTopicsToBlackboard(Behaviour):
    def __init__(self, name, node, topic_types):
        super(PublishTopicsToBlackboard, self).__init__(name)
        self.blackboard = BlackboardClient(name=name)
        self.node = node
        self.topic_types = topic_types
        self.subscribers = {}

    def setup(self):
        for topic, topic_type in self.topic_types.items():
            self.subscribers[topic] = self.node.create_subscription(
                topic_type, topic, lambda msg, t=topic: self.topic_callback(msg, t), 10
            )

    def topic_callback(self, msg, topic):
        setattr(self.blackboard, topic.replace('/', '_'), msg)

    def update(self):
        return Status.RUNNING

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("publish_topics_to_blackboard")

    # Definiere die Topics und ihre Typen
    topic_types = {
        "/topic1": String,  # Beispiel-Nachrichtentyp
        "/topic2": String,  # Beispiel-Nachrichtentyp
        # Weitere Topics und deren Typen hier hinzufügen
    }

    publish_topics_to_blackboard = PublishTopicsToBlackboard(name="PublishTopicsToBlackboard", node=node, topic_types=topic_types)
    publish_topics_to_blackboard.setup()

    behaviour_tree = py_trees.trees.BehaviourTree(root=publish_topics_to_blackboard)
    behaviour_tree.setup(timeout=15)

    rate = node.create_rate(10)
    while rclpy.ok():
        behaviour_tree.tick_tock(500)
        rclpy.spin_once(node)
        rate.sleep()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
