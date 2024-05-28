import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from py_trees.behaviours import Success, Failure
from pynput import keyboard
import py_trees_msgs.msg

class SimpleTree(Node):
    def __init__(self):
        super().__init__('simple_tree')
        self.root = py_trees.composites.Selector(name="root")
        self.success = Success(name="Success")
        self.failure = Failure(name="Failure")
        self.root.add_child(self.success)
        self.tree = py_trees_ros.trees.BehaviourTree(self.root)

        # Publisher für die Baumstruktur
        self.tree_publisher = self.create_publisher(py_trees_msgs.msg.BehaviourTree, 'tree', 10)

        self.tree.setup(timeout=15)
        self.tick_timer = self.create_timer(1.0, self.tick)
        self.get_logger().info("Baum initialisiert und tickt alle 1 Sekunde")
        self.listener = keyboard.Listener(on_press=self.on_key_press)
        self.listener.start()

    def on_key_press(self, key):
        try:
            if key.char == 't':
                if self.root.children[0].name == "Success":
                    self.root.remove_child(self.success)
                    self.root.add_child(self.failure)
                else:
                    self.root.remove_child(self.failure)
                    self.root.add_child(self.success)
        except AttributeError:
            pass

    def tick(self):
        self.tree.tick()

        # Baumstruktur in ASCII-Format umwandeln
        ascii_tree = py_trees.display.ascii_tree(self.tree.root)

        # Veröffentliche die Baumstruktur auf dem Topic
        msg = py_trees_msgs.msg.BehaviourTree()
        msg.root = ascii_tree
        self.tree_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTree()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
