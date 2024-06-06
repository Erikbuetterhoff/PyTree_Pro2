import tkinter as tk
from py_trees.blackboard import Client as BlackboardClient

class BlackboardGui(tk.Tk):
    def __init__(self, blackboard):
        super().__init__()

        self.title("Blackboard GUI")
        self.blackboard = blackboard

        self.key_label = tk.Label(self, text="Key:")
        self.key_label.grid(row=0, column=0)

        self.value_label = tk.Label(self, text="Value:")
        self.value_label.grid(row=1, column=0)

        self.key_entry = tk.Entry(self)
        self.key_entry.grid(row=0, column=1)

        self.value_entry = tk.Entry(self)
        self.value_entry.grid(row=1, column=1)

        self.publish_button = tk.Button(self, text="Publish", command=self.publish)
        self.publish_button.grid(row=2, column=0, columnspan=2)

        self.blackboard_label = tk.Label(self, text="Blackboard Value: ")
        self.blackboard_label.grid(row=3, column=0, columnspan=2)

    def publish(self):
        key = self.key_entry.get()
        value = self.value_entry.get()
        self.blackboard.set(key, value)
        print(f"Published Key: {key}, Value: {value}")
        self.update_blackboard_label()

    def update_blackboard_label(self):
        value = self.blackboard.get("/Systemcheck_erfolgreich")
        self.blackboard_label.config(text=f"Blackboard Value: {value}")

def main():
    blackboard = BlackboardClient(name="Client")
    blackboard.register_key(key="Systemcheck_erfolgreich", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="EDGE_verbunden", access=py_trees.common.Access.READ)
    blackboard.register_key(key="WPM_geladen", access=py_trees.common.Access.READ)

    # GUI erstellen und starten
    app = BlackboardGui(blackboard)
    app.mainloop()

if __name__ == '__main__':
    main()
