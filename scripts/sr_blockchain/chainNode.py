#from __future__ import annotations
import rospy
from  sr_blockchain.gui import GUI

#a = GUI()

class foo():
    def __init__(self) -> None:
        print("foo init")
    pass

class ChainNode(GUI):
    def __init__(self):
        super().__init__()

        self.mainWIndow = self.addWindow("mainWindow","chainNodeUI.ui")
        pass

    def startNode(self):
        if not self.mainWIndow.isOpened:
            self.mainWIndow.open()
        pass
        return self

    def startLoop(self):
        rospy.init_node("chainNode")
        while not rospy.is_shutdown() and self.isOpened:
            self.update()


