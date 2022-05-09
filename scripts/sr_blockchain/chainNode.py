from __future__ import annotations
import rospy
from  sr_blockchain.gui import GUI

import signal
class ChainNode(GUI):
    def __init__(self):
        super().__init__()
        self.mainWIndow = self.addWindow("mainWindow","chainNodeUI.ui")
        rospy.init_node("chainNode")
        pass

    def startNode(self):
        if not self.mainWIndow.isOpened:
            self.mainWIndow.open()
        
        return self
        pass

    def startLoop(self):
        while not rospy.is_shutdown() and self.isOpened:
            self.update()
        pass

    def getTransactions(self):
        pass


