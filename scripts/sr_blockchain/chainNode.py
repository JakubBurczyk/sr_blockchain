from __future__ import annotations
from typing import List

import cryptography
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization

import rospy
import rosnode

import os
import random
import signal
import json

from  sr_blockchain.gui import GUI
from sr_blockchain.srv import *


class ChainNode(GUI):
    def __init__(self):
        super().__init__()
        self.mainWIndow = self.addWindow("mainWindow","chainNodeUI.ui")
        pass

    def initWidgets(self):
        self.mainWIndow.addButton("pushButton",self.getTransactions)
        self.mainWIndow.addButton("pushButton_register",self.register)
        self.mainWIndow.addButton("pushButton_startROS",self.startROS)
        self.spinBox_id = self.mainWIndow.addSpinBox("spinBox_ROS")
        self.spinBox_id_transactions = self.mainWIndow.addSpinBox("spinBox")
        self.lcd_self_id = self.mainWIndow.addLCD("lcdNumber")
        pass

    def startNode(self):
        self.initWidgets()
        
        if not self.mainWIndow.isOpened:
            self.mainWIndow.open()
        
        return self
        pass

    def startROS(self):
        self.id = str(self.spinBox_id.value)
        self.rospyNode = rospy.init_node("chainNode_" + self.id)
        self.transactionHistoryServer = rospy.Service('getTransactionHistory_' + self.id, getTransactionHistory, self.returnTransactions)
        self.lcd_self_id.display(self.id)
        pass

    def startLoop(self):
        while not rospy.is_shutdown() and self.isOpened:
            self.update()
            #self.running_nodes = rosnode.get_node_names()
        pass

    def returnTransactions(self,foo):
        """
        Got no idea where the additional argument is coming from, discard it who cares!
        """
        
        with open(self.transactionsPath, "r") as file:
            data = file.read()
            return data
            pass

        return "NO DATA"
        pass

    def getTransactions(self):
        ros_nodes: List[str] = rosnode.get_node_names()
        chanNode_ids = []   

        for node in ros_nodes:
            if node.find("chainNode") != -1 and node.find("_") != -1:
                node_id = node[node.find("_")+1:]
                if node_id != self.id:
                    chanNode_ids.append(node_id)

        if chanNode_ids:
            id = random.choice(chanNode_ids)

            rospy.wait_for_service('getTransactionHistory_'+id)
            try:
                getHistory = rospy.ServiceProxy('getTransactionHistory_'+id, getTransactionHistory)
                response = getHistory()
                
                with open(self.transactionsPath,"w") as file:
                    try:
                        file.write(response.transactionHistory)
                        print("Received transaction history")
                        print(response.transactionHistory)
                    except Exception() as e:
                        print(e)

            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)
            pass
        else:
            print("No nodes to ask for transaction history")
            pass
    
    def register(self):
        self.startROS()

        self.userPath = os.path.join(os.path.realpath(__file__),"..\\"*3)
        self.userPath = os.path.join(self.userPath,"users","user_" + self.id)
        self.userPath = os.path.abspath(self.userPath)

        self.keyPath = os.path.join(self.userPath,"keys")
        self.publicKeyPath = os.path.join(self.keyPath,"public.txt")
        self.privateKeyPath = os.path.join(self.keyPath,"private.txt")

        self.transactionsPath = os.path.join(self.userPath,"transactions.txt")

        if not os.path.exists(self.userPath):
            os.mkdir(self.userPath)
            pass

        if not os.path.exists(self.keyPath):
            os.mkdir(self.keyPath)

        if not os.path.exists(self.privateKeyPath):
            self.createKeySet()
        else:
            self.loadKeySet()
        pass

        if not os.path.exists(self.transactionsPath):
            self.getTransactions()
            pass

    def loadKeySet(self):
        with open(self.privateKeyPath, "rb") as key_file:
            self.private_key = serialization.load_pem_private_key(
                                            key_file.read(),
                                            password=None,
                                        )
            self.public_key = self.private_key.public_key
            pass
        print("Loaded key set")
        pass

    def createKeySet(self):
        self.privateKey = private_key = rsa.generate_private_key(
                                                public_exponent=65537,
                                                key_size=512,
                                                backend=default_backend()
                                            )
        pem = private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=serialization.NoEncryption()
                )

        with open(self.privateKeyPath, "wb") as file:
            file.write(pem)
            pass
        
        self.public_key = self.privateKey.public_key()
        pem = self.public_key.public_bytes(
            encoding=serialization.Encoding.PEM,
            format=serialization.PublicFormat.SubjectPublicKeyInfo
            )

        with open(self.publicKeyPath, "wb") as file:
            file.write(pem)
            pass

        print("Generated key set")
        pass

