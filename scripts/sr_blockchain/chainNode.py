from __future__ import annotations
from cgitb import enable
from typing import Dict, List

import cryptography
from cryptography.hazmat.backends import default_backend
from cryptography.hazmat.primitives.asymmetric import rsa
from cryptography.hazmat.primitives import hashes
from cryptography.hazmat.primitives.asymmetric import padding
from cryptography.hazmat.primitives import serialization
from cryptography.hazmat.primitives.asymmetric.rsa import RSAPrivateKey, RSAPublicKey

import rospy
from rospy import ROSException
from rospy import ServiceException
import rosnode

import os
import random
import signal
import json
import datetime

from sr_blockchain.transactionConstants import TransactionConstants
from sr_blockchain.chainNodeConstants import ChainNodeConstants

from sr_blockchain.gui import GUI
from sr_blockchain.srv import getTransactionHistory, initializeTransaction
from sr_blockchain.srv import signTransaction, signTransactionResponse
from sr_blockchain.srv import validateSingleJSON, validateSingleJSONResponse

class Transaction():
    sender: str
    recipient: str
    token: str
    hash: str

    time_request: float
    time_validated: float
    value: float

    ready: bool
    cancelled: bool
    valid: bool

    def __init__(self,sender: str = None, recipient: str = None, value:float = None, token: str = None):
        self.sender = sender
        self.recipient = recipient
        self.token = token
        self.hash = ""
        self.time_request = datetime.datetime.now().timestamp()
        self.time_validated = -1
        self.value = value

        self.ready = False
        self.cancelled = False
        self.valid = False

        self.dict_repr = {}
        pass

    def fromJson(self,line):
        
        data = json.loads(line)
        self.sender = data[TransactionConstants.JSON_FIELD_SENDER.value]
        self.recipient = data[TransactionConstants.JSON_FIELD_RECIPIENT.value]
        self.token = data[TransactionConstants.JSON_FIELD_TOKEN.value]
        self.hash = data[TransactionConstants.JSON_FIELD_HASH.value]
        self.value = data[TransactionConstants.JSON_FIELD_VALUE.value]
        self.time_request = data[TransactionConstants.JSON_FIELD_TIME_REQUEST.value]
        self.time_validated = data[TransactionConstants.JSON_FIELD_TIME_VALIDATED.value]

        return self
        pass

    def asDict(self):
        self.dict_repr[TransactionConstants.JSON_FIELD_SENDER.value] = self.sender
        self.dict_repr[TransactionConstants.JSON_FIELD_RECIPIENT.value] = self.recipient
        self.dict_repr[TransactionConstants.JSON_FIELD_TOKEN.value] = self.token
        self.dict_repr[TransactionConstants.JSON_FIELD_HASH.value] = self.hash
        self.dict_repr[TransactionConstants.JSON_FIELD_VALUE.value] = self.value
        self.dict_repr[TransactionConstants.JSON_FIELD_TIME_REQUEST.value] = self.time_request
        self.dict_repr[TransactionConstants.JSON_FIELD_TIME_VALIDATED.value] = self.time_validated
        return self.dict_repr
        pass


    def toJson(self):
        return json.dumps(self.asDict())
        pass

    

class ChainNode(GUI):
    private_key: RSAPrivateKey
    public_key: RSAPublicKey
    transaction_dict: Dict[str,Transaction] = dict()

    def __init__(self):
        super().__init__()
        self.mainWIndow = self.addWindow("mainWindow","chainNodeUI.ui")
        pass

    def __del__(self):
        self.printConsole("Killing chainNode")
        rospy.signal_shutdown("Destroying chainNode rospy kill")

    def initWidgets(self):
        self.terminal_out = self.mainWIndow.addTextBrowser("textBrowser")

        self.mainWIndow.addButton("pushButton",self.getTransactions)
        self.button_register = self.mainWIndow.addButton("pushButton_register",self.register)
        self.mainWIndow.addButton("pushButton_initTransaction", self.startTransaction)
        self.mainWIndow.addButton("pushButton_calcMyBalance",lambda: self.calculateBalance(f"{self.id}"))

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
        self.lcd_self_id.display(self.id)
        try:
            self.rospyNode = rospy.init_node("chainNode_" + self.id)
            self.transactionHistoryServer = rospy.Service('getTransactionHistory_' + self.id, getTransactionHistory, self.returnTransactions)
            self.transactionInitServer = rospy.Service('initializeTransaction_' + self.id, initializeTransaction, self.processTransaction)
            self.transactionSignServer = rospy.Service('signTransaction_' + self.id, signTransaction, self.processSignature)
            self.transactionValidationJSONServer = rospy.Service('validateSingleJSON_' + self.id, validateSingleJSON, self.validateSingleTransactionJSON)
            
            self.button_register.disable()
            self.spinBox_id.disable()
        except (ServiceException,ROSException) as e:
            self.printConsole(e)
            self.lcd_self_id.display(-1)
            self.button_register(enable)
            self.spinBox_id(enable)
            pass
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
        chainNode_ids = []   

        for node in ros_nodes:
            if node.find("chainNode") != -1 and node.find("_") != -1:
                node_id = node[node.find("_")+1:]
                if node_id != self.id:
                    chainNode_ids.append(node_id)

        if chainNode_ids:
            id = random.choice(chainNode_ids)
            self.printConsole(["Nodes", chainNode_ids])
            rospy.wait_for_service('getTransactionHistory_' + id,timeout = 1)
            try:
                getHistory = rospy.ServiceProxy('getTransactionHistory_'+id, getTransactionHistory)
                response = getHistory()
                
                with open(self.transactionsPath,"w") as file:
                    try:
                        file.write(response.transactionHistory)
                        self.printConsole("Received transaction history")
                        self.printConsole(response.transactionHistory)
                    except Exception() as e:
                        self.printConsole(e)

            except rospy.ServiceException as e:
                self.printConsole("Get transaction history service call failed: %s"%e)
            pass
        else:
            self.printConsole("No nodes to ask for transaction history")
            pass
    
    def register(self):
        self.startROS()

        self.userPath = os.path.join(os.path.realpath(__file__),"..\\"*3)
        self.userPath = os.path.join(self.userPath,ChainNodeConstants.DIR_NAME_USERS.value)
        self.userPath = os.path.abspath(self.userPath)
        self.selfPath = os.path.join(self.userPath,"user_" + self.id)
        self.selfPath = os.path.abspath(self.selfPath)

        self.keyPath = os.path.join(self.selfPath, ChainNodeConstants.DIR_NAME_KEYS.value)
        self.publicKeyPath = os.path.join(self.keyPath, ChainNodeConstants.FILE_NAME_PUBLIC_KEY.value)
        self.privateKeyPath = os.path.join(self.keyPath, ChainNodeConstants.FILE_NAME_PRIVATE_KEY.value)

        self.transactionsPath = os.path.join(self.selfPath, ChainNodeConstants.FILE_NAME_TRANSACTIONS.value)

        if not os.path.exists(self.userPath):
            os.mkdir(self.userPath)
            pass

        if not os.path.exists(self.selfPath):
            os.mkdir(self.selfPath)
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
            self.public_key = self.private_key.public_key()
            pass
        self.printConsole("Loaded key set")
        pass

    def createKeySet(self):
        self.printConsole("Creating key set")
        self.private_key = rsa.generate_private_key(
                            public_exponent=65537,
                            key_size=512,
                            backend=default_backend()
                        )
        self.printConsole("Serializing private key")
        pem = self.serializePrivateKey()

        self.printConsole("Saving key set")
        with open(self.privateKeyPath, "wb") as file:
            file.write(pem)
            pass
        
        self.printConsole("Serializing public key")
        self.public_key = self.private_key.public_key()
        pem = self.serializePublicKey()

        self.printConsole("Saving public key")
        with open(self.publicKeyPath, "wb") as file:
            file.write(pem)
            pass

        self.printConsole("Generated key set")
        pass

    def serializePrivateKey(self) -> str:
        pem = self.private_key.private_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PrivateFormat.TraditionalOpenSSL,
                encryption_algorithm=serialization.NoEncryption()
                )
        return pem
        pass

    def serializePublicKey(self) -> str:
        pem = self.private_key.public_key().public_bytes(
                encoding=serialization.Encoding.PEM,
                format=serialization.PublicFormat.SubjectPublicKeyInfo
                )
        return pem
        pass

    def processTransaction(self,initializeTransaction):
        self.printConsole([initializeTransaction.sender, initializeTransaction.recipient, initializeTransaction.value])
        token = "token_" + self.id + str(random.randint(0,10000000))
        self.printConsole(token)

        transaction = Transaction(
                        sender = initializeTransaction.sender,
                        recipient = initializeTransaction.recipient,
                        value = initializeTransaction.value,
                        token = token
                        )

        self.transaction_dict[token] = transaction

        return token
        pass

    def processSignature(self,sgnTransaction): 
        token:str = sgnTransaction.token
        user_id:str = sgnTransaction.user
        response = signTransactionResponse()
        response.user = self.id
        response.response = "signed"
        try:
            if self.transaction_dict[token].sender != user_id:
                self.printConsole("Sender user mismatch")
                #raise Exception("Transaction signed user and requested sender mismatch")
            else:
                signedToken = bytes(bytearray(sgnTransaction.signedToken))
                public_key = bytes(bytearray(sgnTransaction.public_key))

                senderNode_public_key = serialization.load_pem_public_key(public_key)
                self.printConsole("Verifying signature")
                senderNode_public_key.verify(
                                    signedToken,
                                    bytes(token,"UTF-8"),
                                    padding.PSS(
                                        mgf=padding.MGF1(hashes.SHA256()),
                                        salt_length=padding.PSS.MAX_LENGTH
                                        ),
                                    hashes.SHA256()
                                    )
                self.printConsole("Validating transaction")
                self.validate(self.transaction_dict[token])

        except Exception() as e:
            self.printConsole(e)
            response.response = "fake"
            pass

        return response
        pass

    def getActiveNodes(self, chainNode_ids: List[str]):
        try:
            self.printConsole("Getting active nodes")
            ros_nodes: List[str] = rosnode.get_node_names()
            
            for node in ros_nodes:
                if node.find("chainNode") != -1 and node.find("_") != -1:
                    node_id = node[node.find("_")+1:]
                    if node_id != self.id:
                        self.printConsole(f"Adding node: {node_id}")
                        chainNode_ids.append(node_id)
                        
        except (rospy.ServiceException, rospy.ROSException) as e:
            self.printConsole(f"Failed to get active nodes: {e}")
            pass
        
        self.printConsole(f"Returning active nodes {chainNode_ids}")
        return chainNode_ids
        pass

    def validate(self, transaction: Transaction):
        if transaction.value <= 0:
            self.printConsole("Transaction value cannot be below 0")
            return
        else:
            nodes = []
            self.getActiveNodes(nodes).append(self.id)
            nodes = [n for n in nodes if n!=transaction.sender]

            votes = {"valid":0, "invalid":0}
            self.printConsole(f"Active nodes {nodes}")
            self.printConsole("Starting voting procedure")
            try:
                for node in nodes:
                    
                        self.printConsole("Waiting for validation service ")
                        rospy.wait_for_service('validateSingleJSON_' + node,1)
                        self.printConsole("Validate proxy")
                        validateFromNode = rospy.ServiceProxy('validateSingleJSON_' + node, validateSingleJSON)
                        self.printConsole(f"Asking node: {node}")
                        response = validateFromNode(transaction.toJson())
                        self.printConsole(response)
            except Exception as e:
                self.printConsole(f"Generic validate exception {e}")
                pass
            #except (rospy.ServiceException, rospy.ROSException) as e:
                
            #    self.printConsole(f"Validate transaction service call failed: {e}")
            #pass
        pass

    def validateSingleTransactionJSON(self, transactionJSON):
        transaction = Transaction().fromJson(transactionJSON.transaction_json)
        self.printConsole(f"Transaction w/ token: {transaction.token} is to be validated")
        self.printConsole(transactionJSON)

        self.printConsole(f"Transaction sender {transaction.sender}")
        balance = self.calculateBalance(transaction.sender)
        
        status:TransactionConstants = TransactionConstants.RETURN_VALIDATION_INVALID
        if balance > transaction.value:
            status = TransactionConstants.RETURN_VALIDATION_VALID

        response = validateSingleJSONResponse()
        response.node_id = self.id
        response.result = status.value
        return response
        pass

    def calculateBalance(self, user):
        with open(self.transactionsPath,"r") as file:
            balance = 0
            for line in file:
                transaction = Transaction().fromJson(line)
                self.printConsole(f"Sender {transaction.sender} Recipient {transaction.recipient}")
                if transaction.sender == user:
                    balance -= transaction.value
                elif transaction.recipient == user:
                    balance += transaction.value

        self.printConsole(f"Balance of user {user} = {balance}")
        return balance
        pass

    def startTransaction(self):
        nodes = []
        self.getActiveNodes(nodes)

        if not nodes:
            self.printConsole(f"No nodes to process transaction")
        else:
            id = random.choice(nodes)
            rospy.wait_for_service('initializeTransaction_' + id,1)
            rospy.wait_for_service('signTransaction_' + id,1)
            try:
                initTransaction = rospy.ServiceProxy('initializeTransaction_' + id, initializeTransaction)
                response = initTransaction(self.id,id,1.23)
                token: str = response.token
                #token = token[token.find("_")+1:]
                self.printConsole("Signing token")
                signedToken = self.private_key.sign(
                                token.encode(),
                                padding.PSS(
                                    mgf=padding.MGF1(hashes.SHA256()),
                                    salt_length=padding.PSS.MAX_LENGTH
                                ),
                                hashes.SHA256()
                            )
                self.printConsole("Signed token")
                sgnTransaction = rospy.ServiceProxy('signTransaction_' + id, signTransaction)
                serialized_public_key = self.serializePublicKey()

                response = sgnTransaction(token,self.id,signedToken, serialized_public_key)
                self.printConsole(response)

            except rospy.ServiceException as e:
                self.printConsole(f"Init transaction service call failed: {e}")
            pass

    def printConsole(self,text):
        if isinstance(text, str):
            self.printConsoleLine(text)
        else:
            self.printConsoleLine(repr(text))
        pass

    def printConsoleLine(self,line:str):
        line = f"[{datetime.datetime.now()}] {line}"
        print(line)
        self.terminal_out.appendLine(line)
