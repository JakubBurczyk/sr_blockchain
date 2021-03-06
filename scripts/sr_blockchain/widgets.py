from __future__ import annotations
import datetime

from PyQt5.QtWidgets import *
from PyQt5.QtGui import QPixmap
from PyQt5 import *
from PyQt5 import QtWidgets, uic, QtGui
import sys
from termcolor import colored
import numpy as np
from typing import Callable, List

import os
from abc import ABC
import sr_blockchain.window as window

class Widget(ABC):

    def __init__(self, win:window.Window, name: str):
        self._window = win
        self._name = name
        self._widget = getattr(self._window, self._name)
        pass

    def disable(self) -> None:
        self._widget.setEnabled(False)
        pass

    def enable(self) -> None:
        self._widget.setEnabled(True)
        pass

    def setEnabled(self, state: bool):
        self._widget.setEnabled(state)
        pass

    def toggleEnable(self) -> bool:
        """
        Switch button on/off
        :return: bool, Returns resultant state of the button
        """
        state = self.enabled
        self.setEnabled(not state)
        return self.enabled
        pass

    def update(self):
        pass

    @property
    def name(self):
        return self._name

    @property
    def widget(self):
        return self._widget
        pass

    @property
    def enabled(self):
        return self._widget.isEnabled()
        pass


class Button(Widget):
    _widget: QPushButton

    def __init__(self, win:window.Window, name: str, function: Callable):
        super(Button, self).__init__(win, name)
        self._widget.clicked.connect(function)
        pass


class SpinBoxAbstract(Widget):
    def __init__(self, win:window.Window, name: str):
        super(SpinBoxAbstract, self).__init__(win, name)
        pass

    @property
    def value(self):
        return self._widget.value()
        pass


class SpinBox(SpinBoxAbstract):
    _widget: QSpinBox
    value: int

    def __init__(self, win:window.Window, name: str):
        super(SpinBox, self).__init__(win, name)
        pass


class DoubleSpinBox(SpinBoxAbstract):
    _widget: QDoubleSpinBox
    value: float

    def __init__(self, win:window.Window, name: str):
        super(DoubleSpinBox, self).__init__(win, name)
        pass


class Pixmap(Widget):
    _widget: QLabel

    def __init__(self, win:window.Window, name: str, imgPath):
        super(Pixmap, self).__init__(win, name)
        self.imgPath = imgPath
        self.pixmap = QPixmap(self.imgPath)
        self._widget.setPixmap(self.pixmap)
        self._widget.setScaledContents(True)

        self.frequency = 1  # Hz
        self.updateDt = 1 / self.frequency
        self.lastUpdate = datetime.datetime.now()
        pass

    def update(self):
        dt_ms = (datetime.datetime.now() - self.lastUpdate).total_seconds()
        if dt_ms >= self.updateDt:
            self.pixmap = QPixmap(self.imgPath)
            self._widget.setPixmap(self.pixmap)
            self._widget.setScaledContents(True)
        else:
            pass


class LCD(Widget):
    _widget: QLCDNumber
    getValue: Callable

    def __init__(self, win:window.Window, name: str):
        super(LCD, self).__init__(win, name)

        self._value = 0
        self.frequency = 1 #Hz
        self.updateDt = 1/self.frequency
        self.lastUpdate = datetime.datetime.now()
        self.getValue = None
        self.updateable = False
        pass

    def setCallback(self, function: Callable):
        self.getValue = function
        pass

    def setValue(self, value):
        #print(f"LCD: {self.name} displaying: {self.value}")
        self._value = value
        self.updateable = True
        pass

    def display(self, value):
        #print(f"LCD: {self.name} displaying: {self.value}")
        if isinstance(value, np.ndarray):
            vlist = value.tolist()
            if len(vlist) > 0:
                value = vlist[0]
            else:
                value = 0

        self._widget.display(value)
        pass

    def update(self):
        dt_ms = (datetime.datetime.now() - self.lastUpdate).total_seconds()
        if dt_ms >= self.updateDt:
            if self.getValue is not None:
                self._value = self.getValue()
                self.lastUpdate = datetime.datetime.now()

            self.display(self.value)
        else:
            pass

    #@property
    #def updateable(self):
    #    print(self.name, self.getValue is not None)
    #    return self.getValue is not None
    #    pass

    @property
    def value(self):
        return self._value
        pass

class TextBrowser(Widget):
    _widget:QTextBrowser

    def __init__(self, win: window.Window, name: str):
        super().__init__(win, name)
        pass

    def setText(self, text:str):
        self._widget.setText(text)
        pass

    def appendLine(self, line):
        if isinstance(line, str):
            self._widget.append(line)
        elif isinstance(line, List):
            for l in line:
                self._widget.append(l)
        try:
            #self._widget.verticalScrollBar().setValue(self._widget.verticalScrollBar().maximum())
            pass
        except:
            pass
        pass

    def clear(self):
        self._widget.clear()
        pass



