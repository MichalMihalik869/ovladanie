# -*- coding: utf-8 -*-
"""
Created on Thu Jun 10 08:38:04 2021

@author: NEMEC
"""

import threading
import socket
from time import sleep

class RequestHandler:
    """
    Server-side request handler
    """
    
    def __init__(self, ep, onRequestReceived, onConnectionClose = None):
        self.ep = ep
        self.onRequestReceived = onRequestReceived
        self.onConnectionClose = onConnectionClose
        self.th = threading.Thread(target=self.run)
        self.th.start()
        
    def run(self):
        while True:
            try:
                rx = self.ep.recv(4096)
                if len(rx) != 0:
                    tx = self.onRequestReceived(rx.decode())
                
                    if tx != None:
                        self.ep.sendall(tx.encode())
                    
                sleep(0.001)
                
            except:
                if self.onConnectionClose != None:
                    self.onConnectionClose(self)
                return

class SimpleServer:
    """
    Implements TCP/IP socket server
    """
    
    def __init__(self, host, port, onRequestReceived, clientsLimit = 1):
        self.host = host
        self.port = port
        self.ep = None
        self.th = None
        self.enabled = False
        self.clients = set()
        self.clientsLimit = clientsLimit
        self.onRequestReceived = onRequestReceived
        self.connect()
        
    def __del__(self):
        self.disconnect()
         
    def connect(self):
        """ Connects to the robotic arm """
        self.disconnect()
        sleep(0.5)    
        self.enabled = True
        self.th = threading.Thread(target=self.run)
        self.th.start()
        
    def disconnect(self):
        """ Closes and deletes the connection """
        self.enabled = False
        if self.th != None:
            self.th.join()
            del self.th
        self.th = None
        self.clients = set()
        
    def onDisconnect(self, cl):
        print('Client ' + hex(id(cl)) + ' disconnected')
        self.clients.remove(cl)
        
    def run(self):
        self.ep = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ep.bind((self.host, self.port))
        self.ep.listen()

        while self.enabled:
            if len(self.clients) < self.clientsLimit:
                cl, addr = self.ep.accept()
                h = RequestHandler(cl, self.onRequestReceived, self.onDisconnect)
                print('Client ' + hex(id(h)) + ' connected')
                self.clients.add(h)
            sleep(0.001)
            
        if self.ep != None:
            self.ep.close()
            del self.ep
        self.ep = None
            

class SimpleClient:
    """
    Implements TCP/IP socket client
    """
    
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.ep = None
        self.connect()
        
    def __del__(self):
        self.disconnect()
         
    def connect(self):
        """ Connects to the robotic arm """
        self.disconnect()
        sleep(0.5)    
        self.ep = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.ep.connect((self.host, self.port))
        self.ep.setblocking(True)
        self.ep.settimeout(1.0)

    def disconnect(self):
        """ Closes and deletes the connection """
        if self.ep != None:
            self.ep.close()
            del self.ep
        self.ep = None
        
    def write(self, data):
        try:
            self.ep.sendall(data.encode())
            return self.ep.recv(4096)
        except ConnectionError:
            self.connect()
        
if __name__ == '__main__':
    def onClientRequest(request) :
        if request != "":
            print("Received: " + request)
            if request == 'Ahoj':
                return "Thank you"
            else:
                return "Go to hell"
        
    srv = SimpleServer('localhost',5555, onClientRequest)
    
    