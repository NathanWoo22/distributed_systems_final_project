from socket import *
import threading
import time

#Robot Node for distributed SLAM

class RoboNode():
    def __init__(self):
        return
    
    #File is data file to read from ID is the node number and contactList is the list of other nodes listening ports
    #contactList should be in the form [[ip,port],[ip,port],[ip,port]....,[ip,port]]
    #delta is number of keyframes to read and timestep is the delay
    def startUp(self,file, myID, contactList, delta, timestep):
        self.myID = myID
        self.dataFile = file
        self.contacts = contactList
        self.myPort = self.contacts[self.myID][1]
        self.dataStorage = {}
        self.Com = {}
        self.alive = {}
        self.delta = delta
        self.timestep = timestep
        self.syncs = 0
        self.wrapup = []
        self.wrapped = False
        #Create an array for each robots points under its id
        for i in range(len(self.contacts)):
            self.dataStorage[i] = []
            self.Com[i] = True
            self.alive[i] = 10
            self.wrapup.insert(i, False)
        print(self.dataStorage)

        self.sendSocket = socket(AF_INET, SOCK_DGRAM)
        self.sendSocket.bind(('',self.myPort+100))
        self.listenSocket = socket(AF_INET, SOCK_DGRAM)
        self.listenSocket.bind(('', self.myPort))
        self.listenThread = threading.Thread(target=self.Listen, daemon=True)
        self.listenThread.start()
        time.sleep(25)
        for i in range(len(self.contacts)):
            self.sendSimMessage(f"SYNC:{self.myID}", i)
        while self.syncs != len(self.contacts)-1:
            time.sleep(0.0001)

        #LAST CALL
        self.SimulateThread = threading.Thread(target=self.SimulateSLAM,daemon=True)
        self.SimulateThread.start()
        self.livingThread = threading.Thread(target=self.LivingDecay, daemon=True)
        self.livingThread.start()

    #Started in seperate thread
    def Listen(self):
        while True:
            message, clientAddress = self.listenSocket.recvfrom(81920)
            composed = message.decode()
            decomposed = composed.split(sep= ':')
            m_type = decomposed[0]
            sender = int(decomposed[1])
            #simulate dropped connections
            if m_type == "SYNC":
                self.syncs += 1
            if m_type == "WRAP":
                self.wrapup[sender] = True
                test = 0
                for bot in self.wrapup:
                    print(f"Bot is {bot}")
                    if bot:
                        test += 1
                if test == len(self.contacts):
                    self.wrapped = True
            if self.Com[sender]:
                if len(decomposed) != 5:
                    print(composed)
                else:
                    print(f"{decomposed[0]}:{decomposed[1]}:{decomposed[2]}:{decomposed[3]}:data")
                #response recived so sender is alive
                self.ManageLiving(sender,True)
                if m_type == "PUSHREQ":
                    numframe = self.getLastFrame(sender)
                    self.sendMessage(f"PUSHACK:{self.myID}:{numframe}", sender)
                elif m_type == "PUSHACK":
                    #We are going to send all of ours that they don't have
                    keyframe = int(decomposed[2])
                    if keyframe < 0:
                        keyframe = 0
                    for i in range(keyframe,len(self.dataStorage[self.myID])):
                        self.sendMessage(f"UPDATE:{self.myID}:{self.myID}:{i}:{self.dataStorage[self.myID][i]}", sender)
                elif m_type == "UPDATE":
                    bot = int(decomposed[2])
                    keyframe = int(decomposed[3])
                    while keyframe > len(self.dataStorage[bot]):
                        self.dataStorage[bot].append([])
                    if keyframe >= len(self.dataStorage[bot]):
                        self.dataStorage[bot].insert(keyframe,decomposed[4])
                    else:
                        self.dataStorage[bot][keyframe] = decomposed[4]
                elif m_type == "PULLREQ":
                    bot = int(decomposed[2])
                    baseFrame = int(decomposed[3])
                    if baseFrame == -1:
                        baseFrame = 0
                    lastFrame = len(self.dataStorage[bot])
                    if baseFrame != lastFrame:
                        for i in range(baseFrame,lastFrame):
                            print("sending requested update")
                            self.sendMessage(f"UPDATE:{self.myID}:{bot}:{i}:{self.dataStorage[bot][i]}", sender)


     #find first missing keyframe or last frame if none missing return -1 means we have no frames
    def getLastFrame(self,botID):
        numframe = -1
        if len(self.dataStorage[botID]) != 0:
            for i in range(len(self.dataStorage[botID])):
                if self.dataStorage[botID][i] != []:
                    numframe += 1
                else:
                    numframe = i
                    break
        return numframe
    
    def ManageLiving(self, sendID, alive):
        #We only care if they are living if they haven't finished
        if not self.wrapup[sendID]:
            if alive or sendID == self.myID:
                self.alive[sendID] = 10
            else:
                if self.alive[sendID] == 0:
                    lastFrame = self.getLastFrame(sendID)
                    self.Broadcast(f"PULLREQ:{self.myID}:{sendID}:{lastFrame}")
                else:
                    self.alive[sendID] -= 1
        

    def LivingDecay(self):
        while True:
            for i in range(len(self.Com)):
                self.ManageLiving(i, False)
                time.sleep(self.timestep)

    #Started in seperate thread
    #Simulates robot moving through space and creating SLAM map
    def SimulateSLAM(self):
        SLAMDATA = []
        with open(self.dataFile) as f:
            SLAMDATA = [line for line in f]
        keyframe = 0
        while keyframe < len(SLAMDATA):
            for j in range(self.delta):
                self.dataStorage[self.myID].append([SLAMDATA[keyframe]])
                keyframe += 1
            self.Broadcast(f"PUSHREQ:{self.myID}")
            time.sleep(self.timestep)
        print("Finished Simulating")
        time.sleep(5)
        for i in range(len(self.contacts)):
            print("Sending Wrapped")
            self.sendSimMessage(f"WRAP:{self.myID}",i)


    #Makes starting and ending easy
    def sendSimMessage(self, message, clientID):
        clientAddress = self.contacts[clientID]
        composed = message.encode()
        self.sendSocket.sendto(composed, (clientAddress))


    def sendMessage(self, message, clientID):
        if self.Com[clientID]:
            clientAddress = self.contacts[clientID]
            composed = message.encode()
            self.sendSocket.sendto(composed, (clientAddress))

    def Broadcast(self, message):
        #Need to adjust this for individulized drop likely an array but its purpose is to be used to simulate com drops
        for i in range(len(self.contacts)):
            if i != self.myID:
                self.sendMessage(message,i)
    
    def comChange(self, bot, on):
        self.Com[bot] = on
        print(f"Turning off coms from {self.myID} to {bot}")
        
                
        
