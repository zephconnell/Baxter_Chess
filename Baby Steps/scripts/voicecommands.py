#!/usr/bin/env python


import baxter_interface
from baxter_interface import CHECK_VERSION
import rospy
from std_msgs.msg import (
      String,
      Float64,
)

import rospkg

#import and set up virtual chess logic classes
from Game import Game
game = Game()
game.initial_board.initialize_board()
game.create_piece_dict()
game.print_board()

#import class for clean shutdown
from neutral import Reset

class Voice:
    def __init__(self):
        
        rs = baxter_interface.RobotEnable(CHECK_VERSION)
        print("Enabling robot... ")
        rs.enable()

        self.rate = 5
        self.r = rospy.Rate(self.rate)

        self.message = 'nothing yet'
        
        # A flag to determine whether or not voice control is paused
        self.paused = False

	# A flag to determine if the given command is the pickup index or placement index
        self.movementdefined = False

	# variables for the starting and ending positions
        self.piecetomove = 'none'
        self.destination = 'nowhere'
        
        # variable to determine whose turn it is
        self.whiteturn = True

        #publishing the verbal command to the command topic
        self.pub = rospy.Publisher('/command', String, queue_size=5)
        
        # Subscribe to the /recognizer/output topic to receive voice commands.
        rospy.Subscriber('/recognizer/output', String, self.speech_callback)      
        
         
    def speech_callback(self, msg):
        
        # Get the motion command from the recognized phrase
        command = msg.data
        
        # Log the command to the screen
        rospy.loginfo("Command: " + str(command))
        
        # If the user has asked to pause/continue voice control,
        # set the flag accordingly 
        if command == 'pause':
            self.paused = True
            print("Voice command features disabled.")
            return
        elif command == 'continue':
            self.paused = False
            print("Voice command features enabled.")
            return
        
        # If voice control is paused, simply return without
        # performing any action
        if self.paused:
            print("Currently paused")
            return       
        
        if command == 'shutdown':
            print("Initializing node...")
            #rospy.init_node("reset", anonymous = True)
            reset = Reset()
            reset.set_neutral()
            reset.reset_facescreen()
            reset.grippers_reset()
            reset.disable_motors()
            print("\nBaxter's arms now in neutral position, led display back to rethink log and motors have been disabled.")
            return

        piece = 'none'
        index = 'nowhere'
        
        # get index of piece to move or index to move to                     
        if command == 'alpha one':  
            index = 'a1'
        elif command == 'alpha two':  
            index = 'a2'
        elif command == 'alpha three':
            index = 'a3'
        elif command == 'alpha four':  
            index = 'a4'
        elif command == 'alpha five':
            index = 'a5'
        elif command == 'alpha six':
            index = 'a6'
        elif command == 'alpha seven':  
            index = 'a7'
        elif command == 'alpha eight':
            index = 'a8'
        elif command == 'bravo one':  
            index = 'b1'
        elif command == 'bravo two':  
            index = 'b2'
        elif command == 'bravo three':
            index = 'b3'
        elif command == 'bravo four':  
            index = 'b4'
        elif command == 'bravo five':
            index = 'b5'
        elif command == 'bravo six':
            index = 'b6'
        elif command == 'bravo seven':  
            index = 'b7'
        elif command == 'bravo eight':
            index = 'b8'
        elif command == 'charlie one':  
            index = 'c1'
        elif command == 'charlie two':  
            index = 'c2'
        elif command == 'charlie three':
            index = 'c3'
        elif command == 'charlie four':  
            index = 'c4'
        elif command == 'charlie five':
            index = 'c5'
        elif command == 'charlie six':
            index = 'c6'
        elif command == 'charlie seven':  
            index = 'c7'
        elif command == 'charlie eight':
            index = 'c8'
        elif command == 'delta one':  
            index = 'd1'
        elif command == 'delta two':  
            index = 'd2'
        elif command == 'delta three':
            index = 'd3'
        elif command == 'delta four':  
            index = 'd4'
        elif command == 'delta five':
            index = 'd5'
        elif command == 'delta six':
            index = 'd6'
        elif command == 'delta seven':  
            index = 'd7'
        elif command == 'delta eight':
            index = 'd8'
        elif command == 'echo one':  
            index = 'e1'
        elif command == 'echo two':  
            index = 'e2'
        elif command == 'echo three':
            index = 'e3'
        elif command == 'echo four':  
            index = 'e4'
        elif command == 'echo five':
            index = 'e5'
        elif command == 'echo six':
            index = 'e6'
        elif command == 'echo seven':  
            index = 'e7'
        elif command == 'echo eight':
            index = 'e8'
        elif command == 'foxtrot one':  
            index = 'f1'
        elif command == 'foxtrot two':  
            index = 'f2'
        elif command == 'foxtrot three':
            index = 'f3'
        elif command == 'foxtrot four':  
            index = 'f4'
        elif command == 'foxtrot five':
            index = 'f5'
        elif command == 'foxtrot six':
            index = 'f6'
        elif command == 'foxtrot seven':  
            index = 'f7'
        elif command == 'foxtrot eight':
            index = 'f8'
        elif command == 'golf one':  
            index = 'g1'
        elif command == 'golf two':  
            index = 'g2'
        elif command == 'golf three':
            index = 'g3'
        elif command == 'golf four':  
            index = 'g4'
        elif command == 'golf five':
            index = 'g5'
        elif command == 'golf six':
            index = 'g6'
        elif command == 'golf seven':  
            index = 'g7'
        elif command == 'golf eight':
            index = 'g8'
        elif command == 'hotel one':  
            index = 'h1'
        elif command == 'hotel two':  
            index = 'h2'
        elif command == 'hotel three':
            index = 'h3'
        elif command == 'hotel four':  
            index = 'h4'
        elif command == 'hotel five':
            index = 'h5'
        elif command == 'hotel six':
            index = 'h6'
        elif command == 'hotel seven':  
            index = 'h7'
        elif command == 'hotel eight':
            index = 'h8'
        elif command == 'pawn one':
            piece = 'P1'
        elif command == 'pawn two':
            piece = 'P2'
        elif command == 'pawn three':
            piece = 'P3'
        elif command == 'pawn four':
            piece = 'P4'
        elif command == 'pawn five':
            piece = 'P5'
        elif command == 'pawn six':
            piece = 'P6'
        elif command == 'pawn seven':
            piece = 'P7'
        elif command == 'pawn eight':
            piece = 'P8'
        elif command == 'rook one':
            piece = 'R1'
        elif command == 'rook two':
            piece = 'R2'
        elif command == 'knight one':
            piece = 'N1'
        elif command == 'knight two':
            piece = 'N2'
        elif command == 'bishop one':
            piece = 'B1'
        elif command == 'bishop two':
            piece = 'B2'
        elif command == 'king':
            piece = 'K'
        elif command == 'queen':
            piece = 'Q'
        elif command == 'reset':
            self.piecetomove = 'none'
            self.destination = 'nowhere'
            self.movementdefined = False
            print("Reset confirmed. Name the piece you would like to move or a functional operation.")
        else:
            print("No valid command")
            return
	
        if self.piecetomove == 'none' and piece != 'none':
            self.piecetomove = piece
            print(self.piecetomove + " has been chosen as the piece to move. Give the location to which you would like to move it.")
        if self.destination == 'nowhere' and index != 'nowhere' and self.piecetomove != 'none':
            self.destination = index
            self.movementdefined = True
            print(self.destination + " has been chosen as the destination.")
        if self.movementdefined:
            self.message = self.piecetomove + ' ' + self.destination
		

if __name__=="__main__":

    try:
        rospy.init_node('voicecommands', anonymous = True)
        voice = Voice()
        print("Going to start the while loop\n")
        print("Name the piece you would like to move or a functional operation.")
        
        while not rospy.is_shutdown():
            if voice.movementdefined:
                voice.pub.publish(voice.message)
                #game.move_piece("P1","a3",False)
                validmove = game.move_piece(voice.piecetomove,voice.destination,voice.whiteturn)
                print("Move " + voice.piecetomove + " to postion " + voice.destination)
                voice.piecetomove = 'none'
                voice.destination = 'nowhere'
                voice.movementdefined = False
                if validmove:
                    voice.whiteturn = not voice.whiteturn
                game.print_board()
                print("Give a piece and a location to move it.")
            voice.r.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice command script terminated.")

