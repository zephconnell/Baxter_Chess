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

#import and initialization for board and piece recognition and movement
from chess_real2 import Locate
import string
import copy
import os
image_directory = os.getenv("HOME") + "/Golf/"

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

        # variables to determine if Baxter successfully executed a move if valid
	self.checkedmove = False
        self.waitingforcheck = False
        self.movesuccess = False

        # variable to determine if a chess piece movement has been successfully requested
        self.movementdefined = False
        
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
            reset = Reset()
            reset.set_neutral()
            reset.reset_facescreen()
            reset.grippers_reset()
            reset.disable_motors()
            print("\nBaxter's arms now in neutral position, led display back to rethink log and motors have been disabled.")
            return

        # wait for an answer to whether Baxter successfully made a move or not before accepting any new movements
        if self.movementdefined and not self.checkedmove:
            if command == 'yes':
                self.checkedmove = True
                self.movesuccess = True
            elif command == 'no':
                self.checkedmove = True
                self.movesuccess = False
            else:
                print("Must answer before requesting another move.\nDid Baxter successfully make the move most recently requested?\nReply 'Yes' or 'No'.")
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
            print("Reset confirmed. Say the piece you would like to move, or say a functional operation.")
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

		
# read the setup parameters from setup.dat
#This is the first function to look at.It is entirely
# optional.  The limb and distance can be "hard coded" or 
#you can even get user input if desired. 
def get_setup():
    global image_directory
    file_name = image_directory + "setup.dat"

    try:
        f = open(file_name, "r")
    except ValueError:
        sys.exit("ERROR: setup must be run before this")

    # find limb
    s = string.split(f.readline())
    if len(s) >= 3:
        if s[2] == "left" or s[2] == "right":
            limb = s[2]
        else:
            sys.exit("ERROR: invalid limb in %s" % file_name)
    else:
        sys.exit("ERROR: missing limb in %s" % file_name)

    # find distance to table
    s = string.split(f.readline())
    if len(s) >= 3:
        try:
            distance = float(s[2])
        except ValueError:
            sys.exit("ERROR: invalid distance in %s" % file_name)
    else:
        sys.exit("ERROR: missing distance in %s" % file_name)

    return limb, distance

if __name__=="__main__":

    try:
        rospy.init_node('voicecommands', anonymous = True)

        # get setup parameters---go to function with #2 by it
        limb, distance = get_setup()
        print "limb     = ", limb
        print "distance = ", distance
        # create an instance of the class Locate. 
        locator = Locate(limb, distance)
        rospy.on_shutdown(locator.clean_shutdown) 
        raw_input("Press Enter to start initialization: ")
      
        locator.gripper.open()
    

        # move the arm whose camera you are using close to the board
        # you may wish to change this starting location. 
        locator.pose = [locator.cBoard_tray_x,
                        locator.cBoard_tray_y,
                        locator.cBoard_tray_z,
                        locator.roll, locator.pitch, locator.yaw]
        locator.baxter_ik_move(locator.limb, locator.pose)

        # find the chess board
        locator.find_cBoard_tray()

        # create a list of poses in Baxter's coordinates for each chess board square
        board_spot = list() 
        raw_input("Press Enter to begin voice command operation: ")
        for i in range (65):
            locator.pose = [copy.copy(locator.cBoard_tray_place[i][0]), #-.015
        	copy.copy(locator.cBoard_tray_place[i][1]), #-.045
        	locator.piece_z - .32,
        	locator.roll,
                locator.pitch,
                locator.yaw]
    
            board_spot.append(locator.pose)
        
        #locator.baxter_ik_move(locator.limb, locator.pose)

        voice = Voice()
        print("Say the piece you would like to move or a functional operation.")
        
        while not rospy.is_shutdown():
            # if the user defined a possible move attempt to perform it once
            if voice.movementdefined and not voice.checkedmove and not voice.waitingforcheck:
                #print the desired move to the terminal for reference
                print("Attempting to move " + voice.piecetomove + " to postion " + voice.destination)

                #get index of the piece the player wants to move
                if(voice.whiteturn):
                    pieceindex = game.white_piece_dict[voice.piecetomove]
                else:
                    pieceindex = game.black_piece_dict[voice.piecetomove]

                #get the coordinates of the starting and ending positions for the desired move
                pos1 = 8*(7-pieceindex[1]) + pieceindex[0]
                loc = game.label_to_loc(voice.destination)
                pos2 = 8*(7-loc[1]) + loc[0]

                #check if the move is valid using the virtual board
                validmove = game.check_move(voice.piecetomove,loc,voice.whiteturn, piecekilled)

                #request baxter to perform the physical movement
                if validmove:
                    voice.paused = True
                    if piecekilled == True:
                        print("\nKilling")
                        locator.pick(board_spot[pos2])
                        locator.middle(board_spot[28])
                        locator.place(board_spot[64])
                        locator.middle(board_spot[28])
                        
                    print("\nPicking...")
                    locator.pick(board_spot[pos1])
                    locator.middle(board_spot[28])
                    print("\nPlacing...")
                    updatedpose = copy.copy(board_spot[pos2])
                    updatedpose[2] = board_spot[pos2][2] + .02
                    locator.place(updatedpose)
                    voice.waitingforcheck = True
                    print("Did Baxter successfully make the move requested? \nReply 'Yes' or 'No'.")
                    voice.paused = False
                else:
                    print("Requested move was invalid.\nSay the piece you would like to move or a functional operation.")
                    voice.movementdefined = False
                    voice.piecetomove = 'none'
                    voice.destination = 'nowhere'

            # if baxter successfully executed the move
            # update the virtual board state,
            # change turns,
            # and reset variables to be ready for next verbal command
            if voice.movementdefined and voice.checkedmove:
                if voice.movesuccess:
                    validmove = game.move_piece(voice.piecetomove,voice.destination,voice.whiteturn) # update virtual board state
                    voice.whiteturn = not voice.whiteturn # change turn between white and black
                game.print_board()
                voice.piecetomove = 'none'
                voice.destination = 'nowhere'
                voice.movementdefined = False
                voice.waitingforcheck = False
                voice.checkedmove = False
                print("Say the piece you would like to move or a functional operation.")
            voice.r.sleep()
            
    except rospy.ROSInterruptException:
        rospy.loginfo("Voice command script terminated.")
        reset = Reset()
        reset.set_neutral()
        reset.reset_facescreen()
        reset.grippers_reset()
        reset.disable_motors()
        print("\nBaxter's arms now in neutral position, led display back to rethink log and motors have been disabled.")


