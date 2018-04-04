
import numpy as np
from Pawn import Pawn
from Square import Square
from Null import Null
from Queen import Queen

class Board():

    def __init__(self):
        letters = ['a','b','c','d','e','f','g','h']
        numbers = ['1','2','3','4','5','6','7','8']

        self.board = np.empty(shape=[8,8],dtype = Square)
        x = 0
        y = 0

        for i in range(8):

            for j in range(8):
                self.board[i][j] = Square()
                x,y = self.ij_to_xy(i,j)

                self.board[i][j].set_piece(Null())
                self.board[i][j].return_piece().change_colorp()
                self.board[i][j].set_loc(x,y)
                self.board[i][j].set_label(letters[x]+numbers[y])
                self.board[i][j].return_piece().set_labelp("Null")
                self.board[i][j].return_piece().set_locp(x,y)
                

                if(i%2==0):
                    if(j%2==0):
                        self.board[i][j].change_color()
                else:
                    if(j%2!=0):
                        self.board[i][j].change_color()

    def initialize_board(self):
        i = 0
        j = 0
        pawn_labels = ["P1","P2","P3","P4","P5","P6","P7","P8"]
        queen_label = "Q"
        
        #White Pawns
        for n in range(8):
            i,j = self.xy_to_ij(n,1)
            self.board[i][j].change_square_state()
            self.board[i][j].set_piece(Pawn())
            self.board[i][j].return_piece().set_labelp(pawn_labels[n])
            self.board[i][j].return_piece().set_locp(n,1)
            self.board[i][j].return_piece().set_color_string("White")
        #White Queen
        
            i,j = self.xy_to_ij(3,0)
            self.board[i][j].change_square_state()
            self.board[i][j].set_piece(Queen())
            self.board[i][j].return_piece().set_labelp(queen_label)
            self.board[i][j].return_piece().set_locp(3,0)
            self.board[i][j].return_piece().set_color_string("White")
        
        for n in range(8):
            i,j = self.xy_to_ij(n,6)
            self.board[i][j].change_square_state()
            self.board[i][j].set_piece(Pawn())
            self.board[i][j].return_piece().change_colorp()
            self.board[i][j].return_piece().set_labelp(pawn_labels[n])
            self.board[i][j].return_piece().set_locp(n,6)
            self.board[i][j].return_piece().set_color_string("Black")
        #White Queen
    
        i,j = self.xy_to_ij(3,7)
        self.board[i][j].change_square_state()
        self.board[i][j].set_piece(Queen())
        self.board[i][j].return_piece().set_labelp(queen_label)
        self.board[i][j].return_piece().set_locp(3,7)
        self.board[i][j].return_piece().set_color_string("Black")
        
        

    def ij_to_xy(self,i,j):
        x = j
        y = 7 - i
        return x,y
    def xy_to_ij(self,x,y):
        i = 7 - y
        j = x
        return i,j
