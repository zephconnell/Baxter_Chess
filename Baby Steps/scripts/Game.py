from __future__ import print_function
from Board import Board
from Null import Null


class Game():
    def __init__(self):

        self.initial_board = Board()
        self.piece_dict = {}
        self.white_piece_dict = {}
        self.black_piece_dict = {}
        self.killed_piece = False
        
    def label_to_loc(self,label):
       loc = []
       letters = ['a','b','c','d','e','f','g','h']
       numbers = ['1','2','3','4','5','6','7','8']

       
       for i in range(8):
           if(letters[i]==label[0]):
               loc.append(i)
       for j in range(8):
           if(numbers[j]==label[1]):
               loc.append(j)
       return loc
                
    def print_board(self):

        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_piece().identity(), end = " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_piece().return_color_string(), end = " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_piece().return_labelp(), end = " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_loc(), end = " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_color(), end = " ")
            print("\n")

    def create_piece_dict(self):
        self.create_white_piece_dict()
        self.create_black_piece_dict()
    def create_white_piece_dict(self):
        i = 0
        j = 0
        #Pawns
        for n in range(8):
            i,j = self.initial_board.xy_to_ij(n,1)
            key = self.initial_board.board[i][j].return_piece().return_labelp()
            value = self.initial_board.board[i][j].return_piece().return_locp()
            self.white_piece_dict[key] = value
        #Queen
        i,j = self.initial_board.xy_to_ij(3,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        #Rooks
        i,j = self.initial_board.xy_to_ij(0,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        i,j = self.initial_board.xy_to_ij(7,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        #Bishops
        i,j = self.initial_board.xy_to_ij(2,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        i,j = self.initial_board.xy_to_ij(5,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        #King
        i,j = self.initial_board.xy_to_ij(4,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        #Knights
        i,j = self.initial_board.xy_to_ij(1,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
        i,j = self.initial_board.xy_to_ij(6,0)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.white_piece_dict[key] = value
    def create_black_piece_dict(self):
        i = 0
        j = 0
        #Pawns
        for n in range(8):
            i,j = self.initial_board.xy_to_ij(n,6)
            key = self.initial_board.board[i][j].return_piece().return_labelp()
            value = self.initial_board.board[i][j].return_piece().return_locp()
            self.black_piece_dict[key] = value
        #Queen
        i,j = self.initial_board.xy_to_ij(3,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        #Rooks
        i,j = self.initial_board.xy_to_ij(0,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        
        i,j = self.initial_board.xy_to_ij(7,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        #Bishops
        i,j = self.initial_board.xy_to_ij(2,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        i,j = self.initial_board.xy_to_ij(5,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        #King
        i,j = self.initial_board.xy_to_ij(4,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        #Knights
        i,j = self.initial_board.xy_to_ij(1,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
        i,j = self.initial_board.xy_to_ij(6,7)
        key = self.initial_board.board[i][j].return_piece().return_labelp()
        value = self.initial_board.board[i][j].return_piece().return_locp()
        self.black_piece_dict[key] = value
	
    #function which prints the contents of white and black dictionaries
    def print_piece_dict(self):
        for key,value in self.white_piece_dict.items():
            print(key,":",value)
        print('\n')
        for key,value in self.black_piece_dict.items():
            print(key,":",value)
        
    #function which iterates through the dictionary keys and returns if there is 
    #a match to label
    def check_piece_dict(self,label,white_turn):
        if(white_turn):
            for key in self.white_piece_dict.keys():
                if(label==key):
                    return True
        else:
            for key in self.black_piece_dict.keys():
                if(label==key):
                    return True
        return False
    #function which generates a list of valid moves for a piece which wants to move
    def generate_moves(self,locp,white_turn):
        i = 0
        j = 0
        move_list = []
        piece_type = " "
        i,j = self.initial_board.xy_to_ij(locp[0],locp[1])
        piece = self.initial_board.board[i][j].return_piece()
        piece_type = piece.identity()
        currentX = piece.return_locp()[0]
        currentY = piece.return_locp()[1]
        
        if(piece_type == "Pawn"):
            if(white_turn):
                d1 = self.pawn_d1(currentX,currentY)
                u = self.pawn_u(currentX,currentY)
                u2 = self.pawn_u2(currentX,currentY)
                d2 = self.pawn_d2(currentX,currentY)
                
                if(d1):
                    move_list.append([currentX-1,currentY+1])
                if(u):
                    move_list.append([currentX,currentY+1])
                if(u2):
                    move_list.append([currentX,currentY+2])
                if(d2):
                    move_list.append([currentX+1,currentY+1])
            else:
                d4 = self.pawn_d4(currentX,currentY)
                l  = self.pawn_l(currentX,currentY)
                l2 = self.pawn_l2(currentX,currentY)
                d3 = self.pawn_d3(currentX,currentY)
        
                if(d4):
                    move_list.append([currentX-1,currentY-1])
                if(l):
                    move_list.append([currentX,currentY-1])
                if(l2):
                    move_list.append([currentX,currentY-2])
                if(d3):
                    move_list.append([currentX+1,currentY-1])
        elif(piece_type == "Queen"):
            move_list = self.f_moves(currentX,currentY,move_list)
            move_list = self.d1_moves(currentX,currentY,move_list)
            move_list = self.u_moves(currentX,currentY,move_list)
            move_list = self.d2_moves(currentX,currentY,move_list)
            move_list = self.r_moves(currentX,currentY,move_list)
            move_list = self.d3_moves(currentX,currentY,move_list)
            move_list = self.l_moves(currentX,currentY,move_list)
            move_list = self.d4_moves(currentX,currentY,move_list)
        elif(piece_type == "Rook"):
            move_list = self.f_moves(currentX,currentY,move_list)
            move_list = self.u_moves(currentX,currentY,move_list)
            move_list = self.r_moves(currentX,currentY,move_list)
            move_list = self.l_moves(currentX,currentY,move_list)
        elif(piece_type == "Bishop"):
            move_list = self.d1_moves(currentX,currentY,move_list)
            move_list = self.d2_moves(currentX,currentY,move_list)
            move_list = self.d3_moves(currentX,currentY,move_list)
            move_list = self.d4_moves(currentX,currentY,move_list)
        elif(piece_type == "King"):
            move_list = self.f_move(currentX,currentY,move_list)
            move_list = self.d1_move(currentX,currentY,move_list)
            move_list = self.u_move(currentX,currentY,move_list)
            move_list = self.d2_move(currentX,currentY,move_list)
            move_list = self.r_move(currentX,currentY,move_list)
            move_list = self.d3_move(currentX,currentY,move_list)
            move_list = self.l_move(currentX,currentY,move_list)
            move_list = self.d4_move(currentX,currentY,move_list)
        elif(piece_type == "Knight"):
            move_list = self.n1_move(currentX,currentY,move_list)
            move_list = self.n2_move(currentX,currentY,move_list)
            move_list = self.n3_move(currentX,currentY,move_list)
            move_list = self.n4_move(currentX,currentY,move_list)
            move_list = self.n5_move(currentX,currentY,move_list)
            move_list = self.n6_move(currentX,currentY,move_list)
            move_list = self.n7_move(currentX,currentY,move_list)
            move_list = self.n8_move(currentX,currentY,move_list)
            
        return move_list

    ''' PAWN CARDINAL DIRECTION BOOLEAN FUNCTIONS '''

    def pawn_u(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentY+1 > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX,currentY+1)

        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color == "None"):
            return True
        else:
            return False
        
    def pawn_l(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentY-1 < 0):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX,currentY-1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
        
        if(enemy_color == "None"):
            return True
        else:
            return False
    def pawn_u2(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentY+2 > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX,currentY+2)

        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(self.initial_board.board[i][j].return_piece().init_loc == False):
            return False
        if(enemy_color == "None"):
            return True
        else:
            return False
    def pawn_l2(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentY-2 < 0):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX,currentY-2)
    
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
    
        if(self.initial_board.board[i][j].return_piece().init_loc == False):
            return False
        if(enemy_color == "None"):
            return True
        else:
            return False

    def pawn_d1(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentX-1 < 0 or currentY+1 > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY+1)

        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color == "None"):
            return False
        else:
            if(enemy_color == piece_color):
                return False
            else:
                return True
    def pawn_d2(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentX+1 > 7 or currentY+1 > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY+1)

        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color == "None"):
            return False
        else:
            if(enemy_color == piece_color):
                return False
            else:
                return True
    def pawn_d3(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentX+1 > 7 or currentY-1 < 0):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY-1)

        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color == "None"):
            return False
        else:
            if(enemy_color == piece_color):
                return False
            else:
                return True
    def pawn_d4(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentX-1 < 0 or currentY-1 < 0):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY-1)

        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color == "None"):
            return False
        else:
            if(enemy_color == piece_color):
                return False
            else:
                return True
    '''END OF PAWN CARDINAL DIRECTION BOOLEAN FUNCTIONS '''
    '''START OF KNIGHT SPECIFIC MOVEMENT CHECKS'''
    def n1_move(self,currentX,currentY,move_list):
        X = currentX-1
        Y = currentY+2
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n2_move(self,currentX,currentY,move_list):
        X = currentX+1
        Y = currentY+2
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n3_move(self,currentX,currentY,move_list):
        X = currentX+2
        Y = currentY+1
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n4_move(self,currentX,currentY,move_list):
        X = currentX+2
        Y = currentY-1
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n5_move(self,currentX,currentY,move_list):
        X = currentX+1
        Y = currentY-2
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n6_move(self,currentX,currentY,move_list):
        X = currentX-1
        Y = currentY-2
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n7_move(self,currentX,currentY,move_list):
        X = currentX-2
        Y = currentY-1
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
    def n8_move(self,currentX,currentY,move_list):
        X = currentX-2
        Y = currentY+1
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check = self.n_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y])
            return move_list
        else:
            return move_list
'''END OF KNIGHT SPECIFIC MOVEMENT CHECKS'''
'''START OF KING SPECIFIC MOVEMENT CHECKS'''
    def f_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.f_check(X,Y,piece_color)
        if(check):
            move_list.append([X-1,Y])
            return move_list
        else:
            return move_list
    def d1_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d1_check(X,Y,piece_color)
        if(check):
            move_list.append([X-1,Y+1])
            return move_list
        else:
            return move_list
    def u_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.u_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y+1])
            return move_list
        else:
            return move_list
    
    def d2_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d2_check(X,Y,piece_color)
        if(check):
            move_list.append([X+1,Y+1])
            return move_list
        else:
            return move_list
    def r_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.r_check(X,Y,piece_color)
        if(check):
            move_list.append([X+1,Y])
            return move_list
        else:
            return move_list
    def d3_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d3_check(X,Y,piece_color)
        if(check):
            move_list.append([X+1,Y-1])
            return move_list
        else:
            return move_list
    def l_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.l_check(X,Y,piece_color)
        if(check):
            move_list.append([X,Y-1])
            return move_list
        else:
            return move_list
    def d4_move(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d4_check(X,Y,piece_color)
        if(check):
            move_list.append([X-1,Y-1])
            return move_list
        else:
            return move_list
     '''END OF KING SPECIFIC MOVEMENT CHECKS'''
     '''START OF CARDINAL DIRECTION CHECKS'''   
    def d4_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentX-1 < 0 or currentY-1 < 0):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY-1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
     
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def d3_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentX+1 > 7 or currentY-1 < 0):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY-1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
     
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def d2_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentX+1 > 7 or currentY+1 > 7):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY+1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
     
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def d1_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentX-1 < 0 or currentY+1 > 7):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY+1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
     
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def l_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentY-1 < 0):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX,currentY-1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
     
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def u_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentY+1 > 7):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX,currentY+1)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
     
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def r_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentX+1 > 7):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
        
        
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
    def f_check(self,currentX,currentY,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        
        if(currentX-1 < 0):
            return False,False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
        
        if(enemy_color == "None"):
            return True,False
        else:
            if(enemy_color == piece_color):
                return False,False
            else:
                return True,True
     '''END of CARDINAL DIRECTION CHECKS'''
    #function which does a check for all knight movements
    def n_check(self,X,Y,piece_color):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        if(X < 0 or X > 7 or Y < 0 or Y > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(X,Y)
        
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string()
        
        if(enemy_color == "None"):
            return True
        else:
            if(enemy_color == piece_color):
                return False
            else:
                return True
    #START OF QUEEN/BISHOP/ROOK MOVEMENT CHECKS
    def d4_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d4_check(X,Y,piece_color)
        while(check):
            move_list.append([X-1,Y-1])
            Y = Y - 1
            X = X - 1
          
            if(enemy_piece):
                break
            check,enemy_piece = self.d4_check(X,Y,piece_color)
        return move_list
    def d3_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d3_check(X,Y,piece_color)
        while(check):
            move_list.append([X+1,Y-1])
            Y = Y - 1
            X = X + 1
          
            if(enemy_piece):
                break
            check,enemy_piece = self.d3_check(X,Y,piece_color)
        return move_list
    def d2_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d2_check(X,Y,piece_color)
        while(check):
            move_list.append([X+1,Y+1])
            Y = Y + 1
            X = X + 1
          
            if(enemy_piece):
                break
            check,enemy_piece = self.d2_check(X,Y,piece_color)
        return move_list
    def d1_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.d1_check(X,Y,piece_color)
        while(check):
            
            move_list.append([X-1,Y+1])
            Y = Y + 1
            X = X - 1
          
            if(enemy_piece):
                break
            check,enemy_piece = self.d1_check(X,Y,piece_color)
        return move_list
    def l_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.l_check(X,Y,piece_color)
        while(check):
            move_list.append([X,Y-1])
            Y = Y - 1
            if(enemy_piece):
                break
            check,enemy_piece = self.l_check(X,Y,piece_color)
        return move_list
    def u_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.u_check(X,Y,piece_color)
        while(check):
            move_list.append([X,Y+1])
            Y = Y + 1
            if(enemy_piece):
                break
            check,enemy_piece = self.u_check(X,Y,piece_color)
        return move_list
    def f_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.f_check(X,Y,piece_color)
        while(check):
            move_list.append([X-1,Y])
            X = X - 1
            if(enemy_piece):
                break
            check,enemy_piece = self.f_check(X,Y,piece_color)
        return move_list   
    
    def r_moves(self,currentX,currentY,move_list):
        X = currentX
        Y = currentY
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(X,Y)
        piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
        check,enemy_piece = self.r_check(X,Y,piece_color)
        while(check):
            move_list.append([X+1,Y])
            X = X + 1
            if(enemy_piece):
                break
            check,enemy_piece = self.r_check(X,Y,piece_color)
        return move_list
    '''END OF KING/BISHOP/ROOK MOVEMENT CHECKS'''
    #function that was used for testing purposes      
    def place_piece(self,piece,loc,colorp):
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(loc[0],loc[1])
        self.initial_board.board[i][j].change_square_state()
        self.initial_board.board[i][j].set_piece(piece)
        self.initial_board.board[i][j].return_piece().set_locp(loc[0],loc[1])
        self.initial_board.board[i][j].return_piece().set_color_string(colorp)
        self.initial_board.board[i][j].return_piece().set_labelp("PT")
    #function which is used to remove a key value pair from a either the white or black dictionary.
    def remove_from_dict(self,label,white_turn):
	if(white_turn):
		del self.black_piece_dict[label]
	else:
		del self.white_piece_dict[label]
    #function which determines if the king piece has been killed or not
    def check_king_state(self,white_turn):
	if(white_turn):
	    for key in self.black_piece_dict.keys():
                if('K'==key):
		    return True
	else:
	    for key in self.white_piece_dict.keys():
                if('K'==key):
		    return True
	return False    
    #function which returns true if a move is valid false otherwise
    def check_move(self,label,loc,white_turn):
	self.killed_piece = False
        if(self.check_piece_dict(label,white_turn)):
            if(white_turn):
                current_loc = self.white_piece_dict[label]
            else:
                current_loc = self.black_piece_dict[label]
            i,j,newi,newj = 0,0,0,0 
	    i,j = self.initial_board.xy_to_ij(current_loc[0],current_loc[1])
	    newi,newj = self.initial_board.xy_to_ij(loc[0],loc[1])
	    piece_color = self.initial_board.board[i][j].return_piece().return_color_string()
	    enemy_color = self.initial_board.board[newi][newj].return_piece().return_color_string() 
            for move in self.generate_moves(current_loc,white_turn):
                if(move == loc):
                    if(enemy_color == "None"):  
                    	self.killed_piece = False
			
		    else:
			if(piece_color == enemy_color):
			    self.killed_piece = False
		
			else:
			    self.killed_piece = True
			    
		    return True,self.killed_piece
            return False,self.killed_piece
        else:
            return False,self.killed_piece
    #function which takes care of moving a piece and updating the board state
    def move_piece(self,label,loc_label,white_turn):
        i = 0
        j = 0
        newi = 0
        newj = 0
        current_init_loc = True
        loc = self.label_to_loc(loc_label)
     	if(self.check_piece_dict(label,white_turn)):
	    if(white_turn):
                current_loc = self.white_piece_dict[label]
	    else:
                current_loc = self.black_piece_dict[label]
	else:
	    print("Invalid Move")
            return False	    
        i,j = self.initial_board.xy_to_ij(current_loc[0],current_loc[1])
        newi,newj = self.initial_board.xy_to_ij(loc[0],loc[1])
        piece_type = self.initial_board.board[i][j].return_piece().identity()
        check,isKilled = self.check_move(label,loc,white_turn)
	victim_label = self.initial_board.board[newi][newj].return_piece().return_labelp()
	print(isKilled)
	
        if isKilled:
            self.remove_from_dict(victim_label,white_turn)
	
        if(check):
            if( piece_type == "Pawn"):
                self.initial_board.board[i][j].return_piece().set_init_loc(False)
                current_init_loc = self.initial_board.board[i][j].return_piece().init_loc
                  #if(abs(loc[1]-current_loc[1])==2):#just for pawns
                    
            current_piece = self.initial_board.board[i][j].return_piece()
            current_piece_color = self.initial_board.board[i][j].return_piece().return_colorp()
	    current_color_string = self.initial_board.board[i][j].return_piece().return_color_string()
            current_labelp = self.initial_board.board[i][j].return_piece().return_labelp()


            self.initial_board.board[i][j].change_square_state()
            self.initial_board.board[newi][newj].change_square_state()

            self.initial_board.board[i][j].set_piece(Null())
            self.initial_board.board[i][j].return_piece().set_labelp("Null")
            self.initial_board.board[i][j].return_piece().set_locp(current_loc[0],current_loc[1])
            self.initial_board.board[i][j].return_piece().set_color_string("None")

            self.initial_board.board[newi][newj].set_piece(current_piece)
            self.initial_board.board[newi][newj].return_piece().set_colorp(current_piece_color)
            self.initial_board.board[newi][newj].return_piece().set_labelp(current_labelp)
            self.initial_board.board[newi][newj].return_piece().set_locp(loc[0],loc[1])
            self.initial_board.board[newi][newj].return_piece().set_color_string(current_color_string)
            if (piece_type == "Pawn"):
                self.initial_board.board[newi][newj].return_piece().set_init_loc(current_init_loc)

            if(white_turn):
                self.white_piece_dict[label] = loc
            else:
                self.black_piece_dict[label] = loc
            return True
        else:
            print("Invalid Move")
            return False
