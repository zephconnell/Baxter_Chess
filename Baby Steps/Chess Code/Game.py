from Board import Board
from Null import Null


class Game():
    def __init__(self):

        self.initial_board = Board()
        self.piece_dict = {}
        self.white_piece_dict = {}
        self.black_piece_dict = {}
        
        
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
                print(self.initial_board.board[i][j].return_piece().identity(),end= " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_piece().return_color_string(),end= " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_piece().return_labelp(),end= " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_piece().return_locp(),end= " ")
            print("\n")
        for i in range(8):
            for j in range(8):
                print(self.initial_board.board[i][j].return_color(),end= " ")
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
    def create_black_piece_dict(self):
        i = 0
        j = 0
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
    def print_piece_dict(self):
        for key,value in self.white_piece_dict.items():
            print(key,":",value)
        print('\n')
        for key,value in self.black_piece_dict.items():
            print(key,":",value)
        

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
            ...
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

        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color_string == "None"):
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
        
        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()
        
        if(enemy_color_string == "None"):
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

        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(self.initial_board.board[i][j].return_piece().init_loc == False):
            return False
        if(enemy_color_string == "None"):
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
    
        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()
    
        if(self.initial_board.board[i][j].return_piece().init_loc == False):
            return False
        if(enemy_color_string == "None"):
            return True
        else:
            return False

    def pawn_d1(self,currentX,currentY):
        i = 0
        newi = 0
        j = 0
        newj = 0
        i,j = self.initial_board.xy_to_ij(currentX,currentY)
        if(currentX-1 < 0):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY+1)

        piece_color = self.initial_board.board[i][j].return_piece().return_colorp()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_colorp()
        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()


        if(enemy_color_string == "None"):
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
        if(currentX+1 > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY+1)

        piece_color = self.initial_board.board[i][j].return_piece().return_colorp()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_colorp()
        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()

        if(enemy_color_string == "None"):
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
        if(currentX+1 > 7):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX+1,currentY-1)

        piece_color = self.initial_board.board[i][j].return_piece().return_colorp()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_colorp()
        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()


        if(enemy_color_string == "None"):
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
        if(currentX-1 < 0):
            return False
        else:
            newi,newj = self.initial_board.xy_to_ij(currentX-1,currentY-1)

        piece_color = self.initial_board.board[i][j].return_piece().return_colorp()
        enemy_color = self.initial_board.board[newi][newj].return_piece().return_colorp()
        enemy_color_string = self.initial_board.board[newi][newj].return_piece().return_color_string()


        if(enemy_color_string == "None"):
            return False
        else:
            if(enemy_color == piece_color):
                return False
            else:
                return True
    '''END OF PAWN CARDINAL DIRECTION BOOLEAN FUNCTIONS '''
    
    def place_piece(self,piece,loc,colorp):
        i = 0
        j = 0
        i,j = self.initial_board.xy_to_ij(loc[0],loc[1])
        self.initial_board.board[i][j].change_square_state()
        self.initial_board.board[i][j].set_piece(piece)
        self.initial_board.board[i][j].return_piece().set_locp(loc[0],loc[1])
        self.initial_board.board[i][j].return_piece().set_colorp(colorp)
        self.initial_board.board[i][j].return_piece().set_labelp("PT")

    def check_move(self,label,loc,white_turn):
        if(self.check_piece_dict(label,white_turn)):
            if(white_turn):
                current_loc = self.white_piece_dict[label]
            else:
                current_loc = self.black_piece_dict[label]
                
            for move in self.generate_moves(current_loc,white_turn):
                if(move == loc):
                    return True
            return False
        else:
            return False
    def move_piece(self,label,loc_label,white_turn):
        i = 0
        j = 0
        newi = 0
        newj = 0
        
        loc = self.label_to_loc(loc_label)
        if(white_turn):
            current_loc = self.white_piece_dict[label]
        else:
            current_loc = self.black_piece_dict[label]
            
        i,j = self.initial_board.xy_to_ij(current_loc[0],current_loc[1])
        newi,newj = self.initial_board.xy_to_ij(loc[0],loc[1])
        piece_type = self.initial_board.board[i][j].return_piece().identity()
        
        if(self.check_move(label,loc,white_turn)):
            if( piece_type == "Pawn"):
                if(abs(loc[1]-current_loc[1])==2):#just for pawns
                    self.initial_board.board[i][j].return_piece().set_init_loc(False)
                    current_init_loc = self.initial_board.board[i][j].return_piece().init_loc
         
            current_piece = self.initial_board.board[i][j].return_piece()
            current_piece_color = self.initial_board.board[i][j].return_piece().return_colorp()
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
            
            if (piece_type == "Pawn"):
                self.initial_board.board[newi][newj].return_piece().set_init_loc(current_init_loc)
            
            if(white_turn):
                self.white_piece_dict[label] = loc
            else:
                self.black_piece_dict[label] = loc
        else:
            print("Invalid Move")
