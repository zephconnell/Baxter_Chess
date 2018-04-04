from Piece import Piece
class Square():
    def __init__(self):
        self.hasPiece = False
        self.isBlack = True
        self.label = "a1"
        self.piece = Piece()

        self.locX = 0
        self.locY = 0
        self.loc = [self.locX,self.locY]

    def change_square_state(self):
        self.hasPiece = not self.hasPiece
    def return_loc(self):
        return self.loc

    def return_locX(self):
        return self.real_locX

    def return_locY(self):
        return self.locY

    def set_loc(self,X,Y):
        self.loc = [X,Y]
        self.locX = X
        self.locY = Y

    def set_locX(self,X):
        self.locX = X
        self.loc = [self.locX,self.locY]

    def set_locY(self,Y):
        self.locY = Y
        self.loc = [self.locX,self.locY]

    def change_color(self):
        self.isBlack = not self.isBlack
    def return_color(self):
        return self.isBlack
    def set_label(self,label):
        self.label = label
    def return_label(self):
        return self.label
    def return_square_state(self):
        return self.hasPiece
    def set_piece(self,piece):
        self.piece = piece
    def return_piece(self):
        return self.piece
