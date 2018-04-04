from Piece import Piece
class Pawn(Piece):
    def __init__(self):
        Piece.__init__(self)
        self.init_loc = True

    def set_init_loc(self,init_loc):
        self.init_loc = init_loc
