from Game import Game
from Pawn import Pawn


g = Game()
g.initial_board.initialize_board()
g.create_piece_dict()
g.print_piece_dict()

g.remove_from_dict("P1",True)

g.remove_from_dict("P1",False)
g.print_piece_dict()

