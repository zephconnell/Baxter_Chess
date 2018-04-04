from Game import Game



g = Game()
g.initial_board.initialize_board()

g.create_piece_dict()

g.move_piece("P4","d4",True)
g.move_piece("P4","d6",True)




g.print_board()