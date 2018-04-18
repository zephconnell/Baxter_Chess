
class Piece():
    def __init__(self):
        self.labelp = " "
        self.isWhite = True
        self.color_string = "None"
        self.locXp = 0
        self.locYp = 0
        self.locp = [self.locXp,self.locYp]

    def identity(self):
        return self.__class__.__name__
    def set_labelp(self,label):
        self.labelp = label
    def return_labelp(self):
        return self.labelp

    def return_locp(self):
        return self.locp
    def set_locp(self,Xp,Yp):
        self.locp = [Xp,Yp]
        self.locX = Xp
        self.locY = Yp
    def return_color_string(self):
        return self.color_string
    def set_color_string(self,color):
        self.color_string = color

    def return_colorp(self):
        return self.isWhite
    def change_colorp(self):
        self.isWhite = not self.isWhite
    def set_colorp(self,colorp):
        self.isWhite = colorp
    def return_locXp(self):
        return self.locXp
    def return_locYp(self):
        return self.locYp
    def set_locXp(self,Xp):
        self.locXp = Xp
        self.locp = [self.locXp,self.locYp]
    def set_locYp(self,Yp):
        self.locYp = Yp
        self.locp = [self.locXp,self.locYp]
