

class motor:
    def __init__(self,color,weight,health):
        self.color  = color
        self.weight = weight
        self.health = health
        
    def attributes(self):
        color=self.color
        weight=self.weight
        health=self.health
        return color,weight,health
    def changecolor(self, newcolor):
        self.color = newcolor
        
def tomatoes(a,newcolor):
    a.changecolor(newcolor)
