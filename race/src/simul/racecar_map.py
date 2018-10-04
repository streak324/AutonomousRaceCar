import tkinter

class Track():

    def __init__(self):
        # (y1, y2, x1, x2)
        middle = Square(150,450,150,650)
        middle1 = Square(150,450,150,340)
        middle2 = Square(150,450,460,650)
        left_wall = Square(0,600,0,20)
        right_wall = Square(0,600,780,800)
        top_wall = Square(0,20,0,800)
        bottom_wall = Square(580,600,0,800)
        obstacle = Square(65,80,400,450)
        obstacle2 = Square(80,120,200,230)
        obstacle3 = Square(280,300,700,730)
        wall1 = Square(350,400,650,690)
        wall2 = Square(350,400,740,780)
        obstacle1 = Square(60,90,230,250)
        obstacle4 = Square(100,130,360,410)
        obstacle5 = Square(50,60,480,530)
        obstacle6 = Square(80,110,600,650)
        block1 = Square(450,470,250,340)
        block2 = Square(505,525,250,340)
        block3 = Square(560,580,250,340)
        
        self.obstacles = [middle1, middle2, left_wall, right_wall, top_wall, bottom_wall, obstacle1,
                         obstacle4, obstacle5, obstacle6, obstacle3, wall1, wall2, block1, block2, block3]
        # self.obstacles = [middle1, middle2, left_wall, right_wall, top_wall, bottom_wall]

    def draw(self, screen:"tkinter.Canvas"):
        color = "#FF0000"
        for item in self.obstacles:
            screen.create_rectangle(item.left, item.top, item.right, item.bottom, outline = color)

    def check_collision(self, point:"touple"):
        for item in self.obstacles:
            if item.intersect(point):
                return True
        return False


class Square():

    def __init__(self, top:float, bottom:float, left:float, right:float):
        self.top = top
        self.bottom = bottom
        self.left = left
        self.right = right

    def intersect(self, point:"touple"):
        if(point[0] <= self.right+5 and point[0] >= self.left-5 and point[1] >= self.top-5 and point[1] <= self.bottom+5):
            return True
        else:
            return False
