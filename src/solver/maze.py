class Maze:
    def __init__(self):
        for i in range(8):
            for j in range(8):
                nb = 15
                if i == 0:
                    nb -= 1
                if i == 7:
                    nb -= 2
                if j == 0:
                    nb -= 4
                if j == 7:
                    nb -= 8
                self.lab.append(nb)
        self.draw()
        loop = int(input("ajouter mur 0/1"))
        while loop:
            x = int(input("pos x"))
            y = int(input("pos y"))
            o = int(input("0:bas 1:droite"))
            if o:
                self.lab[8 * x + y] -= 2
                self.lab[8 * (x + 1) + y] -= 1
            else:
                self.lab[8 * x + y] -= 8
                self.lab[8 * x + y + 1] -= 4
            self.draw()
            loop = int(input("ajouter mur 0/1"))

    def next(self, pos, move):
        assert(move >= 0 & move < 12)
        if self.lab[0] == 1:
            pos.loc = 1
        return pos

    def draw(self):
        print('\u250c', end='')
        for i in range(7):
            print('\u2500\u252c', end='')
        print('\u2500\u2510')
        for i in range(8):
            print('\u2502', end='')
            for j in range(8):
                print(' ', end='')
                if self.lab[8*j+i]//2 % 2 == 0:
                    print('\u2502', end='')
                else:
                    print(' ', end='')
            if i != 7:
                print('\n\u251c', end='')
            else:
                print('\n\u2514', end='')
            for j in range(7):
                if self.lab[8*j+i]//8 % 2 == 0:
                    print('\u2500', end='')
                else:
                    print(' ', end='')
                if i != 7:
                    print('\u253c', end='')
                else:
                    print('\u2534', end='')
            if self.lab[56+i]//8 % 2 == 0:
                print('\u2500', end='')
            else:
                print(' ', end='')
            if i != 7:
                print('\u2524')
            else:
                print('\u2518')

    lab = []


class Pos:
    loc: int = 0
    move = []
