from maze import Maze
from maze import Pos


def solve_conf(lab, pos, end: int):
    todo = [pos]
    visit = {pos.loc}
    while len(todo) > 0:
        current = todo[0]
        del todo[0]
        for i in range(12):
            new = lab.next(current, i)
            if not(new.loc in visit):
                if new.loc % 64 == end:
                    return new.move
                todo.append(new)
                visit.add(new.loc)
    return


def main():
    lab = Maze()
    init = Pos()
    try:
        x = int(input("pos X robot C : "))
        y = int(input("pos Y robot C : "))
        init.loc = 8 * x + y
        x = int(input("pos X robot B : "))
        y = int(input("pos Y robot B : "))
        init.loc = 64*init.loc + 8 * x + y
        x = int(input("pos X robot A : "))
        y = int(input("pos Y robot A : "))
        init.loc = 64*init.loc + 8 * x + y
        x: int = int(input("pos X arrive : "))
        y: int = int(input("pos Y arrive : "))
        target = 8 * x + y
    except:
        print("give integer please")
        return
    res = solve_conf(lab, init, target)
    print(res)
    return


def test():
    print('\u250c\u252c\u2500\u2510\n'
          '\u251c\u253c\u2500\u2524\n'
          '\u2502\u2502 \u2502\n'
          '\u2514\u2534\u2500\u2518')
