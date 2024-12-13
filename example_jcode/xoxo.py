import math
import numpy as np


cw = 2
c = 10

print("S 5;")

for x in [-cw/2, cw/2]:
    print(f"P U;W {x} {c-cw*1.5};D 1000;P D;W {x} {c+cw*1.5};P U;")

for y in [-cw/2, cw/2]:
    print(f"P U;W {-cw*1.5} {y+c} ;D 1000;P D;W {cw*1.5} {y+c} ;P U;")

print("S 2;")

def draw_x(cell_x, cell_y):
    cc = (cw*cell_x, cw*cell_y+c)
    for dir in [(cw, cw), (cw, -cw), (-cw, cw), (-cw, -cw)]:
        res = np.array(dir)*0.4+np.array(cc)
        print(f"P U;W {cc[0]} {cc[1]};D 500;P D;W {res[0]} {res[1]};P U;")

def draw_o(cell_x, cell_y):
    cc = (cw*cell_x, cw*cell_y+c)
    print(f"P U;W {cc[0]+cw*0.4} {cc[1]};D 500;P D;")
    for i in np.arange(0, math.pi*2, 0.02):
        print(f"W {cc[0]+cw*0.4*math.cos(i)} {cc[1]+cw*0.4*math.sin(i)};")
    print("P U;")
        

draw_x(0,0)
draw_o(1,0)
draw_x(0,1)
draw_o(0,-1)
draw_x(1,1)
draw_o(-1,-1)
draw_x(-1,1)
print(f"W {cw*-1.2} {cw*1+c};D 500;P D;W {cw*1.2} {cw*1+c};P U;")