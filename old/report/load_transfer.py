import sympy as sp
from sympy import symbols
from sympy import sympify
sp.init_printing(use_unicode=True)

pprint = sp.pprint


half = sympify("1/2")
m, g = symbols("m g")
lr, lf, lm, h = symbols("lr lf lm h")
bf, br = symbols("bf br")
ax_ch,ay_ch = symbols("ax_ch ay_ch")
FLx, FLz = symbols("FLx FLz")

#ax_ch, ay_ch = -ax, -ay
l = lr + lf

Fzfl, Fzrl, Fzrr, Fzfr = symbols("Fzfl Fzrl Fzrr Fzfr")
Fzf = Fzfl + Fzfr
Fzr = Fzrl + Fzrr

mf,mr = Fzf/g, Fzr/g

G = sp.Matrix([
    [1,1,1,1],
    [l,0,0,l],
    [bf/2,0,0,-bf/2],
    [0,br/2,-br/2,0],
])

A = h/g*ay_ch * sp.Matrix([
    [0,0,0,0],
    [0,0,0,0],
    [1,0,0,1],
    [0,1,1,0],
])

b = sp.Matrix([
    m*g,
    -h*m*ax_ch + lr*m*g,
    0,
    0,
])

bload = sp.Matrix([
    FLz,
    (lr+lm)*FLz - h*FLx,
    0,
    0
])


M = A + G
Fg = M.inv() * b
Fg.simplify()
Fl = M.inv() * bload
Fl.simplify()

Fzfl_load = (-h/l*FLx + (lm+lr)/l * FLz)*(half - (h*ay_ch)/(bf*g))
Fzrl_load = ( h/l*FLx + (lf-lm)/l * FLz)*(half - (h*ay_ch)/(br*g))
Fzrr_load = ( h/l*FLx + (lf-lm)/l * FLz)*(half + (h*ay_ch)/(br*g))
Fzfr_load = (-h/l*FLx + (lm+lr)/l * FLz)*(half + (h*ay_ch)/(bf*g))

# equations (9.51) to (9.54) in kiencke
Fzfl_kiencke = m*(lr/l*g - h/l*ax_ch)*(half - h*ay_ch/(bf*g))
Fzfr_kiencke = m*(lr/l*g - h/l*ax_ch)*(half + h*ay_ch/(bf*g))
Fzrl_kiencke = m*(lf/l*g + h/l*ax_ch)*(half - h*ay_ch/(br*g))
Fzrr_kiencke = m*(lf/l*g + h/l*ax_ch)*(half + h*ay_ch/(br*g))

Fzfl_cale = Fzfl_kiencke + Fzfl_load
Fzrl_cale = Fzfr_kiencke + Fzfr_load
Fzrr_cale = Fzrl_kiencke + Fzrl_load
Fzfr_cale = Fzrr_kiencke + Fzrr_load


# difference without load
diff_fl = (Fg[0] - Fzfl_kiencke).simplify()
diff_rl = (Fg[1] - Fzrl_kiencke).simplify()
diff_rr = (Fg[2] - Fzrr_kiencke).simplify()
diff_fr = (Fg[3] - Fzfr_kiencke).simplify()

print("Unloaded differences from Kiencke")
print(diff_fl)
print(diff_rl)
print(diff_rr)
print(diff_fr)

print("Acceleration and geometry")
pprint(Fzfl_kiencke)
pprint(Fzrl_kiencke)
pprint(Fzrr_kiencke)
pprint(Fzfr_kiencke)

print("Nice load contributions")
pprint(Fzfl_load)
pprint(Fzrl_load)
pprint(Fzrr_load)
pprint(Fzfr_load)

print("Difference from computed load contributions")
diff_fl = (Fl[0] - Fzfl_load).simplify()
diff_rl = (Fl[1] - Fzrl_load).simplify()
diff_rr = (Fl[2] - Fzrr_load).simplify()
diff_fr = (Fl[3] - Fzfr_load).simplify()
print(diff_fl)
print(diff_rl)
print(diff_rr)
print(diff_fr)

print("Complete equations for wheel load")
pprint(Fzfl_cale)
pprint(Fzrl_cale)
pprint(Fzrr_cale)
pprint(Fzfr_cale)


