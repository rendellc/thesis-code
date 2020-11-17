import sympy as sp
from sympy import symbols
from sympy import sympify
sp.init_printing(use_unicode=True)


m, g = symbols("m g")
lr, lf, h = symbols("lr lf h")
bf, br = symbols("bf br")
ax,ay = symbols("ax ay")

ax_ch, ay_ch = -ax, -ay
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


M = A + G
Fs = M.inv() * b

Fzfl_cale = Fs[0]
Fzrl_cale = Fs[1]
Fzrr_cale = Fs[2]
Fzfr_cale = Fs[3]

# equations (9.51) to (9.54) in kiencke
Fzfl_kiencke = m*(lr/l*g - h/l*ax_ch)*(sympify("1/2") - h*ay_ch/(bf*g))
Fzfr_kiencke = m*(lr/l*g - h/l*ax_ch)*(sympify("1/2") + h*ay_ch/(bf*g))
Fzrl_kiencke = m*(lf/l*g + h/l*ax_ch)*(sympify("1/2") - h*ay_ch/(br*g))
Fzrr_kiencke = m*(lf/l*g + h/l*ax_ch)*(sympify("1/2") + h*ay_ch/(br*g))


diff_fl = (Fzfl_cale - Fzfl_kiencke).simplify()
diff_rl = (Fzrl_cale - Fzrl_kiencke).simplify()
diff_rr = (Fzrr_cale - Fzrr_kiencke).simplify()
diff_fr = (Fzfr_cale - Fzfr_kiencke).simplify()

print(diff_fl)
print(diff_rl)
print(diff_rr)
print(diff_fr)


