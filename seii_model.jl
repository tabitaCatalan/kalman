# ## Modelo epidemiológico SEII
# El modelo permite modelar el desarrollo de una enfermedad regida por la siguiente
# dinámica:
# ```math
# \begin{aligned}
# S' &= - \alpha(t) S (E + I^m) \\
# E' &= \alpha(t) S (E + I^m) - \gamma_E E \\
# (I^m)' &= (1-\varphi) \gamma_E E - \gamma_I I^m \\
# I' &= \varphi \gamma_E E - \gamma_I I\\
# (cI)' &= \varphi \gamma_E E
# \end{aligned}
# ```
#
# donde
#
# - ``S`` representa a los **susceptibles**, las personas que podrían contagiarse.
# - ``E`` corresponde a los que han sido **expuestos** al virus y lo están incubando.
# - ``I^m`` denota a los infectados *mild*, que tienen síntomas leves o ninguno.
# - ``I`` denota a los infectados sintomáticos.
# - ``cI`` es el total de infectados sintomáticos acumulados (agregamos este valor porque es un dato con el que se cuenta del sistema real).
#
# Además la dinámica tiene los siguientes parámetros:
#
# - ``\alpha(t)`` es la Tasa de contagio en tiempo ``t``.
# - ``\gamma_E`` es Tasa de infección (paso de expuesto a infectado).
# - ``\gamma_I`` es la Tasa de remoción (dejar de estar infectado, ya sea por recuperación o muerte).
# - ``\varphi`` es Proporción de personas sintomáticas.
#
# Denotando ``\mathbf{x}(t) = (S(t), E, I^m, I, cI)``, el sistema es de la forma
# ```math
# \mathbf{x}(t)' = f(\mathbf{x}(t), \alpha(t), p)
# ```
# donde ``\alpha(t)`` será un control y ``p`` representa a los otros parámetros.
# Guardamos la información de la dinámica en la función `episystem_full(x, α, p)`.


function episystem_full(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α * β * x[1] * (x[2] + x[3]),
  α * β * x[1] * (x[2] + x[3]) - γₑ * x[2],
  (1 - φ) * γₑ * x[2] - γᵢ * x[3],
  φ * γₑ * x[2] - γᵢ * x[4],
  φ * γₑ * x[2]]
end


# Vemos que este sistema es no lineal, por lo que para poder trabajarlo con el filtro
# de Kalman necesitaremos linearizarlo. Para eso será necesaria la información
# ```math
# D_x f(\mathbf{x}, \alpha(t), p) =
# ```

function epijacobian_full_x(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α*β*(x[2] + x[3])  -α*β*x[1]     -α*β*x[1]  0.  0.;
  α*β*(x[2] + x[3])  (α*β*x[1]-γₑ)  α*β*x[1]  0.  0.;
  0.                 (1-φ)*γₑ       -γᵢ       0.  0.;
  0.                 φ*γₑ           0.        -γᵢ 0.;
  0.                 φ*γₑ           0.        0.  0.]
end

# Puesto que consideraremos el input como desconocido, y la dinámica ``f`` es no
# lineal en ese input, también necesitaremos ``D_u f``.


function epijacobian_full_u(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-β*x[1]*(x[2] + x[3]),
  β*x[1]*(x[2] + x[3]),
  0.,
  0.,
  0.]
end
