#=
Código que extiende a kalman.jl, para incorporar Updaters no lineales
=#

using LinearAlgebra


################################################################################
#= Discretizadores
El tipo abstracto Discretizer está pensado como una interfaz a las estructuras
que permitan trabajar con un sistema continuo (como una EDO) y transformarolo
en uno discreto. Esto se hace mediante métodos numéricos como Euler progresivo
y Runge Kutta de orden 4 (los ejemplos presentes).
=#
################################################################################


abstract type Discretizer end

function (::Discretizer) error("No se ha definido un método para este discretizador.") end
function jacobian_x(ds::Discretizer, x, α) error("No se ha definido un método para este discretizador.")  end

struct Euler <: Discretizer
  gamma
  dt
  sistema
  jacobiano
end

(eu::Euler)(x, α) = x + eu.dt * eu.sistema(x, α , eu.gamma)
jacobian_x(eu::Euler, x, α) = I + eu.dt * eu.jacobiano(x, α, eu.gamma)

struct RK4 <: Discretizer
  gamma
  dt
  system
  jacobian
end

function get_ks(rk::RK4, x, α)
  γ = rk.gamma

  x1 = x
  k1 = rk.system(x1, α, γ)

  x2 = x1 + dt * k1/2
  k2 = rk.system(x2, α, γ)

  x3 = x1 + dt * k2/2
  k3 = rk.system(x3, α, γ)

  x4 = x1 + dt * k3
  k4 = rk.system(x4, α, γ)
  (x1, x2, x3, x4), (k1, k2, k3, k4)
end

function (rk::RK4)(x,α)

  xs, ks = get_ks(rk, x, α)

  x + (rk.dt/6) * (ks[1] + 2 * ks[2] + 2 * ks[3] + ks[4])
end

function jacobian_x(rk::RK4, x, α)
  dt = rk.dt; γ = rk.gamma
  xs, ks = get_ks(rk, x, α)

  I + (dt/6) * (rk.jacobian(xs[1], α, γ) + 2 * rk.jacobian(xs[2], α, γ) + 2 * rk.jacobian(xs[3], α, γ)+ rk.jacobian(xs[4], α, γ))
end

#########################

mutable struct NLUpdater <: KalmanUpdater
  discretizer::Discretizer
  F
  linear::LinearUpdater
  function NLUpdater(discretizer, F, x0, α)
    linear = linearize_x(discretizer, F, x0, α)
    new(discretizer, F, linear)
  end
end


function update!(updater::NLUpdater, x, α) #NLupdater = NonLinearUpdater(fancyM, jacobian_x)
  linear = linearize_x(updater, x, α)
  updater.linear = linear
end

Mn(updater::NLUpdater) = Mn(updater.linear)
Bn(updater::NLUpdater) = Bn(updater.linear)
Fn(updater::NLUpdater) = Fn(updater.linear)



#function (updater::NLUpdater)(state, control, error)
#  updater.discretizer(state.x, control) + updater.F * error
#end


function (updater::NLUpdater)(x::AbstractArray, u, error)
  updater.discretizer(x, u) + updater.F * error
end
#### podría hacer algo que retorne un linear updater...

function linearize_x(NLup::NLUpdater, x, α)
  discretizer = NLup.discretizer
  linearize_x(discretizer, NLup.F, x, α)
end

function linearize_x(discretizer::Discretizer, F, x, α)
  M = jacobian_x(discretizer, x, α)
  B = discretizer(x, α) - M * x
  LinearUpdater(M, B, F)
end
