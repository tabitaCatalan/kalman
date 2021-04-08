#=
Código que extiende a kalman.jl, para incorporar Updaters no lineales
=#


#########################

"""
$(TYPEDEF)
Define un actualizador dado por la discretización de una ODE.

Define un actualizador posiblemente no lineal del sistema, a partir de la
discretización de una Ecuación Diferencial Ordinaria. Esto permite calcular el
estado ``x_{n+1}`` a partir del estado ``x_{n}`` y el control ``u_n`` según
```math
x_{n+1} = \\mathcal{M}(x_n, u_n) + F N_n
```
donde ``N_n`` es un número aleatorio (dado por una variable aleatorio normal
``\\mathcal{N}(0,1)``) y ``\\mathcal{M}`` está dada por un `Discretizer`.
"""
mutable struct NLUpdater <: KalmanUpdater
  discretizer::Discretizer
  F
  linear::SimpleLinearUpdater
  """
  $(TYPEDSIGNATURES)
  Constructor de un actualizador no lineal `NLUpdater`.
  # Argumentos
  -
  """
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


function (updater::NLUpdater)(x::AbstractArray, u::Real, error)
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
  SimpleLinearUpdater(M, B, F)
end
