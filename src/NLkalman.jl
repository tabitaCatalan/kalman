#=
Código que extiende a kalman.jl, para incorporar Updaters no lineales
=#

using Random
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
``\\mathcal{N}(0,Q)``) y ``\\mathcal{M}`` está dada por un `Discretizer`.

En todo tiempo guarda una versión linealizada del sistema, en torno a un punto 
``x_n, u_n, t_n`` (al construirlo se le deben dar esos valores también.)
"""
mutable struct NLUpdater <: LinearizableUpdater
  discretizer::Discretizer
  F
  linear::SimpleLinearUpdater
  integrity
  """
  $(TYPEDSIGNATURES)
  Constructor de un actualizador no lineal `NLUpdater`.
  # Argumentos
  - `discretizer::Discretizer`: incorpora la información de la función no lineal 
  a usar, así como del método de discretización. Debe tener definido `jacobian_x`.
  - `F::Function`: función de la forma `F(x)` que devuelva una matriz cuadrada de 
  las dimensiones de `x0`, correspondiente a la matriz de dispersión de ruido.
  - `Q`: matriz de covarianzas del ruido (se supone Gaussiano). Debe tener las 
  dimensiones de `x0` y ser semidefinida positiva.
  - `x0`, `α0`, `t0`: condiciones iniciales en torno a las que se va a linealizar.
  - `integrity`: función que transforma un vector `x` para que cumple ciertas 
    restricciones de integridad (ser positivo, etc).
  """
  function NLUpdater(discretizer::Discretizer, F, Q, x0, α0, t0, integrity)
    linear = linearize_x(discretizer, x0, α0, t0, F, Q, integrity)
    new(discretizer, F, linear, integrity)
  end
end


function update!(updater::NLUpdater, x, P, α, t) #NLupdater = NonLinearUpdater(fancyM, jacobian_x)
  linear = linearize_x(updater, x, α, t)
  updater.linear = linear
end

Mn(updater::NLUpdater) = Mn(updater.linear)
Bn(updater::NLUpdater) = Bn(updater.linear)
Fn(updater::NLUpdater) = Fn(updater.linear)
Qn(updater::NLUpdater) = Qn(updater.linear)

dt(updater::NLUpdater) = dt(updater.discretizer)

#function (updater::NLUpdater)(state, control, error)
#  updater.discretizer(state.x, control) + updater.F * error
#end


function (updater::NLUpdater)(x::AbstractArray, u::Real, t, noise)
  updater.integrity(updater.discretizer(x, u, t) + updater.F(x) * noise)
end

update_inner_system(updater::NLUpdater, x::AbstractArray, u::Real, t) = updater(x, u, t, rand(noiser(updater)))
update_aproximation(updater::NLUpdater, x::AbstractArray, u::Real, t) = updater(x, u, t, zeros(dimensions(updater)))

#### podría hacer algo que retorne un linear updater...

function linearize_x(NLup::NLUpdater, x, α, t)
  discretizer = NLup.discretizer
  linearize_x(discretizer, x, α, t, NLup.F, Qn(NLup), NLup.integrity)
end

"""
$(TYPEDEF)
# Argumentos 
- `discretizer::Discretizer`: el que discretiza la ecuación de actualización de estado.
- `x`: Vector en torno al que se va a linealizar 
- `α`: Valor del control en torno al que se va a linealizar 
- `F`: función que recibe `x` y retorna una matriz de dispersión del ruido. 
- `Q`: matriz de covarianza del ruido 
- `integrity`: función de `x`, que conserva el valor dentro de un dominio.
"""
function linearize_x(discretizer::Discretizer, x, α, t, F, Q, integrity)
  M = jacobian_x(discretizer, x, α, t)
  B = discretizer(x, α, t) - M * x 
  SimpleLinearUpdater(M, B, F(x), Q, integrity)
end
