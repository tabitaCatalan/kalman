using Random, Distributions


abstract type KalmanIterator end

"""
$(TYPEDSIGNATURES)

Permite actualizar un `KalmanIterator` usando una nueva evaluación de un control.
Requiere que hayan sido definidos los métodos de la interfaz.
"""
function next_iteration!(iterator::KalmanIterator, control)
  # advance real system
  update_inner_state!(iterator, control)

  forecast_observed_state!(iterator, control) #esto cambia next_hatX

  # observe system
  yₙ₊₁ = observe_inner_system(iterator)

  # analysis
  analyse!(iterator, yₙ₊₁)

  update_updater!(iterator)

  yₙ₊₁
end

################################################################################

"""
$(TYPEDEF)

Define un iterator de Kalman que almacena tanto un estado interno (que corresponde
al sistema real) como un sistema aproximado, el cual es ajustado a partir de
observaciones del sistema interno.

# Campos
$(FIELDS)
"""
mutable struct LinearKalmanIterator{T} <: KalmanIterator
  """Número de iteración actual ``n``. Se inicializa en 0."""
  n::Int
  """Estado interno ``x_n`` y control ``u_{n-1} usado para obtenerla."""
  X::StochasticState{T}
  """
  Aproximación del estado interno ``\\hat{x}_{n,n}``, y una estimación de la matriz
  de covarianzas ``\\hat{P}_{n,n}`` del error ``x_n - \\hat{x}_{n,n}``.
  """
  hatX::ObservedState{T}
  """
  Estimación ``hat{x}_{n,n-1}`` del estado ``x_n`` antes de conocer la observación
  ``y_n`` (solo contando con información hasta ``y_{n-1}``).
  """
  next_hatX::ObservedState{T}
  """`KalmanUpdater` que permite actualizar tanto el estado interno como las aproximaciones."""
  updater::KalmanUpdater
  """Un observador lineal que entrega observaciones ```y_n`` del estado interno."""
  observer::LinearObserver{T}
  #Pn::AbstractMatrix{T}
  """Una distribución que permite agregar ruido al sistema. Por defecto es una ``\\mathcal{N}(0,1)``."""
  noiser::UnivariateDistribution

  """
  $(TYPEDSIGNATURES)
  Crea un iterador que almacena un estado interno.
  # Argumentos
  - `x0`: condición inicial, tanto para el sistema interno como para el observador.
  - `P0`: matriz de covarianza de la condición inicial.
  - `updater::KalmanUpdater`: actualizará tanto el sistema interno como el observado.
  - `observer::KalmanObserver`: permite observar el sistema interno.
  """
  function LinearKalmanIterator(x0::AbstractVector{T}, P0::AbstractMatrix{T},
      updater::KalmanUpdater,
      observer::KalmanObserver) where T <: Real
    n = 0
    X = StochasticState(x0, 0.)
    hatX = ObservedState(x0, P0)
    next_hatX = ObservedState(x0, P0)
    noiser = Normal()
    new{T}(n, X, hatX, next_hatX, updater, observer, noiser)
  end
end

function update_inner_state!(iterator::LinearKalmanIterator, control)
  xₙ₊₁ = next_state_system(iterator, control); uₙ = control
  iterator.X = Xₙ₊₁ = StochasticState(xₙ₊₁, uₙ)
end


function advance_counter!(iterator)
  iterator.n = iterator.n + 1
end

function analyse!(iterator, observation)
  hatPₙ₊₁ₙ₊₁ = analyse_hatP(iterator)
  hatxₙ₊₁ₙ₊₁ = analyse_hatx(iterator, observation)
  iterator.hatX = ObservedState(hatxₙ₊₁ₙ₊₁, hatPₙ₊₁ₙ₊₁)
end

function observe_forecasted_system(iterator)
  iterator.observer(next_hatx(iterator), un(iterator), 0.)
end

# Getters de matrices y fórmulas para actualizar y todo eso

Mn(iterator::LinearKalmanIterator) = Mn(iterator.updater)
Bn(iterator::LinearKalmanIterator) = Bn(iterator.updater)
Fn(iterator::LinearKalmanIterator) = Fn(iterator.updater)
Hn(iterator::LinearKalmanIterator) = Hn(iterator.observer)
Dn(Iterator::LinearKalmanIterator) = Dn(Iterator.observer)
Gn(Iterator::LinearKalmanIterator) = Gn(Iterator.observer)

Sn(iterator::LinearKalmanIterator) = Fn(iterator) * Gn(iterator)'
Rn(iterator::LinearKalmanIterator) = Gn(iterator) * Gn(iterator)'
Qn(iterator::LinearKalmanIterator) = Fn(iterator) * Fn(iterator)'

xn(iterator::LinearKalmanIterator) = iterator.X.x
un(iterator::LinearKalmanIterator) = iterator.X.u

hatx(iterator) = iterator.hatX.hatx
hatP(iterator) = iterator.hatX.hatP

next_hatP(iterator) = iterator.next_hatX.hatP
next_hatx(iterator) = iterator.next_hatX.hatx


En(iterator, hatPnp1n) = Rn(iterator) + Hn(iterator) * hatPnp1n * Hn(iterator)'
En(iterator) = En(iterator, next_hatP(iterator))

KalmanGain(iterator, hatPnp1n) = hatPnp1n * Hn(iterator)' * inv(En(iterator))
KalmanGain(iterator) = KalmanGain(iterator, next_hatP(iterator))


#function analysed_state(iterator, observation)
#  hatxn(iterator) + KalmanGain(iterator) * (observation - observe_observed_system(iterator))
#end


# Funciones más generales para observar y actualizar el sistema

# Observar
observe_inner_system(iterator) = iterator.observer(iterator.X, rand(iterator.noiser))

function observe_observed_system(iterator)
  tempX = StochasticState(next_hatx(iterator), un(iterator))
  iterator.observer(tempX, 0.)
end

# Actualizar
next_state_system(iterator, control) = iterator.updater(xn(iterator), control, rand(iterator.noiser))
#next_observed_system(iterator, control) = iterator.updater(hatx(iterator), control, 0.)

function update_state!(iterator, xnext, control)
  iterator.X = StochasticState(xnext, control)
end

function update_state_system!(iterator, control)
  next = next_state_system(iterator, control)
  update_state!(iterator, next, control)
end

#function update_observer_system!(iterator, updater, control)
#  next = next_observed_system(iterator, control)
#  update_state!(iterator, next, control)
#  # iterator.updater = updater (para matrices variables en el tiempo)
#end

function update_updater!(iterator)
  update!(iterator.updater, hatx(iterator), un(iterator))
end



function forecast(iterator, control)
  hatPₙₙ = hatP(iterator)
  K = KalmanGain(iterator, hatPₙₙ)
  E = En(iterator, hatPₙₙ)
  hatPₙ₊₁ₙ = forecast_hatP(iterator, K, E, hatPₙₙ) # idem
  hatxₙ₊₁ₙ = forecast_hatx(iterator, control) # aun no debe haber cambiado el analizado
  hatxₙ₊₁ₙ, hatPₙ₊₁ₙ
end

function forecast_hatx(updater, hatx, control)
  updater(hatx, control, 0.)
end

function forecast_hatx(iterator::LinearKalmanIterator, control)
  forecast_hatx(iterator.updater, hatx(iterator), control)
end

function forecast_hatP(iterator::LinearKalmanIterator, K, E, hatP)
  Mn(iterator) * hatP * Mn(iterator)' + Qn(iterator)
  #  - Sn(iterator) * inv(E) * Sn(iterator)'
  #  - Sn(iterator) * K' * Mn(iterator)'
  #  - Mn(iterator) * K * Sn(iterator)'
end

function forecast_hatP(iterator::LinearKalmanIterator)
  forecast_hatP(iterator::LinearKalmanIterator, KalmanGain(iterator), En(iterator), hatP(iterator))
end

function forecast_observed_state!(iterator, control)
  # forecast P para calcular K y E
  hatxₙ₊₁ₙ, hatPₙ₊₁ₙ = forecast(iterator, control) # idem
  iterator.next_hatX = ObservedState(hatxₙ₊₁ₙ, hatPₙ₊₁ₙ)
end


#= Analizar solucion incorporando observación=#

function analyse_hatx(iterator, observation)
  next_hatx(iterator) + KalmanGain(iterator) * (observation - observe_forecasted_system(iterator))
end

function analyse_hatP(iterator)
  K = KalmanGain(iterator)
  next_hatP(iterator) - K * En(iterator) * K'
end


"""
# Argumentos
- `control_function`: control en función del tiempo
"""
function full_iteration(iterator, dt, N, control_function)

  dimensions = length(xn(iterator))

  observations = Vector{Float64}(undef, N)
  real_states = Array{Float64, 2}(undef, N, dimensions)
  analysis = Array{Float64, 2}(undef, N, dimensions)
  forecast = Array{Float64, 2}(undef, N, dimensions)

  errors_analysis = Array{Float64, 2}(undef, N, dimensions)
  errors_forecast = Array{Float64, 2}(undef, N, dimensions)

  for i in 1:N
    control = control_function(i * dt)
    #observation = KalmanFilter.observe_inner_system(iterator)

    real_states[i,:] = KalmanFilter.xn(iterator)
    analysis[i,:] = KalmanFilter.hatx(iterator)

    forecastx, forecastP = KalmanFilter.forecast(iterator, control)
    forecast[i,:] = forecastx
    errors_analysis[i,:] = [KalmanFilter.hatP(iterator)[j,j] for j in 1:dimensions]
    errors_forecast[i,:] = [forecastP[j,j] for j in 1:dimensions]

    #iterator.X.x ≈ iterator.hatX.hatx ? print("!") :
    #predictions2[i,:] = KalmanFilter.analysed_state(iterator, observation)

    observations[i] = KalmanFilter.next_iteration!(iterator, control)[1]
    # Save states

  end
  observations, real_states, analysis, forecast, errors_analysis, errors_forecast
end
