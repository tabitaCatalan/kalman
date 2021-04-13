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
  """Control ``u_{n-1}`` usado para llegar al estado actual"""
  u::Real
  """
  Aproximación del estado interno ``\\hat{x}_{n,n}``, y una estimación de la matriz
  de covarianzas ``\\hat{P}_{n,n}`` del error ``x_n - \\hat{x}_{n,n}``.
  """
  hatX::ObservedState{T}
  """
  Estimación ``\\hat{x}_{n,n-1}`` del estado ``x_n`` antes de conocer la observación
  ``y_n`` (solo contando con información hasta ``y_{n-1}``).
  """
  next_hatX::ObservedState{T}
  """`KalmanUpdater` que permite actualizar tanto el estado interno como las aproximaciones."""
  updater::KalmanUpdater
  """
  Un observador lineal que entrega observaciones ``y_n`` del estado real.
  """
  observer::LinearObserver{T}
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
      observer::KalmanObserver, dt = 1.) where T <: Real
    n = 0
    #X = StochasticState(x0, 0.)
    hatX = ObservedState(x0, P0)
    next_hatX = ObservedState(x0, P0)
    noiser = Normal(0.,dt^2)
    new{T}(n, 1., hatX, next_hatX, updater, observer, noiser)
  end
end

function update_inner_state!(iterator::LinearKalmanIterator, control)
  noise = rand(iterator.noiser)
  update_real_state!(iterator.observer, iterator.updater, control, noise)
  iterator.u = control
end


function advance_counter!(iterator)
  iterator.n = iterator.n + 1
end

function analyse!(iterator::LinearKalmanIterator, observation)
  hatPₙ₊₁ₙ₊₁ = analyse_hatP(iterator)
  hatxₙ₊₁ₙ₊₁ = analyse_hatx(iterator, observation)
  iterator.hatX = ObservedState(hatxₙ₊₁ₙ₊₁, hatPₙ₊₁ₙ₊₁)
end

function observe_forecasted_system(iterator::LinearKalmanIterator)
  iterator.observer(next_hatx(iterator), un(iterator), 0.)
end

# Getters de matrices y fórmulas para actualizar y todo eso

Mn(iterator::KalmanIterator) = Mn(iterator.updater)
Bn(iterator::KalmanIterator) = Bn(iterator.updater)
Fn(iterator::KalmanIterator) = Fn(iterator.updater)
Hn(iterator::KalmanIterator) = Hn(iterator.observer)
Dn(Iterator::KalmanIterator) = Dn(Iterator.observer)
Gn(Iterator::KalmanIterator) = Gn(Iterator.observer)

Sn(iterator::KalmanIterator) = Fn(iterator) * Gn(iterator)'
Rn(iterator::KalmanIterator) = Gn(iterator) * Gn(iterator)'
Qn(iterator::KalmanIterator) = Fn(iterator) * Fn(iterator)'

#xn(iterator::LinearKalmanIterator) = iterator.X.x
un(iterator::LinearKalmanIterator) = iterator.u

hatx(iterator::LinearKalmanIterator) = iterator.hatX.hatx
hatP(iterator::LinearKalmanIterator) = iterator.hatX.hatP

next_hatP(iterator) = iterator.next_hatX.hatP
next_hatx(iterator) = iterator.next_hatX.hatx


En(iterator, hatPnp1n) = Rn(iterator) + Hn(iterator) * hatPnp1n * Hn(iterator)'
En(iterator) = En(iterator, next_hatP(iterator))

KalmanGain(iterator, hatPnp1n) = hatPnp1n * Hn(iterator)' * inv(En(iterator, hatPnp1n))
KalmanGain(iterator) = KalmanGain(iterator, next_hatP(iterator))

#function analysed_state(iterator, observation)
#  hatxn(iterator) + KalmanGain(iterator) * (observation - observe_observed_system(iterator))
#end


# Funciones más generales para observar y actualizar el sistema

# Observar
observe_inner_system(iterator::LinearKalmanIterator) = observe_real_state(iterator.observer, un(iterator), rand(iterator.noiser))

function observe_observed_system(iterator::LinearKalmanIterator)
  tempX = StochasticState(next_hatx(iterator), un(iterator))
  iterator.observer(tempX, 0.)
end
function update_updater!(iterator::LinearKalmanIterator)
  update!(iterator.updater, hatx(iterator), un(iterator))
end



function forecast(iterator::LinearKalmanIterator, control)
  hatPₙₙ = hatP(iterator)
  K = KalmanGain(iterator, hatPₙₙ)
  E = En(iterator, hatPₙₙ)
  hatPₙ₊₁ₙ = forecast_hatP(iterator, K, E, hatPₙₙ) # idem
  hatxₙ₊₁ₙ = forecast_hatx(iterator, control) # aun no debe haber cambiado el analizado
  hatxₙ₊₁ₙ, hatPₙ₊₁ₙ
end

function forecast_hatx(updater, hatx, control)
  update_aproximation(updater, hatx, control, 0.)
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

function forecast_observed_state!(iterator::LinearKalmanIterator, control)
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

  dimensions = length(get_inner_state(iterator.observer))

  results = InnerStateSeries(N, dimensions)

  for i in 1:N
    control = control_function(i * dt)
    #observation = KalmanFilter.observe_inner_system(iterator)

    add_state!(results, i, get_inner_state(iterator.observer))
    add_analysis!(results, i, hatx(iterator))


    forecastx, forecastP = forecast(iterator, control)
    add_forecast!(results, i, forecastx)
    add_error_analysis!(results, i, [hatP(iterator)[j,j] for j in 1:dimensions])
    add_error_forecast!(results, i, [forecastP[j,j] for j in 1:dimensions])

    add_observation!(results, i, next_iteration!(iterator, control)[1])

  end
  results
end
