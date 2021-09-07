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

  yₙ₊₁ = observe_inner_system(iterator)

  analyse!(iterator, yₙ₊₁)

  # prepare structures for next iteration 
  update_updater!(iterator)

  advance_counter!(iterator)

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
  """Sistema"""
  system::ObservableSystem
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
  observer::KalmanObserver
  """Una distribución que permite agregar ruido al sistema. Por defecto es una ``\\mathcal{N}(0,1)``."""
  noiser::UnivariateDistribution
  """
  Parámetro ``\\alpha \\in [0,1]`` de un filtro paso bajo que permite eliminar las
  oscilaciones en el estado luego de hacer el análisis. Si ``\\alpha \\approx 1``, entonces 
  no se hace correción, si ``\\alpha \\approx 0``, se suprimen casi todas las oscilaciones.
  """
  alpha 
  """
  $(TYPEDSIGNATURES)
  Crea un iterador que almacena un estado interno.
  # Argumentos
  - `x0`: condición inicial, tanto para el sistema interno como para el observador.
  - `P0`: matriz de covarianza de la condición inicial.
  - `updater::KalmanUpdater`: actualizará tanto el sistema interno como el observado.
  - `observer::KalmanObserver`: permite observar el sistema interno.
  - `dt`:
  - `alpha`: vector de parámetros (de la misma dimensión que el estado)con ``\\alpha_i \\in [0,1]``
    de un filtro paso bajo, permite eliminar oscilaciones en el estado. Mientras más
    cercano a ``1``, menor el efecto. Opcional, por defecto es un vector de 1s.
  """
  function LinearKalmanIterator(x0::AbstractVector{T}, P0::AbstractMatrix{T},
      updater::KalmanUpdater,
      observer::KalmanObserver, system::ObservableSystem,
      dt, alpha = ones(size(x0))) where T <: Real
    n = 0
    # verificar que α es escalar o que tiene el mismo tamaño que x0 
    #X = StochasticState(x0, 0.)
    hatX = ObservedState(copy(x0), copy(P0))
    next_hatX = ObservedState(copy(x0), copy(P0))
    noiser = Normal(0.,sqrt(dt))
    new{T}(n, 1., system, hatX, next_hatX, updater, observer, noiser, alpha)
  end
end

tn(iterator) = iterator.n * dt(iterator.updater)

function update_inner_state!(iterator::LinearKalmanIterator, control)
  update_real_state!(iterator.system, iterator.updater, control)
  iterator.u = control
end


function advance_counter!(iterator)
  iterator.n = iterator.n + 1
end

function analyse!(iterator::LinearKalmanIterator, observation)
  hatPₙ₊₁ₙ₊₁ = analyse_hatP(iterator)
  hatxₙ₊₁ₙ₊₁ = analyse_hatx(iterator, observation)
  iterator.hatX = ObservedState(lowpass(iterator, hatxₙ₊₁ₙ₊₁), hatPₙ₊₁ₙ₊₁)
end

function lowpass(xₙ, yₙ₋₁, α) # si α ≈ 0, entonces se suaviza mucho 
  α .* xₙ + (1 .- α) .* yₙ₋₁
end

function lowpass(iterator::LinearKalmanIterator, hatx_new)
  lowpass(hatx_new, hatx(iterator), iterator.alpha)
end

function observe_forecasted_system(iterator::LinearKalmanIterator)
  iterator.observer(next_hatx(iterator), un(iterator), 0.)
end

# Getters de matrices y fórmulas para actualizar y todo eso

#Mn(iterator::KalmanIterator) = Mn(iterator.updater)
#Bn(iterator::KalmanIterator) = Bn(iterator.updater)
#Fn(iterator::KalmanIterator) = Fn(iterator.updater)
Hn(iterator::KalmanIterator) = Hn(iterator.observer)
#Dn(Iterator::KalmanIterator) = Dn(Iterator.observer)
Gn(Iterator::KalmanIterator) = Gn(Iterator.observer)

#Sn(iterator::KalmanIterator) = Fn(iterator) * Gn(iterator)'
Rn(iterator::KalmanIterator) = Gn(iterator) * Gn(iterator)'
#Qn(iterator::KalmanIterator) = Fn(iterator) * Fn(iterator)'

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
function observe_inner_system(iterator::LinearKalmanIterator)
  noise = rand(iterator.noiser)
  observe_real_state(iterator.system, iterator.observer, un(iterator), noise)
end

function observe_observed_system(iterator::LinearKalmanIterator)
  tempX = StochasticState(next_hatx(iterator), un(iterator))
  iterator.observer(tempX, 0.)
end

function update_updater!(iterator::LinearKalmanIterator)
  update!(iterator.updater, hatx(iterator), hatP(iterator), un(iterator), tn(iterator))
end

function forecast(iterator::LinearKalmanIterator, control)
  forecast(iterator.updater, hatx(iterator), hatP(iterator), control, tn(iterator))
end

function forecast_observed_state!(iterator::LinearKalmanIterator, control)
  # forecast P para calcular K y E
  Xₙ₊₁ₙ = forecast(iterator, control) # idem
  iterator.next_hatX = ObservedState(Xₙ₊₁ₙ.x, Xₙ₊₁ₙ.P)
end


#= Analizar solucion incorporando observación=#

function analyse_hatx(iterator, observation)
  next_hatx(iterator) + KalmanGain(iterator) * (observation - observe_forecasted_system(iterator))
end

function analyse_hatP(iterator)
  K = KalmanGain(iterator)
  (I - K * Hn(iterator)) * next_hatP(iterator) * (I - K * Hn(iterator))' + K * Rn(iterator) * K'
  #next_hatP(iterator) - K * En(iterator) * K'

end


"""
# Argumentos
- `control_function`: control en función del tiempo
"""
function full_iteration(iterator, dt, N, control_function, ensamble_size)

  dimensions = kalman_size(iterator.observer)[1]

  results = InnerStateSeries(N, dimensions)
  ensamble = EnsamblesStoring(ensamble_size, dimensions, N)

  for i in 1:N
    #print(i)
    control = control_function(i * dt)
    #observation = KalmanFilter.observe_inner_system(iterator)
    add_ensamble!(ensamble, i, iterator)
    #add_state!(results, i, get_inner_state(iterator.system))
    add_analysis!(results, i, hatx(iterator))


    #forecastx, forecastP = forecast(iterator, control)
    #add_forecast!(results, i, forecastx)
    add_error_analysis!(results, i, [hatP(iterator)[j,j] for j in 1:dimensions])
    #add_error_forecast!(results, i, [forecastP[j,j] for j in 1:dimensions])

    add_observation!(results, i, next_iteration!(iterator, control)[1])
    print(Int(check_observability(iterator)))

  end
  results, ensamble
end

function check_observability(iterator::LinearKalmanIterator)
  H = KalmanFilter.Hn(iterator.observer)
  M = KalmanFilter.Mn(iterator.updater)
  n = size(M)[1]
  obs_matrix = copy(H)
  for i = 1:(n-1)
          obs_matrix = [obs_matrix; M^i]
  end 
  rank(obs_matrix) == n
end