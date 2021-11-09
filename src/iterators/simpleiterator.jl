using ComponentArrays
################################################################################

"""
$(TYPEDEF)

Define un iterator de Kalman que almacena tanto un estado interno (que corresponde
al sistema real) como un sistema aproximado, el cual es ajustado a partir de
observaciones del sistema interno.

# Campos
$(FIELDS)
"""
mutable struct SimpleKalmanIterator{T<:AbstractFloat,
                                    OSy <: ObservableSystem, 
                                    CA1 <: ComponentArray, 
                                    CA2 <: ComponentArray, 
                                    U <: KalmanUpdater, 
                                    O <: KalmanObserver,  
                                    A <: AbstractVector{T}
                                    } <: KalmanIterator
  """Número de iteración actual ``n``. Se inicializa en 0."""
  n::Int
  """Control ``u_{n-1}`` usado para llegar al estado actual"""
  u::T
  """Sistema"""
  system::OSy
  """
  Aproximación del estado interno ``\\hat{x}_{n,n}``, y una estimación de la matriz
  de covarianzas ``\\hat{P}_{n,n}`` del error ``x_n - \\hat{x}_{n,n}``.
  """
  hatX::CA1
  """
  Estimación ``\\hat{x}_{n,n-1}`` del estado ``x_n`` antes de conocer la observación
  ``y_n`` (solo contando con información hasta ``y_{n-1}``).
  """
  next_hatX::CA1
  """`KalmanUpdater` que permite actualizar tanto el estado interno como las aproximaciones."""
  updater::U
  """
  Un observador lineal que entrega observaciones ``y_n`` del estado real.
  """
  observer::O
  """
  Vector de parámetros ``\\alpha \\in [0,1]`` (uno para cada estado) de un filtro paso bajo que permite eliminar las
  oscilaciones en el estado luego de hacer el análisis. Si ``\\alpha \\approx 1``, entonces 
  no se hace correción, si ``\\alpha \\approx 0``, se suprimen casi todas las oscilaciones.
  """
  alpha::A
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
  function SimpleKalmanIterator(x0::AbstractVector{T}, P0::AbstractMatrix{T},
      updater::KalmanUpdater,
      observer::KalmanObserver, system::ObservableSystem,
      dt::T, alpha::AbstractArray{T} = ones(size(x0))) where T <: AbstractFloat
    n = 0
    hatX = ComponentArray(x = copy(x0), P = copy(P0))
    next_hatX = ComponentArray(x = copy(x0), P = copy(P0))
    new{T, typeof(system), typeof(hatX), typeof(next_hatX), typeof(updater), typeof(observer), typeof(alpha)}(n, 1., system, hatX, next_hatX, updater, observer, alpha)
  end
end

function set_hatX!(iterator::SimpleKalmanIterator, x, P)
  iterator.hatX.x = x 
  iterator.hatX.P = P
end

function set_next_hatX!(iterator::SimpleKalmanIterator, x, P)
  iterator.next_hatX.x = x 
  iterator.next_hatX.P = P
end

isLinearizable(iterator::SimpleKalmanIterator) = isLinearizableUpdater(iterator.updater) && isLinearizableObserver(iterator.observer)

tn(iterator) = iterator.n * dt(iterator.updater)

function update_inner_state!(iterator::SimpleKalmanIterator, control)
  update_real_state!(iterator.system, iterator.updater, control, tn(iterator))
  iterator.u = control
end


function advance_counter!(iterator)
  iterator.n = iterator.n + 1
end

function analyse!(iterator::SimpleKalmanIterator, observation)
  hatPₙ₊₁ₙ₊₁ = analyse_hatP(iterator)
  hatxₙ₊₁ₙ₊₁ = analyse_hatx(iterator, observation)
  set_hatX!(iterator, lowpass(iterator, hatxₙ₊₁ₙ₊₁), hatPₙ₊₁ₙ₊₁)
end

function lowpass(xₙ, yₙ₋₁, α) # si α ≈ 0, entonces se suaviza mucho 
  α .* xₙ + (1 .- α) .* yₙ₋₁
end

function lowpass(iterator::SimpleKalmanIterator, hatx_new)
  lowpass(hatx_new, hatx(iterator), iterator.alpha)
end

function observe_forecasted_system(iterator::SimpleKalmanIterator)
  observe_without_error(iterator.observer, next_hatx(iterator), un(iterator))
end

# Getters de matrices y fórmulas para actualizar y todo eso

Mn(iterator::KalmanIterator) = Mn(iterator.updater)
#Bn(iterator::KalmanIterator) = Bn(iterator.updater)
#Fn(iterator::KalmanIterator) = Fn(iterator.updater)
Hn(iterator::KalmanIterator) = Hn(iterator.observer)
#Dn(Iterator::KalmanIterator) = Dn(Iterator.observer)
Gn(Iterator::KalmanIterator) = Gn(Iterator.observer)

#Sn(iterator::KalmanIterator) = Fn(iterator) * Gn(iterator)'
Rn(iterator::KalmanIterator) = Gn(iterator) * Gn(iterator)'
#Qn(iterator::KalmanIterator) = Fn(iterator) * Fn(iterator)'

#xn(iterator::SimpleKalmanIterator) = iterator.X.x
un(iterator::SimpleKalmanIterator) = iterator.u

hatx(iterator::SimpleKalmanIterator) = iterator.hatX.x
hatP(iterator::SimpleKalmanIterator) = iterator.hatX.P

next_hatx(iterator) = iterator.next_hatX.x
next_hatP(iterator) = iterator.next_hatX.P

En(iterator, hatPnp1n) = Rn(iterator) + Hn(iterator) * hatPnp1n * Hn(iterator)'
En(iterator) = En(iterator, next_hatP(iterator))

KalmanGain(iterator, hatPnp1n) = hatPnp1n * Hn(iterator)' * inv(En(iterator, hatPnp1n))
KalmanGain(iterator) = KalmanGain(iterator, next_hatP(iterator))

#function analysed_state(iterator, observation)
#  hatxn(iterator) + KalmanGain(iterator) * (observation - observe_observed_system(iterator))
#end


# Funciones más generales para observar y actualizar el sistema

# Observar
function observe_inner_system(iterator::SimpleKalmanIterator)
  observe_real_state(iterator.system, iterator.observer, un(iterator), tn(iterator))
end

function update_updater!(iterator::SimpleKalmanIterator)
  update!(iterator.updater, hatx(iterator), hatP(iterator), un(iterator), tn(iterator))
end

function forecast(iterator::SimpleKalmanIterator, control)
  forecast(iterator.updater, hatx(iterator), hatP(iterator), control, tn(iterator))
end

function forecast_observed_state!(iterator::SimpleKalmanIterator, control)
  # forecast P para calcular K y E
  Xₙ₊₁ₙ = forecast(iterator, control) # idem
  set_next_hatX!(iterator, Xₙ₊₁ₙ.x, Xₙ₊₁ₙ.P)
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
function full_iteration(iterator, dt, N, control_function, ensamble_size; obscheck = true)

  dimensions = kalman_size(iterator.observer)[1]

  results = InnerStateSeries(N, dimensions)
  ensamble = EnsamblesStoring(ensamble_size, dimensions, N)

  if obscheck && !isLinearizable(iterator)
    obscheck = false
    print("Can't check observabiblity. Observer or Updater not Linearizable.")
  end

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
    if obscheck
      print(Int(check_observability(iterator)))
    end 

  end
  results, ensamble
end

function check_observability(iterator::SimpleKalmanIterator)
  H = Hn(iterator)
  M = Mn(iterator)
  n = size(M)[1]
  obs_matrix = copy(H)
  for i = 1:(n-1)
          obs_matrix = [obs_matrix; M^i]
  end 
  rank(obs_matrix) == n
end