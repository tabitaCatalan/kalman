abstract type ObservableSystem end 

#=
# Holy Traits 
abstract type InnerSystemTrait end
struct IsMeasurements <: InnerSystemTrait end
struct HastInnerSystem <: InnerSystemTrait end


hasinnersystem(x::T) where {T} = hasinnersystem(InnerSystemTrait(T), x)
hasinnersystem(::IsMeasurements, x) = false
hasinnersystem(::HastInnerSystem, x) = true

# default
InnerSystemTrait(::ObservableSystem) = IsMeasurements()

function observe_real_state(::ObservableSystem) error("observe_real_state no ha sido implementado") end

function get_inner_state(::IsMeasurements) error("A system of this type has no inner state") end
function set_inner_state(::IsMeasurements, x) error("A system of this type has no inner state") end
=#

#=
Measurements 
=#

struct Measurements{T, A <: AbstractArray{T, 2}} <: ObservableSystem
    """Array de mediciones de un sistema físico real.
    Supone que la primera medición es tomada en tiempo ``t = 0``.
    Las columnas corresponden a distintos estados/cantidades
    y las filas a distintos instantes de tiempo."""
    measurements::A
    """Intervalo de *sampleo*."""
    dt::T
end

number_of_measurements(ms::Measurements) = size(ms.measurements)[1]

# para evitar problemas de redondeo
approxfloor(x) = floor(Int, x + eps())

"""
Determine measurement index to read from time `t`.
"""
get_index(ms::Measurements, t) = min(approxfloor(t / ms.dt) + 1, number_of_measurements(ms))

function update_real_state!(ms::Measurements, updater::KalmanUpdater, control, t) end 
  
function observe_real_state(ms::Measurements, t)
    ms.measurements[get_index(ms, t), :]
end

function observe_real_state(ms::Measurements, observer::KalmanObserver, control, t)
    ms.measurements[get_index(ms, t), :]
end

#= 
Inner State 
=#
struct InnerState{T, A <: AbstractVector{T}} <: ObservableSystem
    """Vector de estado"""
    x::A
end
  
#InnerSystemTrait(::InnerState) = HastInnerSystem()

# creo que puedo hacer esto más general, definiendolo 
# para todos los de tipo HasInnerSystem
function update_real_state!(system::InnerState, updater::KalmanUpdater, control, t)
    new_state = update_inner_system(updater, get_inner_state(system), control, t)

    set_inner_state!(system, new_state)
end 
  
function observe_real_state(system::InnerState, observer::KalmanObserver, control, t)
    observe_with_error(observer, system.x, control)
end

function get_inner_state(system::InnerState) system.x end
function set_inner_state!(system::InnerState, x) system.x .= x end




