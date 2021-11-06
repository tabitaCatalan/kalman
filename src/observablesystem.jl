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

mutable struct Measurements <: ObservableSystem
    """Vector de mediciones de un sistema físico real"""
    measurements::Array{Float64,2}
    """Si es pedida una observación, se devolverá la `n`-ésima."""
    n::Int
    """Intervalo de *sampleo*."""
    dt
    function Measurements(measurements, dt)
      new(measurements, 1, dt)
    end
  end

number_of_measurements(ms::Measurements) = size(ms.measurements)[1]

# para evitar problemas de redondeo 
approxfloor(x) = floor(Int, x + eps())

"""
Determine measurement index to read from time `t`.
"""
get_index(ms::Measurements, t) = min(approxfloor(t / ms.dt) + 1, number_of_measurements(ms))

function update_real_state!(ms::Measurements, updater::KalmanUpdater, control, t)
    if 1 <= ms.n && ms.n <= number_of_measurements(ms)
        ms.n += 1
    else
        println("No hay más mediciones disponibles")
    end
end 
  
function observe_real_state(ms::Measurements, observer::KalmanObserver, control, error)
    ms.measurements[ms.n, :]
end

#= 
Inner State 
=#
struct InnerState <: ObservableSystem
    """Vector de estado"""
    x::Vector{Float64}
end
  
#InnerSystemTrait(::InnerState) = HastInnerSystem()

# creo que puedo hacer esto más general, definiendolo 
# para todos los de tipo HasInnerSystem
function update_real_state!(system::InnerState, updater::KalmanUpdater, control, t)
    new_state = update_inner_system(updater, get_inner_state(system), control, t)

    set_inner_state!(system, new_state)
end 
  
function observe_real_state(system::InnerState, observer::KalmanObserver, control, error)
    observer(system.x, control, error)
end

function get_inner_state(system::InnerState) system.x end
function set_inner_state!(system::InnerState, x) system.x .= x end




