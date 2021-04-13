# Filtered Series
using RecipesBase

abstract type FilteredSeries end

# En el futuro, para otros tipos creo que se puede hacer una versión parametrizada
# Para incluir otras dimensiones

"""Permite guardar los resultados de una iteración de filtrado, además de la
evolución de un estado interno."""
mutable struct InnerStateSeries <: FilteredSeries
    """
    Cantidad máxima ``N_t`` de iteraciones que pueden ser guardadas.
    """
    Nt::Int
    """Dimensión del estado interno"""
    dimensions::Int
    """
    Vector de observaciones. `observations[i]` corresponde a la observación
    en la iteración `i`-ésima.
    """
    observations::Vector{Float64}
    inner_states::Array{Float64,2}
    analysis::Array{Float64, 2}
    forecast::Array{Float64, 2}
    error_analysis::Array{Float64, 2}
    error_forecast::Array{Float64, 2}
    control::Vector{Float64}
    function InnerStateSeries(N, dimensions)
        observations = Vector{Float64}(undef, N)
        real_states = Array{Float64, 2}(undef, N, dimensions)
        analysis = Array{Float64, 2}(undef, N, dimensions)
        forecast = Array{Float64, 2}(undef, N, dimensions)

        errors_analysis = Array{Float64, 2}(undef, N, dimensions)
        errors_forecast = Array{Float64, 2}(undef, N, dimensions)

        forecast[1] = NaN
        errors_forecast[1] = NaN
        control = Vector{Float64}(undef, N)
        new(N, dimensions, observations, real_states, analysis, forecast, errors_analysis, errors_forecast, control)
    end
end

function add_to_vector!(vector, index, value)
    if 1 <= index && index <= length(vector)
        vector[index] = value
    end
end
function add_to_array!(array, index, vector_values)
    if 1 <= index && index <= size(array)[1]
        array[index,:] .= vector_values
    end
end

function add_observation!(serie::InnerStateSeries, iteration, observation)
    add_to_vector!(serie.observations, iteration, observation)
end

function add_state!(serie::InnerStateSeries, iteration, state)
    add_to_array!(serie.inner_states, iteration, state)
end

function add_analysis!(serie::InnerStateSeries, iteration, analysed_state)
    add_to_array!(serie.analysis, iteration, analysed_state)
end

function add_forecast!(serie::InnerStateSeries, iteration, forecasted_state)
    add_to_array!(serie.forecast, iteration + 1, forecasted_state)
end

function add_error_analysis!(serie::InnerStateSeries, iteration, forecasted_state)
    add_to_array!(serie.error_analysis, iteration, forecasted_state)
end

function add_error_forecast!(serie::InnerStateSeries, iteration, forecasted_state)
    add_to_array!(serie.error_forecast, iteration + 1, forecasted_state)
end


@recipe function f(r::KalmanFilter.InnerStateSeries, ts, index, rango = 1:length(ts))
    i = index
    titles = ["Susceptibles", "Expuestos", "Infectados mild", "Infectados", "Infectados acumulados", "Control"]
    title --> titles[i]
    xguide --> "Tiempos t (días)"
    yguide --> "Personas"
    if i ≠ 6
        @series begin
            seriestype := :path
            label --> "Real"
            linewidth --> 2.
            ts[rango], r.inner_states[rango,i]
        end
    end
    @series begin
        seriestype := :path
        #primary := false
        linecolor:= nothing
        label --> "Análisis, con error 2σ"
        ribbon --> 2 * sqrt.(r.error_analysis[:,i])
        ts[rango], r.analysis[rango,i]
    end
    @series begin
        seriestype := :path
        a = :seriescolor
        label --> "Análisis, con error 1σ"
        ribbon --> sqrt.(r.error_analysis[:,i])
        ts[rango], r.analysis[rango,i]
    end
    #=@series begin
        seriestype := :path
        label --> "Forecast"
        ts[rango], r.forecast[rango,i]
    end=#
    if i == 5
        @series begin
            seriestype := :path
            label --> "Observaciones"
            ts[rango], r.observations[rango]
        end
    end

    #label --> "Análisis"
    #ts, r.analysis[:,index]
end
