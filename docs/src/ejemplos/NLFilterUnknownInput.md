```@meta
EditURL = "<unknown>/../modelo_seii_input_desconocido.jl"
```

# Filtro de Kalman Extendido con input desconocido

```@example NLFilterUnknownInput
using KalmanFilter
using Plots
```

## Modelo epidemiológico SEII
El modelo permite modelar el desarrollo de una enfermedad regida por la siguiente
dinámica:
```math
\begin{aligned}
S' &= - \alpha(t) S (E + I^m) \\
E' &= \alpha(t) S (E + I^m) - \gamma_E E \\
(I^m)' &= (1-\varphi) \gamma_E E - \gamma_I I^m \\
I' &= \varphi \gamma_E E - \gamma_I I\\
(cI)' &= \varphi \gamma_E E
\end{aligned}
```

donde

- ``S`` representa a los **susceptibles**, las personas que podrían contagiarse.
- ``E`` corresponde a los que han sido **expuestos** al virus y lo están incubando.
- ``I^m`` denota a los infectados *mild*, que tienen síntomas leves o ninguno.
- ``I`` denota a los infectados sintomáticos.
- ``cI`` es el total de infectados sintomáticos acumulados (agregamos este valor porque es un dato con el que se cuenta del sistema real).

Además la dinámica tiene los siguientes parámetros:

- ``\alpha(t)`` es la Tasa de contagio en tiempo ``t``.
- ``\gamma_E`` es Tasa de infección (paso de expuesto a infectado).
- ``\gamma_I`` es la Tasa de remoción (dejar de estar infectado, ya sea por recuperación o muerte).
- ``\varphi`` es Proporción de personas sintomáticas.

Denotando ``\mathbf{x}(t) = (S(t), E, I^m, I, cI)``, el sistema es de la forma
```math
\mathbf{x}(t)' = f(\mathbf{x}(t), \alpha(t), p)
```
donde ``\alpha(t)`` será un control y ``p`` representa a los otros parámetros.
Guardamos la información de la dinámica en la función `episystem_full(x, α, p)`.

```@example NLFilterUnknownInput
function episystem_full(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α * β * x[1] * (x[2] + x[3]),
  α * β * x[1] * (x[2] + x[3]) - γₑ * x[2],
  (1 - φ) * γₑ * x[2] - γᵢ * x[3],
  φ * γₑ * x[2] - γᵢ * x[4],
  φ * γₑ * x[2]]
end
```

Vemos que este sistema es no lineal, por lo que para poder trabajarlo con el filtro
de Kalman necesitaremos linearizarlo. Para eso será necesaria la información
```math
D_x f(\mathbf{x}, \alpha(t), p) =
```

```@example NLFilterUnknownInput
function epijacobian_full_x(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-α*β*(x[2] + x[3])  -α*β*x[1]     -α*β*x[1]  0.  0.;
  α*β*(x[2] + x[3])  (α*β*x[1]-γₑ)  α*β*x[1]  0.  0.;
  0.                 (1-φ)*γₑ       -γᵢ       0.  0.;
  0.                 φ*γₑ           0.        -γᵢ 0.;
  0.                 φ*γₑ           0.        0.  0.]
end
```

Puesto que consideraremos el input como desconocido, y la dinámica ``f`` es no
lineal en ese input, también necesitaremos ``D_u f``.

```@example NLFilterUnknownInput
function epijacobian_full_u(x, α, p)
  γᵢ = p.gammai; γₑ = p.gammae; φ = p.phi;
  β = 1e-7
  [-β*x[1]*(x[2] + x[3]),
  β*x[1]*(x[2] + x[3]),
  0.,
  0.,
  0.]
end
```

Definimos un vector de condiciones iniciales ``x_0``.

```@example NLFilterUnknownInput
x0 = [7.112808e6, 1046.8508799517147, 0.0, 521.963080420307, 0.0]
F = 1000. * ones(5)
```

Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.

```@example NLFilterUnknownInput
H = [0. 0. 0. 0. 1.]
```

Usaremos un error de unas 50.000 personas en las mediciones.

```@example NLFilterUnknownInput
G = [50000.]
dimensions = 5
```

Definimos las matrices ``\tilde{H}`` para nuestro sistema auxiliar, que incluirá al input.

```@example NLFilterUnknownInput
tildeH = [H 0.]

tildeP = [F * F' zeros(5); zeros(5)' 1.]
```

EL nuevo vector de estados será ``\tilde{\mathbf{x}} = \begin{pmatrix} \mathbf{x}_0 \\ 1. \end{pmatrix}``,
donde estamos considerando una suposición inicial para el estado $\alpha = 1$.

```@example NLFilterUnknownInput
tildex0 = [x0; 1.]

dt = 0.2
T = 100.
N = Int(T/dt)
```

Definimos los parámetros

```@example NLFilterUnknownInput
γₑ = 0.14
γᵢ = 1/14
φ = 0.3
```

Que agrupamos en una tupla `p`.

```@example NLFilterUnknownInput
p = (gammae = γₑ, gammai = γᵢ, phi = φ)
```

Usaremos un `Discretizer` RungeKutta de orden 4, diferenciable tanto en el estado
``x`` como en el control ``u``. Lo construimos a partir de un discretizador solo
diferenciable en ``x`` por comodidad.

```@example NLFilterUnknownInput
rkx = KalmanFilter.RK4Dx(episystem_full, epijacobian_full_x, p, dt)
rk = KalmanFilter.RK4Du(rkx, epijacobian_full_u)
```

El control para el sistema interno (que supondremos desconocido y es el que intentaremos
averiguar) será la función constante por pedazos `control_pieces`.

```@example NLFilterUnknownInput
function control_pieces(t)
    if t < 5.
        0.5
    elseif t >= 5. && t < 10.
        0.5
    elseif t >= 10. && t < 15.
        1.1
    elseif t >= 15. && t < 20.
        1.5
    elseif t >= 20.
        2.5
    end
end
```

Veamos un gráfico del control usado, para lo que definimos un vector con los tiempos `ts`.

```@example NLFilterUnknownInput
ts = 0.0:dt:(T-dt)
plot(ts, control_pieces.(ts), label = "Control real")
```

Definimos las estructuras necesarias para crear un `LinearKalmanIterator`.

```@example NLFilterUnknownInput
nlupdater = NLUpdater(rk,F,x0,1.)
nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces)
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G, tildex0)
iterator = KalmanFilter.LinearKalmanIterator(tildex0, tildeP, nlaugmented, observer)
```

Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
en las variables que aparecen abajo.

```@example NLFilterUnknownInput
observations, real_states, analysis, forecast, errors_analysis, errors_forecast = KalmanFilter.full_iteration(iterator, dt, N, t -> 0.)
```

## Resultados
Graficaremos los estados internos considerados, y los resultados obtenidos.

```@example NLFilterUnknownInput
rango = 1:floor(Int,length(ts))
```

Definimos unas funciones horribles para acortar el proceso

```@example NLFilterUnknownInput
function plotstate!(a_plot, state_index, ts, rango = 1:length(ts); labels = ["Real", "Kalman analysed", "Kalman forecast"])
  i = state_index
  dimensions = size(real_states)[2]
  nans = NaN * ones(dimensions)'
  errors_forecast_correction = [nans; errors_forecast[1:end-1,:]]
  forecast_correction = [nans; forecast[1:end-1,:]]
  if i ≠ 6
    plot!(a_plot, ts[rango], real_states[rango,i], label = labels[1])
  end
  #plot!(a_plot, ts[rango], analysis[rango,i], label = labels[2])

  plot!(a_plot, ts[rango], analysis[rango,i], label = labels[2], ribbon = 2 * sqrt.(abs.(errors_analysis[rango,i])))
  plot!(a_plot, ts[rango], analysis[rango,i], label = labels[2], ribbon = sqrt.(abs.(errors_analysis[rango,i])))
  #plot!(a_plot, ts[rango], forecast_correction[rango,i], label = labels[3], ribbon = sqrt.(abs.(errors_forecast_correction[rango,i])))
  a_plot
end

function plot_error(state_index, state_name)
  i = state_index
  a_plot = plot(title = "Error " * state_name)
  plot!(a_plot, ts[1:end-1], sqrt.(errors_analysis[2:end,i]), label = "Analysis")
  plot!(a_plot, ts, sqrt.(errors_forecast[:,i]), label = "Forecast")
end
```

Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
aproximaciones obtenidas con filtro de Kalman.

Susceptibles ``S``

```@example NLFilterUnknownInput
a_plot = plot(title = "Susceptibles")
plotstate!(a_plot, 1, ts)
```

Expuestos ``E``

```@example NLFilterUnknownInput
a_plot = plot(title = "Expuestos")
plotstate!(a_plot, 2, ts)
```

Infectados asintomáticos *mild* ``I^m``

```@example NLFilterUnknownInput
a_plot = plot(title = "Mild")
plotstate!(a_plot, 3, ts)
```

Infectados ``I``

```@example NLFilterUnknownInput
a_plot = plot(title = "Infectado")
plotstate!(a_plot, 4, ts)
```

Infectados acumulados ``cI``, y las observaciones que hicimos de él.

```@example NLFilterUnknownInput
a_plot = plot(title = "Acumulados")
plotstate!(a_plot, 5, ts)
plot!(ts[rango], observations[rango], label = "Observaciones", legend =:bottomright)
```

Finalmente, veamos el control usado y el aproximado

```@example NLFilterUnknownInput
a_plot = plot(title = "Control")
plot!(ts, control_pieces.(ts), label = "Control real")
plotstate!(a_plot, 6, ts)
```

Notamos que tras una cierta cantidad de tiempo es posible averiguarlo con bastante certeza.

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

