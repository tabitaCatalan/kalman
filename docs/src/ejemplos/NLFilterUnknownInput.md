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
F = 100. * ones(5)
```

Solo consideraremos conocida la cantidad total de infectados ``\mathbf{x}_5 = cI``.

```@example NLFilterUnknownInput
H = [0. 0. 0. 0. 1.]
```

Usaremos un error de unas 50.000 personas en las mediciones.

```@example NLFilterUnknownInput
G = [500.]
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
tildex0 = [x0; 0.4]

dt = 0.05
T = 30.
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
        1.
    elseif t >= 5. && t < 12.
        0.8
    elseif t >= 12. && t < 25.
        0.5
    elseif t >= 25. && t < 40.
        1.5
    elseif t >= 40.
        1.
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
nlupdater = NLUpdater(rk,F,x0,0.4)
nlaugmented = KalmanFilter.NLUpdaterUnknowInput(nlupdater, control_pieces, 1.)
observer = KalmanFilter.LinearObserver(tildeH, zeros(1), G)
system = KalmanFilter.InnerState(tildex0)
iterator = KalmanFilter.LinearKalmanIterator(tildex0, tildeP, nlaugmented, observer, system, dt)
```

Y realizamos un total de `N` iteraciones, guardando los estamos intermedios
en las variables que aparecen abajo.

```@example NLFilterUnknownInput
results, ensamble = KalmanFilter.full_iteration(iterator, dt, N, t -> 0., 1)
```

## Resultados
Graficaremos los estados internos considerados, y los resultados obtenidos.

Ahora podemos graficar los diferentes estados de nuestro sistema, así como las
aproximaciones obtenidas con filtro de Kalman.

Susceptibles ``S``

```@example NLFilterUnknownInput
plot(results, ts, 1)
```

Expuestos ``E``

```@example NLFilterUnknownInput
plot(results, ts, 2)
```

Infectados asintomáticos *mild* ``I^m``

```@example NLFilterUnknownInput
plot(results, ts, 3)
```

Infectados ``I``

```@example NLFilterUnknownInput
plot(results, ts, 4)
```

Infectados acumulados ``cI``, y las observaciones que hicimos de él.

```@example NLFilterUnknownInput
plot(results, ts, 5)
```

Finalmente, veamos el control usado y el aproximado

```@example NLFilterUnknownInput
plot(ts, control_pieces.(ts), label = "Control real")
plot!(results, ts, 6)
```

Notamos que tras una cierta cantidad de tiempo es posible averiguar el control
Aunque hay bastante incerteza de los resultados.

---

*This page was generated using [Literate.jl](https://github.com/fredrikekre/Literate.jl).*

