# Discretizadores: `Discretizer`

El tipo abstracto `Discretizer` está pensado como una interfaz a las estructuras
que permitan trabajar con una Ecuación Diferencial Ordinaria autónoma (un sistema continuo)
de la forma 
```math
x'(t) = f_p(x(t), \alpha(t), t)
```
donde ``\alpha`` es una función de ``t`` conocida, y transformarlo en uno discreto
```math
x_{n+1} = \mathcal{M}_p(x_n, \alpha_n, t_n)
```
El subíndice ``p`` denota la posible dependencia de  ``f`` de algunos parámetros.
Esto se hace mediante métodos numéricos como Euler, Runge Kutta, etc.

Para que funcione correctamente deben implementarse los siguientes métodos:

Métodos a implementar | Breve descripción
--- | ---
`(::Discretizer)(x::AbstractArray, α::Real, t) ` | El método debe poder ser evaluado en un estado ``x``, un control ``\alpha`` y en tiempo ``t``.
`jacobian_x(ds::Discretizer, x, α, t)` | Debe contarse con la derivada con respecto al estado.
`dt(ds::Discretizer)` | ``\Delta t`` para la discretización temporal.
Usaremos Euler progresivo como un ejemplo de cómo implementar la interfaz. 


## Ejemplo de implementación: Método Euler progresivo 

Dada una condición inicial ``x(0) = x_0``, y un paso de tiempo ``\Delta t``, definimos ``t_{n+1} = t_{n} + \Delta t``.
Denotamos ``\alpha_n := \alpha(t_n)``.
Definimos la sucesión ``\{x_n \}_n``, intentando que ``x_n \approx x(t_{n+1})``, mediante la recursión
```math
x_{n+1} = x_n + \Delta t f_p(x_n, \alpha_n, t_n) =: \mathcal{M}_p(x_n, \alpha_n, t_n)
```
Para definir el discretizador, usaremos además la información del diferencial con respecto a ``x`` de ``f_p``, ya que
esto es útil más adelante a la hora de definir un filtro de kalman extendido.

```@docs
KalmanFilter.Euler
```

Esto permite definir los dos métodos necesarios de la interfaz 
```julia
(eu::Euler)(x, α, t) = x + eu.dt * eu.f(x, α , eu.p, t)
```
```julia
jacobian_x(eu::Euler, x, α, t) = I + eu.dt * eu.Dxf(x, α, eu.p, t)
```
Donde `I` es la matriz identidad dada por el paquete `LinearAlgebra`.

## `Discretizer`s disponibles 

`Discretizer` | Descripción 
---|--- 
`Euler` | Método de Euler progresivo, que permite ser diferenciado c/r a la variable ``x``.
`RK4Dx` | Método de Runge Kutta de orden 4, que permite ser diferenciado c/r a la variable ``x``. 

Agregar constructores de todos los discretizers disponibles. <!-- TODO -->