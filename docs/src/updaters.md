# Actualizar el sistema: KalmanUpdater

El tipo abstracto `KalmanUpdater` está pensado como una interfaz a las estructuras que permitan actualizar el sistema desde un estado ``x_{n}`` a un estado ``x_{n+1}``. Se espera que sean, o bien lineales de la forma

```math
x_{n+1} = M_n x_n + B_n u_n + F_n N_n
```

o bien, en caso de ser no lineales ``x_{n+1} = \mathcal{M}(x_n, u_n)``, que puedan ser linealizados a la forma anterior.

Para que funciones correctamente deben implementarse los siguientes métodos:

Métodos a implementar | Breve descripción
--- | ---
`update!(L::KalmanUpdater, hatx, hatP, control)` | un método que permita actualizar al iterador y dejarlo listo para la siguiente iteración. Cuando se usan matrices ``M_n := M, B_n := B, F_n:= F`` contantes se puede dejar en blanco, pero debería usarse, por ejemplo, para linearlizar en torno a ``\hat{x}_n`` cuando se usa un `KalmanUpdater` no lineal.
`Mn`, `Bn`, `Fn`| De la linearización en el estado actual
`(::KalmanUpdater)(x::AbstractArray, u::Real, error)` | Debe poder evaluarse en esa firma. Por ejemplo, para el caso no lineal, `udpater(x,u,ε)` podría devolver ``\mathcal{M}(x,u) + Fε``.
`forecast(updater::KalmanUpdater, hatx, hatP, control)` | Devuelve una tupla que contiene a ``\hat{x}_{n+1, n}, \hat{P}_{n+1, n}`` a partir de ``\hat{x}_{n,n}``(`hatx`), ``\hat{P}_{n,n}``(`hatP`) y un control.

En general los constructores pedirán además una **función de integridad**; cuya idea es evitar que los estados se salgan de cierto dominio. Por ejemplo, para el caso en que trabajamos con EDOs epidemiológicas, es necesario que las cantidades sean no negativas, por lo que se usa la función de integridad `(x) -> max.(x, 0.)`, que transforma un vector `x` para dejar en 0 las coordenadas negativas.

## SimpleLinearUpdater

```@docs
SimpleLinearUpdater
```

## NLUpdater

```@docs
NLUpdater
```

