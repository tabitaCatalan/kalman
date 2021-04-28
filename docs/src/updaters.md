# Actualizar el sistema: KalmanUpdater

El tipo abstracto `KalmanUpdater` está pensado como una interfaz a las estructuras que permitan actualizar el sistema desde un estado ``x_{n}`` a un estado ``x_{n+1}``.

Para que funciones correctamente deben implementarse los siguientes métodos:

Métodos a implementar | Breve descripción
--- | ---
`update!(L::KalmanUpdater, hatx, hatP, control)` | un método que permita actualizar al iterador y dejarlo listo para la siguiente iteración. Cuando se usan matrices ``M_n := M, B_n := B, F_n:= F`` contantes se puede dejar en blanco, pero debería usarse, por ejemplo, para linearlizar en torno a ``\hat{x}_n`` cuando se usa un `KalmanUpdater` no lineal.
`forecast(updater::KalmanUpdater, hatx, hatP, control)` | Devuelve una tupla que contiene a ``\hat{x}_{n+1, n}, \hat{P}_{n+1, n}`` a partir de ``\hat{x}_{n,n}``(`hatx`), ``\hat{P}_{n,n}``(`hatP`) y un control.


## `LinearizableUpdater`
Un grupo importante de `KalmanUpdater`s serán los que, o bien lineales de la forma

```math
x_{n+1} = M_n x_n + B_n u_n + F_n N_n
```
(donde ``M_n, F_n`` son matrices, ``x_n, N_n, B_n`` vectores y ``u_n`` es un control escalar), o bien, en caso de ser no lineales ``x_{n+1} = \mathcal{M}(x_n, u_n)``, pueden ser linealizados a la forma anterior.

Los miembros de esta clase necesitan implementar 

Métodos a implementar | Breve descripción
--- | ---
`Mn`, `Bn`, `Fn`| De la linearización en el estado actual

Cuentan además con las siguientes funciones

```@docs
KalmanFilter.dimensions
```

```@docs
KalmanFilter.noiser
```

### SimpleLinearUpdater

```@docs
SimpleLinearUpdater
```

### NLUpdater

```@docs
NLUpdater
```

Ambas estructuras pueden ser además evaluadas en la firma `(::KalmanUpdater)(x::AbstractArray, u::Real, error)`. Por ejemplo, para el caso no lineal, `udpater(x,u,ε)` devuelve ``\mathcal{M}(x,u) + Fε``.


## Otros `KalmanUpdaters` implementados 
Un actualizador importante es `ODEForecaster`, que ofrece un enfoque alternativo para trabajar con ecuaciones diferenciales estocásticas de la forma 

```math 
x(t) = F(x, t)dt + G(x, t)dW(t)
```

Este método se llama Filtro de Kalman Continuo Discreto, y es muy similar al Filtro de Kalman Extendido, pero el paso de *forecast* se hace resolviendo una ecuación diferencial acoplada, la Ecuación de los Momentos, para ``\hat{x}`` y ``\hat{P}``:

```math
\begin{aligned}
\hat{x}'(t) &= F(\hat{x},t) \\ 
\hat{P}'(t) &= \partial_\hat{x} F(\hat{x}, t) \hat{P} + \hat{P} \left( \partial_\hat{x} F(\hat{x}, t) \right) ^{\top} + G(t)Q(t)G(t)'
\end{aligned}
```

De esta forma, para obtener ``\hat{x}_{n+1,n}, \hat{P}_{n+1,n}`` basta con resolver la ecuación de los momentos en ``[t_{n}, t_{n+1}]``, con condiciones iniciales ``\hat{x}_{n+1,n}, \hat{P}_{n+1,n}``.

### ODEForecaster 

```@docs 
KalmanFilter.ODEForecaster
```

## Consideraciones adicionales 

### Función de integridad 

En general los constructores pedirán además una **función de integridad**; cuya idea es evitar que los estados se salgan de cierto dominio. Por ejemplo, para el caso en que trabajamos con EDOs epidemiológicas, es necesario que las cantidades sean no negativas, por lo que se usa la función de integridad `(x) -> max.(x, 0.)`, que transforma un vector `x` para dejar en 0 las coordenadas negativas.

### `update_inner_system` y `update_aproximation` 

Aún no está definida del todo la interfaz de `KalmanUpdater`, y varias estructuras implementan estos dos métodos.  

CREO que ya ni siquiera es necesaria la distinción, pues venía del hecho de que `KalmanIterator` manejaba los ruidos, cosa que ahora hace `KalmanUpdater`. En todo caso, ciertos actualizadores como `ODEForecaster` no tienen definido un método `update_inner_system` porque no están pensados para hacer eso (solo deberían usarse con `Measurements`, no con `InnerState`s). Es necesario implementar *holy traits* o algo así para diferenciar mejor los métodos que deben implementarse.