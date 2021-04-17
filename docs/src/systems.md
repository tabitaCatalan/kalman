# Sistema observables: ObservableSystem

El tipo abstracto `ObservableSystem` está pensado como una interfaz que permite observar el sistema real.

El sistema real puede ser, o bien un estado interno o bien entregar simplemente mediciones obtenidas a partir de un sistema físico externo. En el primer caso `ObservableSystem` se encarga de almacenar y actualizar el estado ``x_n``, y puede recibir un `KalmanObserver` para obtener observaciones ``y_n`` de él. En el segundo,  `ObservableSystem` solo cuenta con una lista de las mediciones ``y_n``, las cuales supondremos que proceden de un sistema físico descrito por un `KalmanUpdater` y que fueron observadas por un `KalmanUpdater` que no seremos capaces de conocer directamente.

La interfaz requiere los siguientes métodos 

Métodos a implementar | Breve descripción
---|---
`update_real_state!(system::ObservableSystem, updater::KalmanUpdater, control, error)` | Transforma el estado interno ``x_n`` en ``x_{n+1}``, por medio de un `KalmanUpdater`. Puede usarse para actualizar el número de iteración ``n`` a ``n+1``, para poder obtener, por ejemplo, una nueva medición la próxima vez que sea pedida. 
`observe_real_state(system::ObservableSystem, observer::KalmanObserver, control, error)` | Entrega una medición del estado real.

Los `ObservableSystem`s que almacenen un estado interno pueden definir además los siguientes métodos.

Métodos opcionales | Breve descripción
--- | ---
`get_inner_state(system::ObservableSystem)` | Devuelve un estado interno ``x_n``
`set_inner_state(system::ObservableSystem, x)` | Setea el estado interno ``x_n`` a ``x``.

## `InnerState`
Un `ObservableSystem` que almacena un estado interno es `InnerState`. 

```@docs 
KalmanFilter.InnerState
```

## `Measurements`

```@docs 
KalmanFilter.Measurements
```

