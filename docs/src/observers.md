# Observadores del sistema real: Observers 

El tipo abstracto `KalmanObserver` está pensado como una interfaz a las estructuras
que permitan observar el sistema real en estado ``x_{n}``, entregando una observación
``y_{n}``. 

Se espera que un `KalmanObserver` implemente la siguiente interfaz:

Métodos a implementar | Breve descripción
---|---
`(ob::KalmanObserver)(x::AbstractArray, u::Real, error)` | Este método permite evaluar el `KalmanObserver` en un estado `x`. Se usa para hacer una observación del sistema aproximado con `error`, evaluando en ``\hat{x_n}`` y el control ``u_n`` para devolver una observación ``\hat{y}_n``.
`noiser(observer::KalmanObserver)` | Devuelve una Distribución (usualmente gaussiana centrada en ``0``). Para que funcione `observe_with_error` se requiere que la dimensión de `rand(noiser(observer))` sea la misma que la de las observaciones.


Métodos disponibles | Breve descripción 
--- |--- 
`observe_with_error(observer::KalmanObserver, x, u)` | Observa el estado `x` con error en la medición dado por `rand(noiser(observer))`.
`isLinearizableObserver(::KalmanUpdater)` | `false` por defecto. 

## La clase de `LinearObserver`s

Una clase importarte (y de hecho, todos los `KalmanObserver`s implementados por defecto pertenecen a ella) es `LinearizableObserver`.

```@docs 
KalmanFilter.LinearizableObserver
```

Agrupa a los observadores lineales de la forma

```math
y_{n} = H_n x_n + D_n u_n + G_n v_n
```
o bien, en caso de ser no lineales ``y_{n} = \mathcal{H}(x_n, u_n, v_n)``, a los que puedan ser linealizados a la forma anterior. ``v_n \sim \mathcal{N}(0, R_n)`` es un vector de ruido gaussiano.

Se espera que tengan la siguiente interfaz (los métodos opcionales deberían implementarse dejándose en blanco):

Métodos a implementar | Breve descripción
---|---
`Hn`, `Dn`, `Gn`, `Rn` | las matrices/vectores ``H, D, G, R`` respectivamente. 
`(ob::KalmanObserver)(x::AbstractArray, u::Real, error)` | Este método permite evaluar el `KalmanObserver` en un estado `x`. Se usa para hacer una observación del sistema aproximado, evaluando en ``\hat{x_n}`` y el control ``u_n`` para devolver una observación ``\hat{y}_n``.


Métodos disponibles | Breve descripción 
--- |--- 
`isLinearizableObserver(::KalmanUpdater)` | `true` 
`noiser(observer::LinearizableObserver)` | Devuelve una `Distribution` normal multivariada centrada en ``0`` y con matriz de covarianzas `Rn(observer)`.
`observe_with_error(observer::KalmanObserver, x, u)` | Observar el estado `x` con error en la medición dado por `noiser(observer)`.
`state_dimension(obs::LinearizableObserver)` | De cuántas dimensiones es el estado ``x`` que el `obs`ervador puede observar.
`observation_dimension(obs::LinearizableObserver)` | Dimensión de las observaciones ``y``.
`(obs::LinearizableObserver)(x::AbstractArray, u::Real, error)` | `Hn(obs) * x + Dn(obs) * u + Gn(obs) * error`

Existe también la clase `LinearObserver` pero no define ni requiere métodos nuevos 

```@docs 
KalmanFilter.LinearObserver
```

### Un observador simple: `SimpleLinearObserver`
Una estructura sencilla que implementa todos los métodos anteriores es `SimpleLinearObserver`. Consiste en un observador para el caso lineal reducido en el que todas las matrices son constantes:

```math
y_{n} = H x_n + D u_n + G v_n
```

```@docs 
KalmanFilter.SimpleLinearObserver
```

### Un observador no lineal `NLObserver`

```@docs 
KalmanFilter.NLObserver
```

El constructor de `NLObserver` pedirá además una **función de integridad**; la idea es evitar que las observaciones entreguen resultados sin sentido físico, transformándolas para dejarlas dentro de cierto dominio. Por ejemplo, si trabajamos con EDOs epidemiológicas, y nuestras observaciones corresponden al número de infectados, no tiene sentido que este sea un valor negativo, por lo que podría usarse la función de integridad `(y) -> max.(y, 0.)`, que transforma un vector de observaciones `y`, dejando en 0 las coordenadas negativas.