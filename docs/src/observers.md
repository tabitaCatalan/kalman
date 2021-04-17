# Observadores del sistema real: Observers 

El tipo abstracto `KalmanObserver` está pensado como una interfaz a las estructuras
que permitan observar el sistema real en estado ``x_{n}``, entregando una observación
``y_{n}``. Se espera que sean, o bien lineales de la forma

```math
y_{n} = H_n x_n + D_n u_n + G_n N_n
```
o bien, en caso de ser no lineales ``y_{n} = \mathcal{H}(x_n, u_n)``, que puedan ser linealizados a la forma anterior. 

Lo que observer debería ofrecer 

- Observar un sistema, es decir, entregar una observación ``y_n`` a partir de un estado ``x_n``.
- Observer DEBE ser linearizable, eso implica que debe poder verse como ``H_n x_n + D_n u_n + G_n N_n``. Esto es necesario para calcular la matriz de ganancia. 
- Se debe poder usar para observar el sistema aproximado ``\hat{x}_n \to \hat{y}_n``. Esto significa que debe poder evaluarse `(ob::KalmanObserver)(x, u, error)`.

Se espera que tengan la siguiente interfaz (los métodos opcionales deberían implementarse dejándose en blanco):

Métodos a implementar | Breve descripción
---|---
`Hn`, `Dn`, `Gn` | las matrices/vectores ``H, D, G`` respectivamente. Por ahora serán constantes.
`(ob::KalmanObserver)(x::AbstractArray, u::Real, error)` | Este método permite evaluar el `KalmanObserver` en un estado `x`. Se usa para hacer una observación del sistema aproximado, evaluando en ``\hat{x_n}`` y el control ``u_n`` para devolver una observación ``\hat{y}_n``.

## LinearObserver 

Una estructura sencilla que implementa todos los métodos anteriores es `LinearObserver`.

```@docs 
KalmanFilter.LinearObserver
```


