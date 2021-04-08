# Observadores del sistema real: Observers 

El tipo abstracto `KalmanObserver` está pensado como una interfaz a las estructuras
que permitan observar el sistema en estado ``x_{n}``, entregando una observación
``y_{n}``. Se espera que sean, o bien lineales de la forma

```math
y_{n} = H_n x_n + D_n u_n + G_n N_n
```
o bien, en caso de ser no lineales ``y_{n} = \mathcal{H}(x_n, u_n)``, que puedan ser linealizados a la forma anterior.
Se espera que tengan la siguiente interfaz:

Métodos a implementar | Breve descripción
---|---
`update!(L::KalmanUpdater, hatx, control)`| un método que permita actualizar al iterador y dejarlo listo para la siguiente iteración. Cuando se usan matrices ``H_n := M, D_n := D, G_n:= G`` contantes se puede dejar en blanco, pero debería usarse, por ejemplo, para linearlizar en torno a ``\hat{x}_n`` cuando se usa un `KalmanObserver` no lineal.
`Hn`, `Dn`, `Gn` | de la linearización en el estado actual
`(x::AbstractArray, u::Real, error)` | ... Esto tengo que decidirlo, creo que sería más conveniente que el observer tenga un método para actualizar el sistema interno, de tener uno. Que reciba un updater que pueda evaluar el sistema o algo así. Y eso resolvería el problema de insertar datos que provienen de mediciones.