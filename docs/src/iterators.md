# Iteradores: KalmanIterator 

Un iterador debe contener la información de un paso del filtro de kalman. Puede contar con un estado interno (para hacer pruebas por ejemplo, ver `LinearKalmanIterator`), o podría contar con información obtenida a partir de mediciones de un sistema físico real. 

La función `next_iteration!(iterator::KalmanIterator, control)` permite generar una nueva iteración, pero requiere que se implementen los siguientes métodos:

Métodos a implementar | Breve descripción
--- | ---
`update_inner_state!(iterator, control) ` | Si el iterador guarda un estado interno ``x_n``, este método debería actualizarlo a ``x_{n+1}``. Si no (como cuando se está observando un sistema físico real del que solo se tienen mediciones), puede estar en blanco.
`forecast_observed_state!(iterator, control)` | Actualiza la estimación a priori de ``\hat{x}_{n+1,n}``, usando el control ``u_n``.
`observe_inner_system(iterator)` | Entrega una observación ``y_{n+1}`` del sistema real. 
`analyse!(iterator, observation)` | Analiza la observación ``y_{n+1}`` para generar una nueva estimación a posteriori ``\hat{x}_{n+1,n+1}``.
`update_updater!(iterator)` | Permite actualizar el updater luego de la iteración. Puede usarse por ejemplo, con un updater no lineal que debe ser linealizado tras cada iteración. Puede dejarse en blanco. 

Un ejemplo es 

```@docs
KalmanFilter.LinearKalmanIterator
```
