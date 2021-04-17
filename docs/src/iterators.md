# Iteradores: KalmanIterator 

Un iterador debe contener la información de un paso del filtro de kalman. 

La función `next_iteration!(iterator::KalmanIterator, control)` permite generar una nueva iteración, pero requiere que se implementen los siguientes métodos:

Métodos a implementar | Breve descripción
--- | ---
`update_inner_state!(iterator, control) ` | Si el iterador guarda un estado interno ``x_n``, este método debería actualizarlo a ``x_{n+1}``. Si no (como cuando se está observando un sistema físico real del que solo se tienen mediciones), puede estar en blanco.
`forecast_observed_state!(iterator, control)` | Actualiza la estimación a priori de ``\hat{x}_{n+1,n}``, usando el control ``u_n``.
`observe_inner_system(iterator)` | Entrega una observación ``y_{n+1}`` del sistema real. 
`analyse!(iterator, observation)` | Analiza la observación ``y_{n+1}`` para generar una nueva estimación a posteriori ``\hat{x}_{n+1,n+1}``.
`update_updater!(iterator)` | Permite actualizar el updater luego de la iteración. Puede usarse por ejemplo, con un updater no lineal que debe ser linealizado tras cada iteración. Puede dejarse en blanco. 

Hay dos opciones disponibles 

## LinearKalmanIterator 

Permite realizar iterar con el filtro de Kalman de toda la vida.

```@docs
KalmanFilter.LinearKalmanIterator
```

## EnKF 

Permite iterar un ensamble de estados aproximados, los que representan una muestra de la distribución de los estados en el tiempo indicado. Esto permite generar aproximaciones con el promedio y la covarianza muestral de los datos.

```@docs
KalmanFilter.EnKF
```

!!! danger "Atención"
    Hasta ahora es necesario que el usuario cuide que las condiciones iniciales estén correctamente inicializadas tanto en `KalmanObserver` (para el caso en que se guarda un sistema interno) y para `KalmanUpdater` (importa en la linearización inicial de un sistema no lineal).