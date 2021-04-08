# Observadores del sistema real: Observers 

El tipo abstracto `KalmanObserver` está pensado como una interfaz a las estructuras
que permitan observar el sistema en estado ``x_{n}``, entregando una observación
``y_{n}``. Se espera que sean, o bien lineales de la forma

```math
y_{n} = H_n x_n + D_n u_n + G_n N_n
```
o bien, en caso de ser no lineales ``y_{n} = \mathcal{H}(x_n, u_n)``, que puedan ser linealizados a la forma anterior. 

Los tipos que implementen `KalmanObserver` pueden almacenar un estado interno, o bien pueden simplemente entregar mediciones obtenidas a partir de un sistema físico real.

Lo que observer debería ofrecer 
- [Opcional] La posibilidad de guardar un sistema interno ``x_n`` y actualizarlo por medio de un updater (``x_n \to x_{n+1}``), usando un control ``u_n`` y un `Updater`. 
- Observar el sistema principal. Esto debería ser una caja negra, la idea es que el iterator no tenga por qué saber si observer tiene un sistema interno o si entrega mediciones reales.
- Observer DEBE ser linearizable, eso implica que debe poder verse como ``H_n x_n + D_n u_n + G_n N_n``. Esto es necesario para calcular la matriz de ganancia. 
- Se debe poder usar para observar el sistema aproximado ``\hat{x}_n \to \hat{y}_n``. Esto significa que debe poder evaluarse `(ob::KalmanObserver)(x, u, error)`.

Se espera que tengan la siguiente interfaz (los métodos opcionales deberían implementarse dejándose en blanco):

Métodos a implementar | Breve descripción
---|---
`Hn`, `Dn`, `Gn` | las matrices/vectores ``H, D, G`` respectivamente. Por ahora serán constantes.
`(ob::KalmanObserver)(x::AbstractArray, u::Real, error)` | Este método permite evaluar el `KalmanObserver` en un estado `x`. Se usa para hacer una observación del sistema aproximado, evaluando en ``\hat{x_n}`` y el control ``u_n`` para devolver una observación ``\hat{y}_n``.
**Métodos opcionales** | **Breve descripción**
`inner_state(observer)` | Devuelve un estado interno ``x_n``
`update_inner_state!(observer, updater, control, error)` | Transforma el estado interno ``x_n`` en ``x_{n+1}``, por medio de un `KalmanUpdater`. Puede usarse para actualizar el número de iteración ``n`` a ``n+1``, para poder obtener, por ejemplo, una nueva medición la próxima vez que sea pedida.