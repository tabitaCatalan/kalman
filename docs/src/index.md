```@meta
CurrentModule = KalmanFilter
DocTestSetup = quote
    using KalmanFilter
end
```

# KalmanFilter: Un paquete que hice por mi tesis y que nadie usará además de mí

Estoy probando a crear documentación.

## Filtro de Kalman 

Filtro de Kalman es un método poderoso para estimar un estado a partir de observaciones de este. 

Supongamos que contamos con un sistema 
```math 
x_{n+1} = M_n x_n + B_n u_n + F_n N_n
```
Donde ``N_n`` son variables aleatorias normales, ``N_n \sim \mathcal{N}(0,1)``. Suponemos además que este sistema es desconocido, y que solo podemos obtener observaciones ``y_n`` de él.
```math 
y_n = H_n x_n + D_n u_n + G_n N_n 
```
El filtro de Kalman es un método para construir una estimación filtrada ``\hat{x}_n`` del estado real ``x_n``, a partir de las observaciones ``y_n`` y suponiendo que el control ``u_n`` es conocido. 

Una iteración del filtro de Kalman se puede dividir en dos pasos: 
1. Un paso de _**forecast**_, donde hacemos un suposición *a priori* de dónde estará el estado real en la próxima iteración a partir de la dinámica y nuestra estimación actual.
2. Una vez que recibimos la observación proveniente del estado real, hacemos un paso de **análisis** donde incorporamos esa información para obtener una estimación *a posteriori*.

Hablando en términos más matemáticos, nuestro objetivo es obtener, en cada paso ``n = 1, 2, \dots``, la distribución **filtrada** del estado; buscamos la distribución *a posteriori* de ``x_n`` dadas las observaciones ``y_1, \dots, y_n`` hasta la iteración ``n``.


AGREGAR FORMULAS DE KALMAN. 

## Filtro de Kalman por ensambles 

Ver [Katzfussa], la descripción está tomada de ahí.

El filtro de Kalman por ensambles (**EnKF** por su nombre en inglés *Ensemble Kalman Filter*), es una versión aproximada del filtro de Kalman, donde la distrubución del estado se representa con una muestra o "ensamble" de esa distribución . 

Más específicamente, suponemos que ``\hat{x}^{(1)}_{n,n}, \dots, \hat{x}^{(N)}_{n,n}`` es una muestra de la distribución filtrada en tiempo ``n``, es decir, que ``\hat{x}^{(i)}_{n,n} \sim \mathcal{N}(\hat{x}_{n,n}, \hat{P}_{n,n})``. 

Al igual que con el filtro de Kalman, una iteración de EnKF también se hará en un paso de *forecast* y un paso de análisis. 

Durante el *forecast*, nuestra distribución a priori será 
```math
\hat{x}^{(i)}_{n+1,n} = f_n(\hat{x}^{(i)}_{n,n}, u_n, w^{(i)}_n)
```
con ``w^{(i)}_n \sim \mathcal{N}(0,Q_n)``. Para el caso lineal en que ``f_n(x, u, w) = M_n x + B_n u + w`` se puede verificar que ``\hat{x}^{(i)}_{n+1,n} \sim \mathcal{N}(\mu, \Sigma)`` (COMPLETAR).


## Recursos adicionales 
- [KalmanFilter.NET](https://www.kalmanfilter.net/default.aspx) contiene explicaciones y tutoriales.

Un libro más matemático es 
*Optimal Filtering* de Brian Anderson y John Moore.

- Katzfussa, Stroudb, Wiklec, "Understanding the Ensemble Kalman Filter"


