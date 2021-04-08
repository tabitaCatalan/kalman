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
El filtro de Kalman es un método para construir una estimación ``\hat{x}_n`` del sistema ``x_n`` usando las observaciones ``y_n`` y suponiendo que el control ``u_n`` es conocido. 

AGREGAR FORMULAS DE KALMAN. 

## Recursos adicionales 
- [KalmanFilter.NET](https://www.kalmanfilter.net/default.aspx) contiene explicaciones y tutoriales.

Un libro más matemático es 
*Optimal Filtering* de Brian Anderson y John Moore.




