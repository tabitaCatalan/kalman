# Momentums Continuo-Discreto 

Para el Filtro de Kalman Continuo-Discreto, es necesario resolver la ecuación de los momentos. Esto solo puede hacerse de forma aproximada, y distintos métodos de aproximación dan lugar a distintos filtros. Los momentos implementados fueron el `ExtendedMomentum`, que da lugar al Filtro de Kalman Extendido (EKF), y el `UnscentedMomentum`, que da lugar al Filtro de Kalman "sin olor" (UKF). 

```@docs
KalmanFilter.ContinuousDiscretMomentum
```
## Unscented Momentum 

```@docs
KalmanFilter.UnscentedMomentum
```

```@docs
KalmanFilter.(UM::UnscentedMomentum)
```

```@docs
KalmanFilter.unscented_transform_sqrtP
```

## Extended Momentum 

```@docs
KalmanFilter.ExtendedMomentum
```
