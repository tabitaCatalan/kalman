using KalmanFilter

## Caso autito moviéndose en línea recta

dt = 0.1
M = [1. dt; 0. 1.]
B = [dt^2/2, dt]
H = [1. 0.]

wn = 0.2
F = [dt*wn/2, wn]
G = [10.]
x0 = [0., 0.]

updater = KalmanFilter.SimpleLinearUpdater(M,B,F)
observer = KalmanFilter.LinearObserver(H,zeros(1),G)

iterator = KalmanFilter.LinearKalmanIterator(x0, F*F', updater, observer)

T = 60
N = Int(T/dt)



observations, real_states, analysis, forecast, errors_analysis, errors_forecast = KalmanFilter.full_iteration(iterator, N)


using Plots

ts = 0.0:dt:(T-dt)

plotstate(2, "Velocidad")
plotstate(1, "Posición")
plot!(ts, observations, label = "Observación")
