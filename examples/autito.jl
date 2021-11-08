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



T = 60
N = Int(T/dt)

iterator = KalmanFilter.SimpleKalmanIterator(x0, F*F', updater, observer)
observations, real_states, analysis, forecast, errors_analysis, errors_forecast = KalmanFilter.full_iteration(iterator, dt, N, t -> 1.)


using Plots

ts = 0.0:dt:(T-dt)


real_states
plotstate(2, "Velocidad u=1.0", ts)
plotstate(1, "Posición u=1.0", ts)
plot!(ts, observations, label = "Observación")

plot_error(2, "Velocidad")
plot_error(1, "Posición")
