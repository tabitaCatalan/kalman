using Random, Distributions


abstract type KalmanIterator end

"""
$(TYPEDSIGNATURES)

Permite actualizar un `KalmanIterator` usando una nueva evaluación de un control.
Requiere que hayan sido definidos los métodos de la interfaz.
"""
function next_iteration!(iterator::KalmanIterator, control)
  # advance real system
  update_inner_state!(iterator, control)

  forecast_observed_state!(iterator, control) #esto cambia next_hatX

  yₙ₊₁ = observe_inner_system(iterator)

  analyse!(iterator, yₙ₊₁)

  # prepare structures for next iteration 
  update_updater!(iterator)

  advance_counter!(iterator)

  yₙ₊₁
end
