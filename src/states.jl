struct StochasticState{T}# < State
  x::AbstractVector{T}
  u::T
end

struct ObservedState{T} #< State
  hatx::AbstractVector{T}
  hatP::AbstractMatrix{T}
end
