struct StochasticState{T, V <: AbstractVector{T}}# < State
  x::V
  u::T
end

struct ObservedState{T, V <: AbstractVector{T}, M<:AbstractArray{T, 2}} #< State
  hatx::V
  hatP::M
end
