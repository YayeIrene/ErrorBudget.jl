module ErrorBudget
using Cuas, ExternalBallistics

# Write your package code here.
include("types.jl")
include("paralax.jl")
include("totalError.jl")

export μ, AimingError, AlignmentError, LofError, FlightError, σ, UnitEffect

end
