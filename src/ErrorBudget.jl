module ErrorBudget
using Cuas, ExternalBallistics
using Distributions
using DataFrames
using Interpolations

# Write your package code here.
include("types.jl")
include("paralax.jl")
include("variableBias.jl")
include("totalError.jl")

export μ, AimingError, AlignmentError, LofError, FlightError, σ, UnitEffect, windCrossError,windRangeError,interTable,rangeError,
        elevationError, azimuthError, muzzleVelError, temperatureError, cantError

end
