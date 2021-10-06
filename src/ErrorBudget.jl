module ErrorBudget
using ExternalBallistics
using WeaponSystems
using Distributions
using DataFrames
using Interpolations

# Write your package code here.
include("types.jl")
include("transformations.jl")
include("paralax.jl")
include("variableBias.jl")
include("totalError.jl")
include("generate.jl")


export μ, AimingError, AlignmentError, LofError, FlightError, σ, UnitEffect, windCrossError,windRangeError,interTable,rangeError,
        elevationError, azimuthError, muzzleVelError, temperatureError, cantError, Error, variableBias, TargetRect, TargetCirc,
        SpheError, createTargetRect, createTargetCirc, targetPos, muzzlePos, muzzleVel, wind

end
