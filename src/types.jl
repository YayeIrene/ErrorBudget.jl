mutable struct AimingError
    point_def::Tuple{Float64, Float64}
    human::Tuple{Float64, Float64}
    res::Tuple{Float64, Float64}
    opt_path::Tuple{Float64, Float64}
end

mutable struct AlignmentError
    boresight_acc::Tuple{Float64, Float64}
    boresight_ret::Tuple{Float64, Float64}
    sync::Tuple{Float64, Float64}
    stab::Tuple{Float64, Float64}
    gun_pos::Tuple{Float64, Float64}
end

mutable struct LofError
    fcs::Tuple{Float64, Float64}
    lead::Tuple{Float64, Float64}
    cant::Tuple{Float64, Float64}
    earth_rate::Tuple{Float64, Float64}
    muzzle_vel::Tuple{Float64, Float64}
    temp::Tuple{Float64, Float64}
    press::Tuple{Float64, Float64}
    cross_wind::Tuple{Float64, Float64}
    range_wind::Tuple{Float64, Float64}
    range::Tuple{Float64, Float64}
    jump::Tuple{Float64, Float64}
    zeroing::Tuple{Float64, Float64}
end

mutable struct FlightError
    aero::Tuple{Float64, Float64}
end
