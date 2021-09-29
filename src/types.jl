"""
    AimingError(point_def,human,res,opt_path)

defines an objet 'AimingError' with fields containing the aiming error sources:
* point_def: error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).
* human: human error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).
* res: resolution error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).
* opt_path: optical path bending error (standard deviatin) on the aimpoint definition expressed as a tupple (horizontal and vertical).
"""
mutable struct AimingError
    point_def::Tuple{Float64, Float64}
    human::Tuple{Float64, Float64}
    res::Tuple{Float64, Float64}
    opt_path::Tuple{Float64, Float64}
end

"""
    AlignmentError(boresight_acc,boresight_ret,sync,stab, gun_pos)

defines an objet 'AlignmentError' with fields containing the alignment error sources:
* boresight_acc: boresight accuracy (standard deviatin) expressed as a tupple (horizontal and vertical).
* boresight_ret: boresight retention (standard deviatin) expressed as a tupple (horizontal and vertical).
* sync: synchronism/parallelism error (standard deviatin) expressed as a tupple (horizontal and vertical).
* stab: stabilisation error (standard deviation) expressed as a tupple (horizontal and vertical).
* gun_pos: gun position measurement error (standard deviation) expressed as a tupple (horizontal and vertical).
"""
mutable struct AlignmentError
    boresight_acc::Tuple{Float64, Float64}
    boresight_ret::Tuple{Float64, Float64}
    sync::Tuple{Float64, Float64}
    stab::Tuple{Float64, Float64}
    gun_pos::Tuple{Float64, Float64}
end

"""
    LofError(fcs,lead,cant,earth_rate, muzzle_vel, temp, press, cross_wind, range_wind, range, jump, zeroing)

defines an objet 'LofError' with fields containing the line of fire error sources:
* fcs: fire control error (standard deviation) expressed as a tupple (horizontal and vertical).
* lead: lead angle (standard deviation) expressed as a tupple (horizontal and vertical).
* cant: cant angle error (standard deviation) expressed as a tupple (horizontal and vertical).
* earth_rate: earth rate error (standard deviation) expressed as a tupple (horizontal and vertical).
* muzzle_vel: muzzle velocity error (standard deviation) expressed as a tupple (horizontal and vertical).
* temp: temperature error (standard deviation) expressed as a tupple (horizontal and vertical).
* press: pressure error (standard deviation) expressed as a tupple (horizontal and vertical).
* cross_wind: cross wind error (standard deviation) expressed as a tupple (horizontal and vertical).
* range_wind: range wind error (standard deviation) expressed as a tupple (horizontal and vertical).
* jump: fleet jump error (standard deviation) expressed as a tupple (horizontal and vertical).
* zeroing: calibration error (standard deviation) expressed as a tupple (horizontal and vertical).
"""
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

"""
    FlightError(aero)

defines an objet 'FlightError' which contains the projectile dispersion as
an error (standard deviation) expressed as a tupple (horizontal and vertical).
"""
mutable struct FlightError
    aero::Tuple{Float64, Float64}
end

"""
    UnitEffect(velocity,temp,density, windR,windC)

Returns the UnitEffects for different ranges. The parameters considered are:
    * velocity
    * temp: temperature
    * density
    * windR: wind range
    * windC: cross wind
"""
mutable struct UnitEffect
    velocity::Vector{Float64}
    temp::Vector{Float64}
    density::Vector{Float64}
    windR::Vector{Float64}
    windC::Vector{Float64}
end

"""
    Error(horizontal, vertical)

Returns the Error object with field: 'horizontal' and 'vertical'
"""
mutable struct Error
    horizontal::Float64
    vertical::Float64
end
