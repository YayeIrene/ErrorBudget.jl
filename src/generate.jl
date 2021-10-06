
function createTarget(size::Float64, position::Float64)
    Target1D(size,position)
end

"""
    createTargetRect(a,b, position)
Return a rectangular target by specifying the length 'a', the height 'b' and the 'position' (range)
"""
function createTargetRect(a::Float64,b::Float64, position::Float64)
    TargetRect(a,b,position)
end

"""
    createTargetCirc(size, position)
Returns a circular target by specifying the radius (size) and the position (range)
"""
function createTargetCirc(size::Float64, position::Float64)
    TargetCirc(size,position)
end