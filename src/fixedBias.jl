function paralax(weapon::Gun,tank::Tank)
    h = gun.hg*(target.position-gun.zerotage)/(target.position*gun.zerotage)
    v = gun.vg*(target.position-gun.zerotage)/(target.position*gun.zerotage)
    return h,v
end

function jump()
end

function drift()
end

function ballistic()
end

function fixedBias()

end
