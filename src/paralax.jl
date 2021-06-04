function paralax(gun::Gun,target::AbstractTarget)
    h = gun.hg*(target.position-gun.zerotage)/(target.position*gun.zerotage)
    v = gun.vg*(target.position-gun.zerotage)/(target.position*gun.zerotage)
    return h,v 
end
