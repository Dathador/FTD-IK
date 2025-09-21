function Update(I)
    local v = Vector3(1, 0, 0)
    local u = Vector3(0, 1, 0)

    local w = Vector3.Cross(v, u)
    I:Log(string.format("v: (%.4f, %.4f, %.4f)", v.x, v.y, v.z))
end