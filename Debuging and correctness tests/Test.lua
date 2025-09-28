-- test scratch file

function Update(I)
    local v_x = Vector3(1, 0, 0)
    local v_y = Vector3(0, 1, 0)

    local v_z = Vector3.Cross(v_x, v_y)
    I:Log(v_z) -- should be (0, 0, 1)
end