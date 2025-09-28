-- test file to measure the rotation of a single joint in a FTD construct

-- matrix class
local Matrix = {}
Matrix.__typename = "Matrix"

function Matrix:new(data)
    local obj = {}
    obj.rows = #data
    obj.cols = #data[1] or 1
    
    -- Deep copy the data to prevent reference sharing
    for i = 1, obj.rows do
        obj[i] = {}
        for j = 1, obj.cols do
            obj[i][j] = data[i][j]
        end
    end
    
    setmetatable(obj, self)
    self.__index = self
    return obj
end


-- Convert quaternion to rotation matrix
local function quaternion_to_rotation_matrix(q)
    local w, x, y, z = q.w, q.x, q.y, q.z
    -- normalize the quaternion
    local norm = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    return Matrix:new({
        {1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y},
        {2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x},
        {2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y}
    })
end

function Matrix:tostring()
    local str = ""
    for i = 1, self.rows do
        str = str .. "| "
        for j = 1, self.cols do
            str = str .. string.format("%.4f ", self[i][j])
        end
        str = str .. "|\n"
    end
    return str
end

local function get_joint_list(joint_names, I) -- returns a list of subconstruct IDs for the given joint names
    local joints = {}
    for i = 0, I:GetAllSubconstructsCount() - 1 do -- zero indexed
        local scid = I:GetSubConstructIdentifier(i)
        for j, name in ipairs(joint_names) do
            local custom_name = I:GetSubConstructInfo(scid).CustomName
            if custom_name == name then
                joints[j] = scid
            end
        end
    end
    return joints
end

L_TO_R = Matrix:new({
    {1, 0, 0},
    {0, 0, 1},
    {0, 1, 0}
})

CUSTOM_JOINT_NAMES = {"sb1", "sb2"}
-- sb1 should be rotated -90 around the z axis
-- sb2 is the rotation that the first joint of my robot arm has

function Update(I)
    
    I:Log(CUSTOM_JOINT_NAMES[1] .. " should be identity, \n" .. CUSTOM_JOINT_NAMES[2] .. " should be +90 degrees around z axis")

    -- find the subconstruct IDs for the first joint
    local joint_ids = get_joint_list(CUSTOM_JOINT_NAMES, I)

    -- get the quaternions of the joints
    local joint_info = {}
    for i, id in ipairs(joint_ids) do
        joint_info[i] = I:GetSubConstructInfo(id)
    end

    -- get and log the quaternions for each joint
    local q = {}
    for i, info in ipairs(joint_info) do
        q[i] = info.LocalRotation
        I:Log(string.format("Quaternion for joint %s: %s", CUSTOM_JOINT_NAMES[i], info.LocalRotation))
    end

    -- convert to rotation matrices
    local R = {}
    for i, quat in ipairs(q) do
        R[i] = quaternion_to_rotation_matrix(quat)
    end

    -- Log the rotation matrices
    for i, rot in ipairs(R) do
        I:Log(string.format("Rotation matrix for joint %s:\n%s", CUSTOM_JOINT_NAMES[i], rot:tostring()))
    end
end