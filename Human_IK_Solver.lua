-- Matrix and vector operations
local function matrix_mult(m1, m2)
    local result = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}
    for i = 1, 3 do
        for j = 1, 3 do
            for k = 1, 3 do
                result[i][j] = result[i][j] + m1[i][k] * m2[k][j]
            end
        end
    end
    return result
end

local function rotation_matrix(axis, theta)
    -- Account for left-handed rotation by negating the angle
    local angle = -theta
    local c, s = math.cos(angle), math.sin(angle)
    local t = 1 - c
    local x, y, z = axis.x, axis.y, axis.z
    return {
        {t * x * x + c, t * x * y - z * s, t * x * z + y * s},
        {t * x * y + z * s, t * y * y + c, t * y * z - x * s},
        {t * x * z - y * s, t * y * z + x * s, t * z * z + c}
    }
end

local function matrix_transpose(m)
    return {
        {m[1][1], m[2][1], m[3][1]},
        {m[1][2], m[2][2], m[3][2]},
        {m[1][3], m[2][3], m[3][3]}
    }
end

local function matrix_vector_mult(m, v)
    return Vector3(
        m[1][1] * v.x + m[1][2] * v.y + m[1][3] * v.z,
        m[2][1] * v.x + m[2][2] * v.y + m[2][3] * v.z,
        m[3][1] * v.x + m[3][2] * v.y + m[3][3] * v.z
    )
end

local function skew_symmetric(v)
    return {
        {0, -v.z, v.y},
        {v.z, 0, -v.x},
        {-v.y, v.x, 0}
    }
end

-- Constants and parameters
IDENTITY_MATRIX = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
}

custom_joint_names = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"}

-- DH Parameters for 7-DOF humanoid arm: {a, alpha, d, theta_offset}
local dse = 11  -- shoulder to elbow distance
local dew = 11  -- elbow to wrist distance  
local dwt = 3   -- wrist to tool distance

DH_PARAMS = {
    {0, -math.pi/2, 0, 0},         -- Joint 1
    {0, math.pi/2, 0, 0},          -- Joint 2
    {0, -math.pi/2, dse, 0},       -- Joint 3
    {0, math.pi/2, 0, 0},          -- Joint 4
    {0, -math.pi/2, dew, 0},       -- Joint 5
    {0, math.pi/2, 0, 0},          -- Joint 6
    {dwt, 0, 0, 0}                 -- Joint 7
}

-- Joint axes in base frame at zero configuration
JOINT_AXES = {
    Vector3(0, 0, 1),    -- Joint 1: z-axis
    Vector3(1, 0, 0),    -- Joint 2: x-axis
    Vector3(0, 1, 0),    -- Joint 3: y-axis
    Vector3(1, 0, 0),    -- Joint 4: x-axis
    Vector3(0, 1, 0),    -- Joint 5: y-axis
    Vector3(1, 0, 0),    -- Joint 6: x-axis
    Vector3(0, 0, 1)     -- Joint 7: z-axis
}

-- Stereographic SEW parameters
-- e_t: translation vector (singularity direction) - point away from workspace
-- e_r: reference vector for SEW angle measurement
local E_T = Vector3(0, 0, -1)    -- Point downward (away from workspace)
local E_R = Vector3(1, 0, 0)     -- Reference direction

-- Helper functions
local function get_joint_list(I, joint_names)
    local joints = {}
    for i = 1, I:GetAllSubconstructsCount() do
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

local function set_joint_angles(I, joints, joint_angles)
    if not joint_angles then
        I:Log("Error: No joint angles provided")
        return
    end
    
    for i, scid in ipairs(joints) do
        if joint_angles[i] then
            -- Convert from radians to degrees and account for left-handed rotation
            local angle_degrees = -joint_angles[i] * 180 / math.pi
            I:SetSpinBlockRotationAngle(scid, angle_degrees)
        end
    end
end