-- 7-DOF Inverse Kinematic Solver using Stereographic SEW Angle and IK-GEO Subproblems
-- Based on "Redundancy parameterization and inverse kinematics of 7-DOF revolute manipulators" by Elias & Wen
-- and "IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition" by Elias and Wen

-- there may be some asyncronis behavior in the update function. Maybe I can include a delay or return checks

-- matrix class
Matrix = {}
Matrix.__typename = "Matrix"

-- Fixed Matrix constructor to prevent reference sharing
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

-- function Matrix:new(data)
--     data = data
--     self.rows = #data
--     self.cols = #data[1] or 1
--     setmetatable(data, self)
--     self.__index = self
--     return data
-- end

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

-- -- can return either a Vector3 or a table depending on input type
-- function Matrix:mulVector(other)
--     -- Vector3
--     if self.cols ~= 3 or self.rows ~= 3 then
--         local result = {}

--         for i = 1, self.rows do
--             result[i] = {}
--             result[i][1] = self[1][1] * other.x + self[1][2] * other.y + self[1][3] * other.z
--         end

--         return Matrix:new(result)
--     end
--     return Vector3(
--         self[1][1] * other.x + self[1][2] * other.y + self[1][3] * other.z,
--         self[2][1] * other.x + self[2][2] * other.y + self[2][3] * other.z,
--         self[3][1] * other.x + self[3][2] * other.y + self[3][3] * other.z
--     )
-- end


-- Fixed transpose function for Vector3
function transpose(I, m)
    if m.__typename == "Vector3" then
        -- Vector3 transpose should be 1x3 (row vector)
        return Matrix:new({
            {m.x, m.y, m.z}
        })
    end

    -- For matrices, swap rows and columns
    local result = {}
    for j = 1, m.cols do
        result[j] = {}
        for i = 1, m.rows do
            result[j][i] = m[i][j]
        end
    end

    return Matrix:new(result)
end

-- -- transpose function
-- function transpose(I, m) -- transpose of the matrix
--     if m.__typename == "Vector3" then
--         return Matrix:new({
--             {m.x, m.y, m.z}
--         })
--     end

--     -- initalize result matrix
--     local result = {}

--     for j = 1, m.cols do
--         result[j] = {}
--         for i = 1, m.rows do
--             result[j][i] = m[i][j]
--         end
--     end

--     return Matrix:new(result)
-- end

function Matrix:get_column(col) -- I may wanto to return a table instead of a Vector3
    local result = Vector3(0, 0, 0)
    for i = 1, self.rows do
        result[i] = self[i][col]
    end
    return result
end

-- function Matrix:__mul(other)
--     -- matrix-vector multiplication
--     if other.__typename == "Vector3" then
--         return self:mulVector(other)
--     end

--     -- matrix-matrix multiplication
--     if other.__typename == "Matrix" then
--         local result = {}
--         for i = 1, self.rows do
--             result[i] = {}
--             for j = 1, other.cols do
--                 result[i][j] = 0
--                 for k = 1, self.cols do
--                     result[i][j] = result[i][j] + self[i][k] * other[k][j]
--                 end
--             end
--         end
--         return Matrix:new(result)
--     end

--     -- scalar multiplication
--     if type(other) == "number" then
--         local result = {}
--         for i = 1, self.rows do
--             result[i] = {}
--             for j = 1, self.cols do
--                 result[i][j] = self[i][j] * other
--             end
--         end
--         return Matrix:new(result)
--     end
-- end

-- Fixed Matrix:mulVector function
function Matrix:mulVector(other)
    -- Check if this is a 3x3 matrix multiplying a Vector3 (return Vector3)
    if self.cols == 3 and self.rows == 3 then
        return Vector3(
            self[1][1] * other.x + self[1][2] * other.y + self[1][3] * other.z,
            self[2][1] * other.x + self[2][2] * other.y + self[2][3] * other.z,
            self[3][1] * other.x + self[3][2] * other.y + self[3][3] * other.z
        )
    else
        -- General case: return a matrix
        if self.cols ~= 3 then
            error("Matrix column count (" .. self.cols .. ") doesn't match Vector3 dimension (3)")
        end
        
        local result = {}
        for i = 1, self.rows do
            result[i] = {}
            result[i][1] = self[i][1] * other.x + self[i][2] * other.y + self[i][3] * other.z
        end
        
        return Matrix:new(result)
    end
end

-- Alternative simpler approach - always return a matrix for consistency
function Matrix:mulVector(other)
    if self.cols ~= 3 then
        error("Matrix column count (" .. self.cols .. ") doesn't match Vector3 dimension (3)")
    end
    
    local result = {}
    for i = 1, self.rows do
        result[i] = {}
        result[i][1] = self[i][1] * other.x + self[i][2] * other.y + self[i][3] * other.z
    end
    
    return Matrix:new(result)
end

function Matrix:__mul(other)
    -- matrix-vector multiplication
    if other.__typename == "Vector3" then
        if self.cols ~= 3 then
            error("Matrix column count (" .. self.cols .. ") doesn't match Vector3 dimension (3)")
        end
        
        -- Always return a matrix for consistency
        local result = {}
        for i = 1, self.rows do
            result[i] = {}
            result[i][1] = self[i][1] * other.x + self[i][2] * other.y + self[i][3] * other.z
        end
        return Matrix:new(result)
    end

    -- matrix-matrix multiplication
    if other.__typename == "Matrix" then
        if self.cols ~= other.rows then
            error("Matrix dimension mismatch: " .. self.rows .. "x" .. self.cols .. " * " .. other.rows .. "x" .. other.cols)
        end
        
        local result = {}
        for i = 1, self.rows do
            result[i] = {}
            for j = 1, other.cols do
                result[i][j] = 0
                for k = 1, self.cols do
                    result[i][j] = result[i][j] + self[i][k] * other[k][j]
                end
            end
        end
        return Matrix:new(result)
    end

    -- scalar multiplication
    if type(other) == "number" then
        local result = {}
        for i = 1, self.rows do
            result[i] = {}
            for j = 1, self.cols do
                result[i][j] = self[i][j] * other
            end
        end
        return Matrix:new(result)
    end
    
    error("Unsupported multiplication type: " .. type(other))
end

function Matrix:__pow(exp)
    for _ = 1, exp - 1 do
        self = self * self
    end
end

function Matrix:__unm()
    local result = Matrix:new()
    for i = 1, 3 do
        for j = 1, 3 do
            result[i][j] = -self[i][j]
        end
    end
    return result
end

-- function Vector3:transpose(self)
--     return Matrix:new{
--         {m.x, m.y, m.z}
--     }
-- end

local function rot(axis, theta) -- rotation matrix
    local angle = theta
    local c, s = math.cos(angle), math.sin(angle)
    local t = 1 - c
    local x, y, z = axis.x, axis.y, axis.z
    return Matrix:new{
        {t * x * x + c, t * x * y - z * s, t * x * z + y * s},
        {t * x * y + z * s, t * y * y + c, t * y * z - x * s},
        {t * x * z - y * s, t * y * z + x * s, t * z * z + c}
    }
end

local function skew_symmetric(v) -- Create a skew-symmetric matrix from a vector
    return Matrix:new{
        {0, -v.z, v.y},
        {v.z, 0, -v.x},
        {-v.y, v.x, 0}
    }
end

local function norm_squared(v)
    if v.__typename == "Vector3" then
        return v.x^2 + v.y^2 + v.z^2
    elseif v.__typename == "Matrix" and v.cols == 1 and v.rows == 3 then
        return v[1][1]^2 + v[2][1]^2 + v[3][1]^2
    end
end

local function vector_angle(v1, v2)
    local dot_product = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z
    local magnitude1 = math.sqrt(norm_squared(v1))
    local magnitude2 = math.sqrt(norm_squared(v2))
    return math.acos(dot_product / (magnitude1 * magnitude2))
end

-- Constants and parameters
Matrix.Identity = Matrix:new({
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
})

custom_joint_names = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"}

-- Stereographic SEW parameters
-- e_t: translation vector (singularity direction) - point away from workspace
-- e_r: reference vector for SEW angle measurement
local E_T = Vector3(0, 0, -1)    -- Point downward (away from workspace)
local E_R = Vector3(1, 0, 0)     -- Reference direction


local dse = 11  -- shoulder to elbow distance
local dew = 11  -- elbow to wrist distance  
local dwt = 3   -- wrist to tool distance

kin = {
    P = {
        Vector3(0, 0, 0),  -- Base to Shoulder
        Vector3(0, 0, 0),  -- Shoulder to Joint 2
        Vector3(0, 0, 0),  -- Joint 2 to Joint
        Vector3(0, 0, dse),  -- Joint 3 to Joint 4
        Vector3(0, 0, 0),  -- Joint 4 to Joint 5
        Vector3(0, 0, dew),  -- Joint 5 to Joint 6
        Vector3(0, 0, 0),  -- Joint 6 to Joint 7
        Vector3(dwt, 0, 0)   -- Joint 7 to End Effector
    },
    H = { -- Joint axes (these don't seem to match the robot, need to verify DH_PARAMS)
        Vector3(0, 0, 1), -- Joint 1:  z-axis
        Vector3(0, 1, 0), -- Joint 2:  y-axis
        Vector3(0, 0, 1), -- Joint 3:  z-axis
        Vector3(0, 1, 0), -- Joint 4:  y-axis
        Vector3(0, 0, 1), -- Joint 5:  z-axis
        Vector3(0, 1, 0), -- Joint 6:  y-axis
        Vector3(0, 0, 1)  -- Joint 7:  z-axis
    },
    RT = {
        {1, 0, 0},
        {0, 1, 0},
        {0, 0, 1}
    }
}

-- DH Parameters for 7-DOF humanoid arm: {a, alpha, d, theta_offset}
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
    Vector3( 1,  0,  0), -- Joint 1:  x-axis
    Vector3( 0,  1,  0), -- Joint 2:  y-axis
    Vector3( 0,  0, -1), -- Joint 3: -z-axis
    Vector3(-1,  0,  0), -- Joint 4: -x-axis
    Vector3( 0,  1,  0), -- Joint 5:  y-axis
    Vector3(-1,  0,  0), -- Joint 6: -x-axis
    Vector3( 0,  0,  1)  -- Joint 7:  z-axis
}

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

-- subproblems
local function subproblem_4(I, h, p, k, d)
    local A_11 = Vector3.Cross(k, p)
    local A_11m = -Vector3.Cross(k, A_11)
    -- 3x2 matrix
    local A_1 = Matrix:new({
        {A_11.x, A_11m.x},
        {A_11.y, A_11m.y},
        {A_11.z, A_11m.z}
    })

    I:Log(A_1.rows .. "x" .. A_1.cols)
    I:Log(transpose(I, h).rows .. "x" .. transpose(I, h).cols)
    I:Log(A_1.rows .. "x" .. A_1.cols)

    local A = transpose(I, h) * A_1

    local b = d - Vector3.Dot(h, k) * (Vector3.Dot(k, p))

    local norm_A_2 = norm_squared(A)

    local temp = h * b
    local x_ls_tilde = A_1 * temp

    if norm_A_2 > b * b then
        local xi = math.sqrt(norm_A_2 - b * b)
        local x_N_prime_tilde = {A[2],-A[1]}

        local sc_1 = x_ls_tilde + xi * x_N_prime_tilde
        local sc_2 = x_ls_tilde - xi * x_N_prime_tilde

        theta = {math.atan(sc_1[1], sc_1[2]), math.atan(sc_2[1], sc_2[2])}
        is_ls = false
    else
        theta = {math.atan(x_ls_tilde[1], x_ls_tilde[2]), true}
    end

    return theta, is_ls
end

local function subproblem_3(I, p1, p2, k, d)
    return subproblem_4(I, p2, p1, k, 1/2 * (Vector3.Dot(p1, p1) + Vector3.Dot(p2, p2) - d * d))
end

local function subproblem_2(I, p1, p2, k1, k2)
    local p1_norm = p1.normalized
    local p2_norm = p2.normalized

    local theta1, theta1_is_ls = subproblem_4(I, k2, p1_norm, k1, k2:dot(p2_norm))
    local theta2, theta2_is_ls = subproblem_4(I, k1, p2_norm, k2, k1:dot(p1_norm))

    -- reverse theta2 and duplicate any angle with less solutions
    if #theta1 > 1 or #theta2 > 1 then
        theta1 = {theta1[1], theta1[#theta1]}
        theta2 = {theta2[#theta2], theta2[1]}
    end

    local is_ls = math.abs(p1.magnitude() - p2.magnitude()) > 1e-8 or theta1_is_ls or theta2_is_ls

    return theta1, theta2, is_ls
end

local function subproblem_1(I, p1, p2, k)
    local kxp = Vector3.Cross(k, p1)
    local kxm = -Vector3.Cross(k, kxp)
    -- make a 3x2 matrix

    local A = Matrix:new{
        {kxp.x, kxm.x},
        {kxp.y, kxm.y},
        {kxp.z, kxm.z}
    }

    local x = transpose(I, A) * p2

    local theta = math.atan(x[1], x[2])
    local is_ls = math.abs(p1.magnitude() - p2.magnitude()) > 1e-6 or math.abs(k.dot(p1) - k.dot(p2)) > 1e-6

    return theta, is_ls
end

-- Inverse Kinematics Solver
local function inv_kin(S, W, psi)
    local p_SW = W-S
    local e_SW = p_SW.normalized
    local k_r = Vector3.Cross(e_SW - E_T, E_R)
    local k_x = Vector3.Cross(k_r, p_SW)
    local e_x = k_x.normalized
    
    local e_CE = rot(e_SW, psi)*e_x
    local n_SEW = Vector3.Cross(e_SW, e_CE)

    return e_CE, n_SEW
end

local function IK_3R_R_3R(I, R_07, p_0T, psi, kin)
    Q = {}
    local is_LS_vec = {}

    -- find wrist position
    local W = p_0T - R_07 * kin.P[8]

    -- find shoulder position
    local S = kin.P[1]

    local p_SW = W-S
    local e_SW = p_SW / p_SW.magnitude

    local e_CE, n_SEW = inv_kin(S, W, psi)

    -- Use subproblem 3 to find q4
    local t4, t4_is_LS = subproblem_3(I, kin.P[5], -kin.P[4], kin.H[4], p_SW.magnitude)

    for i_4 = 1,#t4 do
        q4 = t4[i_4]
    end

    -- Solve for theta_b, theta_c using subproblem 2
    -- Only keep one solution, as they will represent the same R_03
    t_b, t_c, t_bc_is_LS = subproblem_2(p_SW, kin.P[4] + rot(kin.H[4],q4)*kin.P[5], -n_SEW,e_SW)
    local theta_b = t_b[1]
    local theta_c = t_c[1]

    -- Solve for theta_a using subproblem 4
    -- Keep only solutions that put the elbow in the correct half plane
    local t_a, t_a_is_LS = subproblem_4(n_SEW, rot(n_SEW, theta_b)*rot(e_SW, theta_c)*kin.P[4], e_SW, 0)
    for i_a = 1, #t_a do
        local theta_a = t_a[i_a]

        R_03 = rot(e_SW, theta_a)*rot(n_SEW, theta_b)*rot(e_SW, theta_c)
        if transpose(I, e_CE) * R_03 * kin.P[4] < 0 then
            goto continue -- Shoulder must be in correct half plane
        end
        local p_spherical_1 = kin.H[2] -- Must be noncollinear with h_3

        -- Find q_1, q_2, q_3 with subproblems 2 and 1
        t2, t1, t12_is_LS = subproblem_2(kin.H[3], R_03*kin.H[3], kin.H[2], -kin.H[1])
        for i_12 = 1, #t1 do
            local q1 = t1[i_12]
            local q2 = t2[i_12]
            local q3, q3_is_LS = subproblem_1(p_spherical_1, transpose(I, rot(kin.H[1], q1) * rot(kin.H[2], q2)) * R_03 * p_spherical_1, kin.H[3])

            -- Find q_5, q_6, q_7 with subproblems 2 and 1
            R_01 = rot(kin.H[1], q1)
            R_12 = rot(kin.H[2], q2)
            R_23 = rot(kin.H[3], q3)
            R_34 = rot(kin.H[4], q4)
            R_47 = transpose(I, R_01 * R_12 * R_23 * R_34) * R_07
            local p_spherical_2 = kin.H[6] -- Must be noncollinear with h_7
            t6, t5, t56_is_LS = subproblem_2(kin.H[7], R_47*kin.H[7], kin.H[6], -kin.H[5])
            for i_56 = 1, #t5 do
                local q5 = t5[i_56]
                local q6 = t6[i_56]
                local q7, q7_is_LS = subproblem_1(p_spherical_2, transpose(I, rot(kin.H[5], q5) * rot(kin.H[6], q6)) * R_47 * p_spherical_2, kin.H[7])
                local q_i = {q1, q2, q3, q4, q5, q6, q7}

                Q = {Q, q_i}
                -- is_LS_vec = [is_LS_vec [t4_is_LS;t_bc_is_LS;t_a_is_LS;t12_is_LS;q3_is_LS;t56_is_LS;q7_is_LS]];
                is_LS_vec = {is_LS_vec, {t4_is_LS or t_bc_is_LS or t_a_is_LS or t12_is_LS or q3_is_LS or t56_is_LS or q7_is_LS}}
            end
        end
        ::continue::
    end
end

function Update(I)
    -- use built in functions like I:GetSubConstructInfo(scid) to get blockinfo 
    local target_pos = Vector3(15, 0, 10)
    local target_rot = Matrix.Identity
    local psi = math.pi/4  -- SEW angle

    local Q, is_ls_vec = IK_3R_R_3R(I, target_rot, target_pos, psi, kin)

    local joints = get_joint_list(I, custom_joint_names)
    set_joint_angles(I, joints, Q[1])  -- Set to first solution
end