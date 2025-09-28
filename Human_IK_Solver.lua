-- 7-DOF Inverse Kinematic Solver using Stereographic SEW Angle and IK-GEO Subproblems
-- Based on "Redundancy parameterization and inverse kinematics of 7-DOF revolute manipulators" by Elias & Wen
-- and "IK-Geo: Unified Robot Inverse Kinematics Using Subproblem Decomposition" by Elias and Wen

-- there may be some asyncronis behavior in the update function. Maybe I can include a delay or return checks

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

function Matrix:__add(other)
    local result = {}
    for i = 1, self.rows do
        result[i] = {}
        for j = 1, self.cols do
            result[i][j] = self[i][j] + other[i][j]
        end
    end
    return Matrix:new(result)
end

function Matrix:__sub(other)
    local result = {}
    for i = 1, self.rows do
        result[i] = {}
        for j = 1, self.cols do
            result[i][j] = self[i][j] - other[i][j]
        end
    end
    return Matrix:new(result)
end

function Matrix:__mul(other)
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

    -- matrix-vector multiplication
    if other.__typename == "Vector3" then
        local trace = "matrix-vector multiplication"

        -- For 3x3 matrix * Vector3, return a Vector3
        if self.rows == 3 then
            trace = self[1][3]
            return Vector3(
                self[1][1] * other.x + self[1][2] * other.y + self[1][3] * other.z,
                self[2][1] * other.x + self[2][2] * other.y + self[2][3] * other.z,
                self[3][1] * other.x + self[3][2] * other.y + self[3][3] * other.z
            )
        else
            -- For non-3x3 matrices, return a Matrix
            local result = {}
            for i = 1, self.rows do
                result[i] = {}
                result[i][1] = self[i][1] * other.x + self[i][2] * other.y + self[i][3] * other.z
            end
            return Matrix:new(result)
        end
    end

    -- matrix-matrix multiplication
    if other.__typename == "Matrix" then
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
end

function Matrix:__pow(exp)
    local result = self
    for _ = 1, exp - 1 do
        result = result * self
    end
    return result
end

function Matrix:__unm()
    local result = {}
    for i = 1, self.rows do
        result[i] = {}
        for j = 1, self.cols do
            result[i][j] = -self[i][j]
        end
    end
    return Matrix:new(result)
end

local function quaternion_to_rotation_matrix(q) -- convert quaternion q to a rotation matrix
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

local function transpose(m) -- tranposes a matrix or a Vector3
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

local function norm_squared(v) -- return the squared norm of a Vector3 or 1xN matrix
    -- this is a bit janky but works for the current calls
    if v.__typename == "Vector3" then
        return v.x^2 + v.y^2 + v.z^2
    else
        local result = 0
        for j = 1, v.cols do
            result = result + v[1][j] * v[1][j]
        end
        return result
    end
end

Matrix.Identity = Matrix:new({
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
})

RIGHT_ARM_PARAMETERS = {
    custom_joint_names = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"},

    -- Stereographic SEW parameters
    -- these will need to be made into variables so I can control the left and right arms
    e_t = Vector3(-1, 0, 0),    -- translation vector (singularity direction), Point away from workspace 
    e_r = Vector3(0, 0, -1),     -- Reference direction

    dse = 11,  -- shoulder to elbow distance
    dew = 11,  -- elbow to wrist distance  
    dwt = 3,   -- wrist to tool distance

    kinematics = {
        P = {
            -- Position vectors between joints (left-handed: x=right, y=up, z=forward)
            Vector3(8, 20, 0),   -- Base to Shoulder (joint 1)
            Vector3(0, 0, 0),    -- Shoulder to joint 2
            Vector3(0, 0, 0),    -- joint 2 to joint 3
            Vector3(0,-RIGHT_ARM_PARAMETERS.dse, 0),  -- joint 3 to joint 4 (elbow)
            Vector3(0, 0, RIGHT_ARM_PARAMETERS.dew),    -- joint 4 to joint 5
            Vector3(0, 0, 0),  -- joint 5 to joint 6 (wrist)
            Vector3(0, 0, 0),    -- joint 6 to joint 7
            Vector3(0, 0, RIGHT_ARM_PARAMETERS.dwt)   -- joint 7 to end effector
        },
        H = {
            -- GLOBAL joint rotation axes - these need to be measured from your actual robot
            Vector3( 1, 0, 0),  -- Joint 1: X-axis
            Vector3( 0, 0, 1),  -- Joint 2: Z-axis
            Vector3( 0,-1, 0),  -- Joint 3:-Y-axis
            Vector3(-1, 0, 0),  -- Joint 4:-X-axis
            Vector3( 0, 0, 1),  -- Joint 5: Z-axis
            Vector3(-1, 0, 0),  -- Joint 6:-X-axis
            Vector3( 0, 1, 0)   -- Joint 7: Y-axis
        }
    }
}

-- Helper functions

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

local function set_joint_angles(joints, joint_angles, I) -- sets the angles of the spin blocks given a list of subconstruct IDs and angles in radians
    if not joint_angles then
        I:Log("Error: No joint angles provided")
        return
    end
    
    for i, scid in ipairs(joints) do
        if joint_angles[i] then
            -- Convert from radians to degrees and account for left-handed rotation
            local angle_degrees = joint_angles[i] * 180 / math.pi
            I:SetSpinBlockRotationAngle(scid, angle_degrees)
        end
    end
end

-- get position of end effector
-- returns a Vector3 and a rotation Matrix 
local function get_end_effector_pose(joints, arm_parameters, I)
    local kin = arm_parameters.kinematics
    local position = Vector3(0, 0, 0)
    local rotation = Quaternion.identity
    for i, scid in ipairs(joints) do
        BlockInfo = I:GetSubConstructInfo(scid)
        position = position + rotation * BlockInfo.LocalPosition
        rotation = rotation * BlockInfo.LocalRotation
    end
    -- add on the end effector offset
    position = position + rotation * kin.P[8]
    return position, quaternion_to_rotation_matrix(rotation)
end

-- subproblems
local function subproblem_4(h, p, k, d)
    local A_11 = Vector3.Cross(k, p)
    local A_11m = Vector3.Cross(A_11, k)
    -- 3x2 matrix
    local A_1 = Matrix:new({
        {A_11.x, A_11m.x},
        {A_11.y, A_11m.y},
        {A_11.z, A_11m.z}
    })

    local A = transpose(h) * A_1

    local b = d - Vector3.Dot(h, k) * (Vector3.Dot(k, p))

    local norm_A_2 = norm_squared(A)

    local x_ls_tilde = transpose(A_1) * (h * b)

    local theta, is_ls

    if norm_A_2 > (b * b) then
        local xi = math.sqrt(norm_A_2 - b * b)
            
        local x_N_prime_tilde = Matrix:new({
            {A[1][2]}, 
            {-A[1][1]}
        })

        local sc_1 = x_ls_tilde + x_N_prime_tilde * xi
        local sc_2 = x_ls_tilde - x_N_prime_tilde * xi
    
        ---@diagnostic disable-next-line: deprecated
        theta = {math.atan2(sc_1[1][1], sc_1[2][1]), math.atan2(sc_2[1][1], sc_2[2][1])}
        is_ls = false
    else
        ---@diagnostic disable-next-line: deprecated
        theta = {math.atan2(x_ls_tilde[1][1], x_ls_tilde[2][1])}
        is_ls = true
    end

    return theta, is_ls
end

local function subproblem_3(p1, p2, k, d)
    return subproblem_4(p2, p1, k, 1/2 * (Vector3.Dot(p1, p1) + Vector3.Dot(p2, p2) - d * d))
end

local function subproblem_2(p1, p2, k1, k2)
    local p1_norm = p1.normalized
    local p2_norm = p2.normalized

    local theta1, theta1_is_ls = subproblem_4(k2, p1_norm, k1, Vector3.Dot(k2, p2_norm))
    local theta2, theta2_is_ls = subproblem_4(k1, p2_norm, k2, Vector3.Dot(k1, p1_norm))

    -- reverse theta2 and duplicate any angle with less solutions
    if #theta1 > 1 or #theta2 > 1 then
        theta1 = {theta1[1], theta1[#theta1]}
        theta2 = {theta2[#theta2], theta2[1]}
    end

    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-8 or theta1_is_ls or theta2_is_ls

    return theta1, theta2, is_ls
end

local function subproblem_1(p1, p2, k)
    local KxP = Vector3.Cross(k, p1)
    local kxm = Vector3.Cross(KxP, k)
    -- make a 3x2 matrix

    local A = Matrix:new{
        {KxP.x, kxm.x},
        {KxP.y, kxm.y},
        {KxP.z, kxm.z}
    }

    local x = transpose(A) * p2

    ---@diagnostic disable-next-line: deprecated
    local theta = math.atan2(x[1][1], x[2][1])
    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-6 or math.abs(Vector3.Dot(k, p1) - Vector3.Dot(k, p2)) > 1e-6

    return theta, is_ls
end

-- Inverse Kinematics Solver
-- find the matlab code thsi is written from here https://github.com/rpiRobotics/stereo-sew/blob/main/IK_helpers/sew_stereo.m#L44
-- e_T = obj.R
-- e_R = obj.V
local function inv_kin(S, W, psi, e_t, e_r)
    local p_SW = W-S
    local e_SW = p_SW.normalized
    local k_r = Vector3.Cross(e_SW - e_t, e_r)
    local k_x = Vector3.Cross(k_r, p_SW)
    local e_x = k_x.normalized
    
    local e_CE = rot(e_SW, psi)*e_x
    local n_SEW = Vector3.Cross(e_SW, e_CE)

    return e_CE, n_SEW
end

local function IK_3R_R_3R(R_07, p_0T, psi, arm_parameters) -- returns a list of joint angle solutions
    local kin = arm_parameters.kinematics
    local e_t = arm_parameters.e_t
    local e_r = arm_parameters.e_r

    local Q = {}
    local is_LS_vec = {}

    -- find wrist position
    local W = p_0T - R_07 * kin.P[8]

    -- find shoulder position
    local S = kin.P[1]

    local p_SW = W - S
    local e_SW = p_SW / p_SW.magnitude

    local e_CE, n_SEW = inv_kin(S, W, psi, e_t, e_r) -- check inv_kin function

    -- Use subproblem 3 to find q4
    local t4, t4_is_LS = subproblem_3(kin.P[5], -kin.P[4], kin.H[4], p_SW.magnitude) -- check subproblem 3 function

    for i_4 = 1, #t4 do
        local q4 = t4[i_4]

        -- Solve for theta_b, theta_c using subproblem 2
        -- Only keep one solution, as they will represent the same R_03
        local t_b, t_c, t_bc_is_LS = subproblem_2(p_SW, kin.P[4] + rot(kin.H[4], q4) * kin.P[5], -n_SEW, e_SW)
        local theta_b = t_b[1]
        local theta_c = t_c[1]

        -- Solve for theta_a using subproblem 4
        -- Keep only solutions that put the elbow in the correct half plane
        local t_a, t_a_is_LS = subproblem_4(n_SEW, rot(n_SEW, theta_b) * rot(e_SW, theta_c) * kin.P[4], e_SW, 0)

        for i_a = 1, #t_a do
            local theta_a = t_a[i_a]

            R_03 = rot(e_SW, theta_a) * rot(n_SEW, theta_b) * rot(e_SW, theta_c)
            if Vector3.Dot(e_CE, R_03 * kin.P[4]) < 0 then
                goto continue -- Shoulder must be in correct half plane
            end
            local p_spherical_1 = kin.H[2] -- Must be noncollinear with h_3

            -- Find q_1, q_2, q_3 with subproblems 2 and 1
            local t2, t1, t12_is_LS = subproblem_2(kin.H[3], R_03 * kin.H[3], kin.H[2], -kin.H[1])
            for i_12 = 1, #t1 do
                local q1 = t1[i_12]
                local q2 = t2[i_12]
                local q3, q3_is_LS = subproblem_1(p_spherical_1, transpose(rot(kin.H[1], q1) * rot(kin.H[2], q2)) * R_03 * p_spherical_1, kin.H[3])

                -- Find q_5, q_6, q_7 with subproblems 2 and 1
                R_01 = rot(kin.H[1], q1)
                R_12 = rot(kin.H[2], q2)
                R_23 = rot(kin.H[3], q3)
                R_34 = rot(kin.H[4], q4)
                R_47 = transpose(R_01 * R_12 * R_23 * R_34) * R_07
                local p_spherical_2 = kin.H[6] -- Must be noncollinear with h_7
                local t6, t5, t56_is_LS = subproblem_2(kin.H[7], R_47 * kin.H[7], kin.H[6], -kin.H[5])
                for i_56 = 1, #t5 do
                    local q5 = t5[i_56]
                    local q6 = t6[i_56]
                    local q7, q7_is_LS = subproblem_1(p_spherical_2, transpose(rot(kin.H[5], q5) * rot(kin.H[6], q6)) * R_47 * p_spherical_2, kin.H[7])
                    local q_i = {q1, q2, q3, q4, q5, q6, q7}

                    -- Append solution
                    table.insert(Q, q_i)
                    table.insert(is_LS_vec, {t4_is_LS or t_bc_is_LS or t_a_is_LS or t12_is_LS or q3_is_LS or t56_is_LS or q7_is_LS})
                end
            end
            ::continue::
        end
    end
    return Q, is_LS_vec
end

local function move_arm_to_target(target_pos, target_rot, psi, solution, RIGHT_ARM_PARAMETERS, I)
    local Q, is_ls_vec = IK_3R_R_3R(target_rot, target_pos, psi, RIGHT_ARM_PARAMETERS)
    local joints = get_joint_list(RIGHT_ARM_PARAMETERS.custom_joint_names, I)
    set_joint_angles(joints, Q[solution], I)

end

function Update(I)
    -- use built in functions like I:GetSubConstructInfo(scid) to get blockinfo 
    local target_pos = Vector3(15, 8, 10)
    local target_rot = Matrix.Identity
    local psi = math.pi/4  -- SEW angle

    move_arm_to_target(target_pos, target_rot, psi, 1, RIGHT_ARM_PARAMETERS, I)
end