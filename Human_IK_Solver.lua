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


LEFT_ARM_PARAMETERS = {
    custom_joint_names = {"lj1", "lj2", "lj3", "lj4", "lj5", "lj6", "lj7"},

    -- Stereographic SEW parameters
    -- these will need to be made into variables so I can control the left and right arms
    e_t = Vector3(1, 0, 0),    -- translation vector (singularity direction), Point away from workspace 
    e_r = Vector3(0, 1, 0),     -- Reference direction

    dse = 11,  -- shoulder to elbow distance
    dew = 11,  -- elbow to wrist distance
    dwt = 3,   -- wrist to tool distance

    kinematics = {}
}
LEFT_ARM_PARAMETERS.kinematics = {
    P = {
        -- Position vectors between joints (left-handed: x=right, y=up, z=forward)
        Vector3(-8, 20, 0),   -- Base to Shoulder (joint 1)
        Vector3( 0, 0, 0),    -- Shoulder to joint 2
        Vector3( 0, 0, 0),    -- joint 2 to joint 3
        Vector3( 0,-LEFT_ARM_PARAMETERS.dse, 0),  -- joint 3 to joint 4 (elbow)
        Vector3( 0, 0, LEFT_ARM_PARAMETERS.dew),    -- joint 4 to joint 5
        Vector3( 0, 0, 0),  -- joint 5 to joint 6 (wrist)
        Vector3( 0, 0, 0),    -- joint 6 to joint 7
        Vector3( 0, 0, LEFT_ARM_PARAMETERS.dwt)   -- joint 7 to end effector
    },
    H = {
        -- GLOBAL joint rotation axes - these need to be measured from your actual robot
        Vector3(-1, 0, 0),  -- Joint 1: X-axis
        Vector3( 0, 0, 1),  -- Joint 2: Z-axis
        Vector3( 0,-1, 0),  -- Joint 3:-Y-axis
        Vector3( 1, 0, 0),  -- Joint 4:-X-axis
        Vector3( 0, 0, 1),  -- Joint 5: Z-axis
        Vector3( 1, 0, 0),  -- Joint 6:-X-axis
        Vector3( 0, 1, 0)   -- Joint 7: Y-axis
    }
}

RIGHT_ARM_PARAMETERS = {
    custom_joint_names = {"rj1", "rj2", "rj3", "rj4", "rj5", "rj6", "rj7"},

    -- Stereographic SEW parameters
    -- these will need to be made into variables so I can control the left and right arms
    e_t = Vector3(-1, 0, 0),    -- translation vector (singularity direction), Point away from workspace 
    e_r = Vector3(0, 1, 0),     -- Reference direction

    dse = 11,  -- shoulder to elbow distance
    dew = 11,  -- elbow to wrist distance
    dwt = 3,   -- wrist to tool distance

    kinematics = {}
}
RIGHT_ARM_PARAMETERS.kinematics = {
    P = {
        -- Position vectors between joints (left-handed: x=right, y=up, z=forward)
        Vector3( 8, 20, 0),   -- Base to Shoulder (joint 1)
        Vector3( 0, 0, 0),    -- Shoulder to joint 2
        Vector3( 0, 0, 0),    -- joint 2 to joint 3
        Vector3( 0,-RIGHT_ARM_PARAMETERS.dse, 0),  -- joint 3 to joint 4 (elbow)
        Vector3( 0, 0, RIGHT_ARM_PARAMETERS.dew),    -- joint 4 to joint 5
        Vector3( 0, 0, 0),  -- joint 5 to joint 6 (wrist)
        Vector3( 0, 0, 0),    -- joint 6 to joint 7
        Vector3( 0, 0, RIGHT_ARM_PARAMETERS.dwt)   -- joint 7 to end effector
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

local function move_arm_to_target(arm_parameters, target_rot, target_pos, psi, solution, I)
    local Q, is_ls_vec = IK_3R_R_3R(target_rot, target_pos, psi, arm_parameters)
    local joints = get_joint_list(arm_parameters.custom_joint_names, I)
    set_joint_angles(joints, Q[solution], I)
end


-- this function is taken from the code controling the Depthian https://steamcommunity.com/sharedfiles/filedetails/?id=2792198306&searchtext=depthian
-- but has been made into a closure so easing functions can be saved into the keyframes.
-- function to produce a smoth transition between two values
-- x1 and x2 change the start and stop of the smoth curve
-- a and b change the shape of the curve
local function new_easing_function(a,b)
    --[[
    x1 to x2 ramps from 0 to 1
    a=1,b=1 proportional
    a>1,b=1 power function
    a<1,b>1 looks like an exponential  ( a=(10-b)/10 rather pretty )
    a>1,b<1 looks like an logarithmic  ( a=1.3,b=0.5,a=1.7,b=0.3,a=2,b=0.2,a=3.2,b=0.1) ( a=( 1+(1-b)/2 )*10/9 rather pretty )
    ]]

    return function(time)
        -- var1 starts at 0, then ramps linearly to 1 between x1 and x2, then stays at 1.
        -- This is used by the if statement blow to decide which curve to use
        local var1 = Mathf.Clamp(time,0,1)
        
        local var2 = 1
        if var1 <= 0.5 then -- first half of the curve
            var2 = 0.5*(2*var1)^a
        elseif var1 <= 1 then -- second half of the curve
            var2 = 1-0.5*(-2*var1+2)^a
        end
        return var2^b
    end
end

local function angle_ease(a1, a2, easing_function, t) -- handles angle wrap around
    local delta = a2 - a1
    if delta > math.pi then
        delta = delta - 2 * math.pi
    elseif delta < -math.pi then
        delta = delta + 2 * math.pi
    end
    return a1 + delta * easing_function(t)
end

local control_point = {}
function control_point:new(position, rotation, sew_angle)
    local obj = {
        position = position or Vector3(0, 0, 0),
        rotation = rotation or Quaternion.Identity,
        sew_angle = sew_angle or 0
    }
    setmetatable(obj, self)
    self.__index = self
    return obj
end

local keyframe = {}
function keyframe:new(obj)
    local obj = obj or {
        time = 0,
        right_arm = {
            position = Vector3(0, 0, 0),
            rotation = Quaternion.Identity, -- Quaternion so we can do slerp
            sew_angle = 0,
            -- interpolations between keyframe i and i+1 will use the easing function of keyframe i+1
            easing_function = nil,
            ik_solution = 1,
            control_points = {}
        },
        left_arm = {
            position = Vector3(0, 0, 0),
            rotation = Quaternion.Identity,
            sew_angle = 0,
            easing_function = nil,
            ik_solution = 1,
            control_points = {}
        },
        right_hand = {
            -- TODO: add hand parameters here
        },
        left_hand = {
            -- TODO: add hand parameters here
        },
        wings = {
            -- TODO: add wing parameters here
        }
    }
    setmetatable(obj, self)
    self.__index = self
    return obj
end

local animation = {}
function animation:new(keyframes, duration, start_time, playing, loop)
    local obj = {
        keyframes = keyframes or {keyframe:new()},
        duration = duration, -- in seconds
        start_time = start_time,
        playing = playing,
        loop = loop
    }
    setmetatable(obj, self)
    self.__index = self
    return obj
end

-- get current keyframe index and next keyframe index based on current_time
function animation:get_keyframe_indices(current_time, I)
    if #self.keyframes < 2 then
        I:Log("Error: Not enough keyframes to interpolate")
        I:Log("Keyframes count: " .. #self.keyframes)
        return nil, nil
    end

    if current_time < self.keyframes[1].time then
        return 1, 2
    end
    
    if current_time >= self.keyframes[#self.keyframes].time then
        return #self.keyframes - 1, #self.keyframes
    end
    
    for i = 1, #self.keyframes - 1 do
        if current_time >= self.keyframes[i].time and current_time < self.keyframes[i + 1].time then
            return i, i + 1
        end
    end

    return #self.keyframes - 1, #self.keyframes
end

-- interpolate between two keyframes based on current_time
function animation:interpolate_keyframes(current_time, I)
    local kf1_index, kf2_index = self:get_keyframe_indices(current_time, I)
    I:Log("Interpolating between keyframes " .. kf1_index .. " and " .. kf2_index)
    local kf1 = self.keyframes[kf1_index]
    local kf2 = self.keyframes[kf2_index]
    I:Log("kf1 object: " .. tostring(kf1))
    I:Log("kf2 object: " .. tostring(kf2))

    local t = (current_time - kf1.time) / (kf2.time - kf1.time)
    t = Mathf.Clamp(t, 0, 1)

    -- right arm
    local right_arm_rot = Quaternion.SlerpUnclamped(kf1.right_arm.rotation, kf2.right_arm.rotation, kf1.right_arm.easing_function(t))
    local right_arm_pos = Vector3.Lerp(kf1.right_arm.position, kf2.right_arm.position, kf1.right_arm.easing_function(t))
    local right_arm_sew = angle_ease(kf1.right_arm.sew_angle, kf2.right_arm.sew_angle, kf1.right_arm.easing_function, t)

    move_arm_to_target(RIGHT_ARM_PARAMETERS, quaternion_to_rotation_matrix(right_arm_rot), right_arm_pos, right_arm_sew, kf2.right_arm.ik_solution, I)

    -- use SlerpUnclamped instead of slerp becuase slerp has a bug where it changes the value of kf2.left_arm.rotation
    local left_arm_rot = Quaternion.SlerpUnclamped(kf1.left_arm.rotation, kf2.left_arm.rotation, kf1.left_arm.easing_function(t))
    local left_arm_pos = Vector3.Lerp(kf1.left_arm.position, kf2.left_arm.position, kf1.left_arm.easing_function(t))
    local left_arm_sew = angle_ease(kf1.left_arm.sew_angle, kf2.left_arm.sew_angle, kf1.left_arm.easing_function, t)

    move_arm_to_target(LEFT_ARM_PARAMETERS, quaternion_to_rotation_matrix(left_arm_rot), left_arm_pos, left_arm_sew, kf2.left_arm.ik_solution, I)
end

local factorial = {
    [0] = 1,
    [1] = 1,
    [2] = 2,
    [3] = 6,
    [4] = 24
}

-- Bernstein polynomial for Bezier curves
local function bezier(t, points) -- points is a list of Vector3 control points and the start and end points
    local n = #points - 1
    local result = Vector3(0, 0, 0)
    for i = 0, n do
        local binom = factorial[n] / (factorial[i] * factorial[n - i])
        local coeff = binom * (t ^ i) * ((1 - t) ^ (n - i))
        result = result + points[i + 1] * coeff
    end
    return result
end

-- using I:LogToHud("draw"..message, args)
local function draw_animation_points(animation, I)
    local right_arm_points = {}
    local left_arm_points = {}
    for _, kf in ipairs(animation.keyframes) do
        table.insert(right_arm_points, kf.right_arm.position)
        for _, cp in ipairs(kf.right_arm.control_points) do
            table.insert(right_arm_points, cp.position)
        end
        table.insert(left_arm_points, kf.left_arm.position)
        for _, cp in ipairs(kf.left_arm.control_points) do
            table.insert(left_arm_points, cp.position)
        end
    end

    for i = 1, #left_arm_points do
        I:LogToHud("draw sphere ^p 1 color green width 0.1 duration 1", {p = left_arm_points[i]})
    end
end

-- useful FTD interface functions
-- I:GetTime()
-- I:GetTimeSinceSpawn()
-- I:GetGameTime()
-- I:GetNumberOfMainframes()
-- I:GetNumberOfTargets(mainframe_idx)
-- I:GetTargetInfo(mainframe_idx, target_idx)
-- I:GetTargetPositionInfo(mainframe_idx, target_idx)
-- I:GetTargetPositionInfoForPosition(mainframe_idx, x, y, z)

target_info = {
    Priority = 0,
    Score = 0.0,
    AimPointPosition = Vector3(0,0,0),
    Team = 0,
    Protected = false,
    Position = Vector3(0,0,0),
    Velocity = Vector3(0,0,0),
    PlayerTargetChoice = false,
    Id = 0
}

target_position_info = {
    Valid = false,
    Azimuth = 0.0,
    Elevation = 0.0,
    ElivationForAltitudeComponentOnly = 0.0,
    Range = 0.0,
    Direction = Vector3(0,0,0),
    GroundDistance = 0.0,
    AltitudeAboveSeaLevel = 0.0,
    Position = Vector3(0,0,0),
    Velocity = Vector3(0,0,0)
}

function KeyInput(I)
    local keys={
        TKey = I:GetCustomAxis("TKey"),
        GKey = I:GetCustomAxis("GKey")
    }
    return keys
end

local state_list = {
    "idle",
    "punch"
}
local state = state_list[1]

local punch = animation:new(
    {
        keyframe:new{
            time = 0,
            right_arm = {
                position = Vector3(8, 15, 5),
                rotation = Quaternion.Euler(0, 0, 0),
                sew_angle = -2 * math.pi * 0.2,
                easing_function = new_easing_function(1, 1),
                ik_solution = 2,
                control_points = {}
            },
            left_arm = {
                position = Vector3(-10, 15, 5),
                rotation = Quaternion.Euler(-45, 30, 0),
                sew_angle = 2 * math.pi * 0.2,
                easing_function = new_easing_function(1, 1),
                ik_solution = 3,
                control_points = {
                    control_point:new(Vector3(-12, 15, 0), Quaternion.Euler(-30, 30, 0), 2 * math.pi * 0.1),
                    control_point:new(Vector3(-14, 15, -5), Quaternion.Euler(-15, 0, 0), 2 * math.pi * 0.1)
                }
            },
            right_hand = {
                -- TODO: add hand parameters here
            },
            left_hand = {
                -- TODO: add hand parameters here
            },
            wings = {
                -- TODO: add wing parameters here
            }
        },
        keyframe:new{
            time = 5,
            right_arm = {
                position = Vector3(8, 15, 5),
                rotation = Quaternion.Euler(0, 0, 0),
                sew_angle = -2 * math.pi * 0.2,
                easing_function = new_easing_function(1, 1),
                ik_solution = 2,
                control_points = {}
            },
            left_arm = {
                position = Vector3(-3, 15, 21),
                rotation = Quaternion.Euler(-45, 30, 0),
                sew_angle = 2 * math.pi * 0.2,
                easing_function = new_easing_function(1, 1),
                ik_solution = 3,
                control_points = {}
            },
            right_hand = {
                -- TODO: add hand parameters here
            },
            left_hand = {
                -- TODO: add hand parameters here
            },
            wings = {
                -- TODO: add wing parameters here
            }
        }
    },
    5,
    false,
    true
)

function Update(I)
    draw_animation_points(punch, I)

    I:ClearLogs()
    I:Log("punch duration: " .. punch.duration)
    I:Log("number of keyframes: " .. #punch.keyframes)

    local current_time = I:GetTimeSinceSpawn()
    I:Log("Current time: " .. current_time)
    I:Log("Current state: " .. state)

    -- start punch animation on key press
    -- local keys = KeyInput(I)
    -- if state == "idle" and keys.TKey == 1 then
    if state == "idle" then
        state = "punch"
        punch.playing = true
        punch.start_time = current_time
    end

    if state == "punch" then
        local elapsed_time = current_time - punch.start_time
        if elapsed_time > punch.duration then
            if punch.loop then
                punch.start_time = current_time
                elapsed_time = 0
            else
                punch.playing = false
                state = "idle"
            end
        end

        if punch.playing then
            punch:interpolate_keyframes(elapsed_time, I)
        else
            state = "idle"
        end
    end
end



