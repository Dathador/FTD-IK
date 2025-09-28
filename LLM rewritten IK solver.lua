-- LLM rewritten IK solver

-- 7-DOF Inverse Kinematic Solver - Left-Handed Coordinate System Consistent
-- Based on IK-GEO algorithm, adapted for left-handed coordinates

-- Matrix class (unchanged from your implementation)
local Matrix = {}
Matrix.__typename = "Matrix"

function Matrix:new(data)
    local obj = {}
    obj.rows = #data
    obj.cols = #data[1] or 1
    
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

    if other.__typename == "Vector3" then
        if self.rows == 3 then
            return Vector3(
                self[1][1] * other.x + self[1][2] * other.y + self[1][3] * other.z,
                self[2][1] * other.x + self[2][2] * other.y + self[2][3] * other.z,
                self[3][1] * other.x + self[3][2] * other.y + self[3][3] * other.z
            )
        else
            local result = {}
            for i = 1, self.rows do
                result[i] = {}
                result[i][1] = self[i][1] * other.x + self[i][2] * other.y + self[i][3] * other.z
            end
            return Matrix:new(result)
        end
    end

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

local function quaternion_to_rotation_matrix(q)
    local w, x, y, z = q.w, q.x, q.y, q.z
    local norm = math.sqrt(w*w + x*x + y*y + z*z)
    w, x, y, z = w/norm, x/norm, y/norm, z/norm
    
    return Matrix:new({
        {1 - 2*y*y - 2*z*z, 2*x*y - 2*w*z, 2*x*z + 2*w*y},
        {2*x*y + 2*w*z, 1 - 2*x*x - 2*z*z, 2*y*z - 2*w*x},
        {2*x*z - 2*w*y, 2*y*z + 2*w*x, 1 - 2*x*x - 2*y*y}
    })
end

local function rot(axis, theta)
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

local function transpose(m)
    if m.__typename == "Vector3" then
        return Matrix:new({{m.x, m.y, m.z}})
    end

    local result = {}
    for j = 1, m.cols do
        result[j] = {}
        for i = 1, m.rows do
            result[j][i] = m[i][j]
        end
    end
    return Matrix:new(result)
end

local function norm_squared(v)
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

-- Robot parameters in LEFT-HANDED coordinates (x=right, y=up, z=forward)
CUSTOM_JOINT_NAMES = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"}

-- CRITICAL: Measure these in the game's left-handed coordinate system
local dse = 11  -- shoulder to elbow distance
local dew = 11  -- elbow to wrist distance  
local dwt = 3   -- wrist to tool distance

-- UPDATED: KIN parameters in left-handed coordinates
-- You need to verify these measurements match your actual robot
local KIN = {
    P = {
        -- Position vectors between joints (left-handed: x=right, y=up, z=forward)
        Vector3(8, 20, 0),   -- Base to Shoulder (joint 1)
        Vector3(0, 0, 0),    -- Shoulder to joint 2
        Vector3(0, 0, 0),    -- joint 2 to joint 3
        Vector3(0,-dse, 0),  -- joint 3 to joint 4 (elbow)
        Vector3(0, 0, dew),    -- joint 4 to joint 5
        Vector3(0, 0, 0),  -- joint 5 to joint 6 (wrist)
        Vector3(0, 0, 0),    -- joint 6 to joint 7
        Vector3(0, 0, dwt)   -- joint 7 to end effector
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

-- Stereographic SEW parameters (left-handed)
local E_T = Vector3(0, -1, 0)    -- singularity direction
local E_R = Vector3(1, 0, 0)     -- reference direction

-- Subproblems (unchanged - these are coordinate system agnostic)
local function subproblem_4(h, p, k, d)
    local A_11 = Vector3.Cross(k, p)
    local A_11m = Vector3.Cross(A_11, k)
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

    if norm_A_2 > b * b then
        local xi = math.sqrt(norm_A_2 - b * b)
        local x_N_prime_tilde = Matrix:new({
            {A[1][2]}, 
            {-A[1][1]}
        })

        local sc_1 = x_ls_tilde + x_N_prime_tilde * xi
        local sc_2 = x_ls_tilde - x_N_prime_tilde * xi

        theta = {math.atan(sc_1[1][1], sc_1[2][1]), math.atan(sc_2[1][1], sc_2[2][1])}
        is_ls = false
    else
        theta = {math.atan(x_ls_tilde[1][1], x_ls_tilde[2][1])}
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

    if #theta1 > 1 or #theta2 > 1 then
        theta1 = {theta1[1], theta1[#theta1]}
        theta2 = {theta2[#theta2], theta2[1]}
    end

    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-8 or theta1_is_ls or theta2_is_ls
    return theta1, theta2, is_ls
end

local function subproblem_1(p1, p2, k)
    local kxp = Vector3.Cross(k, p1)
    local kxm = Vector3.Cross(kxp, k)
    local A = Matrix:new{
        {kxp.x, kxm.x},
        {kxp.y, kxm.y},
        {kxp.z, kxm.z}
    }

    local x = transpose(A) * p2
    local theta = math.atan(x[1][1], x[1][2])
    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-6 or math.abs(Vector3.Dot(k, p1) - Vector3.Dot(k, p2)) > 1e-6

    return theta, is_ls
end

-- SEW parameterization
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

-- CORRECTED: IK solver with proper error handling
local function IK_3R_R_3R(R_07, p_0T, psi, kin)
    local Q = {}
    local is_LS_vec = {}

    -- Find wrist position
    local R_07_times_P8 = R_07 * kin.P[8]
    local W = p_0T - R_07_times_P8

    -- Find shoulder position  
    local S = kin.P[1]
    local p_SW = W - S
    
    -- Check reachability
    local reach_dist = p_SW.magnitude
    local max_reach = dse + dew
    if reach_dist > max_reach * 1.01 then  -- Small tolerance
        return Q, is_LS_vec  -- Return empty if unreachable
    end

    local e_SW = p_SW.normalized
    local e_CE, n_SEW = inv_kin(S, W, psi)

    -- Solve for q4 using subproblem 3
    local t4, t4_is_LS = subproblem_3(kin.P[5], -kin.P[4], kin.H[4], reach_dist)
    
    if not t4 or #t4 == 0 then
        return Q, is_LS_vec
    end

    for i_4 = 1, #t4 do
        local q4 = t4[i_4]

        -- Solve for theta_b, theta_c using subproblem 2
        local t_b, t_c, t_bc_is_LS = subproblem_2(p_SW, kin.P[4] + rot(kin.H[4], q4)*kin.P[5], -n_SEW, e_SW)
        
        if not t_b or #t_b == 0 then
            goto continue_q4
        end

        local theta_b = t_b[1]
        local theta_c = t_c[1]

        -- Solve for theta_a using subproblem 4
        local t_a, t_a_is_LS = subproblem_4(n_SEW, rot(n_SEW, theta_b)*rot(e_SW, theta_c)*kin.P[4], e_SW, 0)
        
        if not t_a or #t_a == 0 then
            goto continue_q4
        end

        for i_a = 1, #t_a do
            local theta_a = t_a[i_a]

            local R_03 = rot(e_SW, theta_a)*rot(n_SEW, theta_b)*rot(e_SW, theta_c)
            
            -- Check elbow constraint
            if Vector3.Dot(e_CE, R_03 * kin.P[4]) < 0 then
                goto continue_theta_a
            end

            -- Solve for q1, q2, q3
            local p_spherical_1 = kin.H[2]
            local t2, t1, t12_is_LS = subproblem_2(kin.H[3], R_03*kin.H[3], kin.H[2], -kin.H[1])
            
            if not t1 or #t1 == 0 then
                goto continue_theta_a
            end

            for i_12 = 1, #t1 do
                local q1 = t1[i_12] 
                local q2 = t2[i_12]
                local q3, q3_is_LS = subproblem_1(p_spherical_1, transpose(rot(kin.H[1], q1) * rot(kin.H[2], q2)) * R_03 * p_spherical_1, kin.H[3])

                -- Solve for q5, q6, q7
                local R_01 = rot(kin.H[1], q1)
                local R_12 = rot(kin.H[2], q2) 
                local R_23 = rot(kin.H[3], q3)
                local R_34 = rot(kin.H[4], q4)
                local R_47 = transpose(R_01 * R_12 * R_23 * R_34) * R_07
                
                local p_spherical_2 = kin.H[6]
                local t6, t5, t56_is_LS = subproblem_2(kin.H[7], R_47*kin.H[7], kin.H[6], -kin.H[5])
                
                if not t5 or #t5 == 0 then
                    goto continue_q12
                end

                for i_56 = 1, #t5 do
                    local q5 = t5[i_56]
                    local q6 = t6[i_56] 
                    local q7, q7_is_LS = subproblem_1(p_spherical_2, transpose(rot(kin.H[5], q5) * rot(kin.H[6], q6)) * R_47 * p_spherical_2, kin.H[7])
                    
                    local q_i = {q1, q2, q3, q4, q5, q6, q7}
                    table.insert(Q, q_i)
                    table.insert(is_LS_vec, t4_is_LS or t_bc_is_LS or t_a_is_LS or t12_is_LS or q3_is_LS or t56_is_LS or q7_is_LS)
                end
                ::continue_q12::
            end
            ::continue_theta_a::
        end
        ::continue_q4::
    end
    
    return Q, is_LS_vec
end

-- CORRECTED: Forward kinematics that matches game's coordinate system
local function compute_forward_kinematics(joint_angles, kin)
    local positions = {}
    local rotations = {}
    
    -- Start at origin in left-handed coordinates
    positions[0] = Vector3(0, 0, 0) 
    rotations[0] = Matrix.Identity
    
    for i = 1, #joint_angles do
        -- Rotation for this joint
        local R_i = rot(kin.H[i], joint_angles[i])
        
        -- Cumulative rotation up to joint i
        rotations[i] = rotations[i-1] * R_i
        
        -- Position of joint i
        positions[i] = positions[i-1] + rotations[i-1] * kin.P[i]
    end
    
    -- End effector position
    local end_effector_pos = positions[#joint_angles] + rotations[#joint_angles] * kin.P[#kin.P]
    
    return positions, rotations, end_effector_pos
end

-- Game interface functions
local function get_joint_list(joint_names, I)
    local joints = {}
    for i = 0, I:GetAllSubconstructsCount() - 1 do
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

local function set_joint_angles(joints, joint_angles, I)
    if not joint_angles then
        I:Log("Error: No joint angles provided")
        return
    end
    
    for i, scid in ipairs(joints) do
        if joint_angles[i] then
            -- Convert radians to degrees
            -- NO NEGATION - stay in left-handed system consistently
            local angle_degrees = joint_angles[i] * 180 / math.pi
            I:SetSpinBlockRotationAngle(scid, angle_degrees)
        end
    end
end

-- CORRECTED: Get end effector pose in consistent left-handed coordinates
local function get_end_effector_pose(joints, I)
    local position = Vector3(0, 0, 0)
    local rotation = Quaternion.identity
    
    for i, scid in ipairs(joints) do
        local BlockInfo = I:GetSubConstructInfo(scid)
        position = position + rotation * BlockInfo.LocalPosition
        rotation = rotation * BlockInfo.LocalRotation
    end

    position = position + rotation * KIN.P[8]  -- Add end effector offset
    -- Stay in left-handed - no coordinate conversion
    return position, quaternion_to_rotation_matrix(rotation)
end

local function test_forward_kinematics_consistency(joint_angles, I)
    I:Log("=== Forward Kinematics Consistency Test ===")
    
    local joints = get_joint_list(CUSTOM_JOINT_NAMES, I)
    if not joints or #joints == 0 then
        I:Log("No joints found")
        return
    end
    
    -- Test with a random pose
    set_joint_angles(joints, joint_angles, I)

    -- Get actual position from game
    local actual_pos, actual_rot = get_end_effector_pose(joints, I)
    
    -- Calculate expected position using forward kinematics
    local calc_positions, calc_rotations, calc_end_effector = compute_forward_kinematics(joint_angles, KIN)

    I:Log("With all joints at random angles:")
    I:Log(string.format("Actual end effector: (%.3f, %.3f, %.3f)", 
          actual_pos.x, actual_pos.y, actual_pos.z))
    I:Log(string.format("Calculated end effector: (%.3f, %.3f, %.3f)", 
          calc_end_effector.x, calc_end_effector.y, calc_end_effector.z))
    
    local error = (actual_pos - calc_end_effector).magnitude
    I:Log(string.format("Error magnitude: %.3f", error))
    
    if error < 0.5 then
        I:Log("✓ Good match - KIN parameters likely correct")
    else
        I:Log("✗ Large error - check KIN.P (link lengths) and KIN.H (joint axes)")
    end
end

local joint_angles = {math.random() * 2 * math.pi, math.random() * 2 * math.pi, math.random() * 2 * math.pi,
                    math.random() * 2 * math.pi,
                    math.random() * 2 * math.pi, math.random() * 2 * math.pi, math.random() * 2 * math.pi}

function Update(I)
    -- local joints = get_joint_list(CUSTOM_JOINT_NAMES, I)
    -- test_forward_kinematics_consistency(joint_angles, I)

    -- Target in LEFT-HANDED coordinates (x=right, y=up, z=forward)
    local target_pos = Vector3(15, 10, 5)  -- Example target
    local target_rot = Matrix.Identity
    local psi = math.pi/4

    local Q, is_ls_vec = IK_3R_R_3R(target_rot, target_pos, psi, KIN)
    
    if not Q or #Q == 0 then
        I:Log("No IK solution found")
        return
    end

    local joints = get_joint_list(CUSTOM_JOINT_NAMES, I)
    
    -- Set first solution
    set_joint_angles(joints, Q[1], I)
    
    -- Log new position after a short delay to allow physics update
    position, rotation = get_end_effector_pose(joints, I)
    I:Log(string.format("Target Position: (%.3f, %.3f, %.3f)", target_pos.x, target_pos.y, target_pos.z))
    I:Log(string.format("Actual Position: (%.3f, %.3f, %.3f)", position.x, position.y, position.z))

    -- calculate expected position using forward kinematics
    local calc_positions, calc_rotations, calc_end_effector = compute_forward_kinematics(Q[1], KIN)
    I:Log(string.format("Calculated Position: (%.3f, %.3f, %.3f)", 
          calc_end_effector.x, calc_end_effector.y, calc_end_effector.z))
    
    local error = (target_pos - position).magnitude
    I:Log(string.format("Position Error: %.3f", error))
end