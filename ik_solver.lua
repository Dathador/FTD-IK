-- 7-DOF Inverse Kinematic Solver using Stereographic SEW Angle and IK-GEO Subproblems
-- Based on "Redundancy parameterization and inverse kinematics of 7-DOF revolute manipulators" by Elias & Wen

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
    local c, s = math.cos(theta), math.sin(theta)
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

-- Compute reference frame for SEW angle using stereographic projection
local function compute_sew_reference_frame(p_sw, e_t, e_r)
    local e_sw = p_sw.normalized
    
    -- Stereographic SEW angle computation
    local k_rt = Vector3.Cross(e_sw - e_t, e_r)
    local k_x = Vector3.Cross(k_rt, p_sw)
    
    if k_x.magnitude < 1e-6 then
        -- Near singularity - use fallback
        I:Log("Warning: Near SEW singularity")
        return e_sw, Vector3(1, 0, 0), Vector3(0, 1, 0)
    end
    
    local e_x = k_x.normalized
    local e_y = Vector3.Cross(e_sw, e_x)
    
    return e_sw, e_x, e_y
end

-- Compute SEW plane normal from given SEW angle
local function compute_sew_plane_normal(psi, e_sw, e_x, e_y)
    -- Using plane definition: n_sew = R(e_sw, psi) * e_y
    local R_psi = rotation_matrix(e_sw, psi)
    return matrix_vector_mult(R_psi, e_y)
end

-- Forward kinematics to get shoulder, elbow, wrist positions
local function get_sew_positions(q)
    -- Shoulder at origin (base frame)
    local p_s = Vector3(0, 0, 0)
    
    -- Compute elbow position (after joints 1, 2, 3)
    local current_pos = Vector3(0, 0, 0)
    local current_rot = IDENTITY_MATRIX
    
    for i = 1, 3 do
        local a, alpha, d, offset = unpack(DH_PARAMS[i])
        local theta = q[i] + offset
        
        -- DH transformation
        local ct, st = math.cos(theta), math.sin(theta)
        local ca, sa = math.cos(alpha), math.sin(alpha)
        
        -- Translation in current frame
        local trans = Vector3(a * ct, a * st, d)
        current_pos = current_pos + matrix_vector_mult(current_rot, trans)
        
        -- Rotation update
        local R_z = {{ct, -st, 0}, {st, ct, 0}, {0, 0, 1}}
        local R_x = {{1, 0, 0}, {0, ca, -sa}, {0, sa, ca}}
        local R_joint = matrix_mult(R_z, R_x)
        current_rot = matrix_mult(current_rot, R_joint)
    end
    local p_e = current_pos
    
    -- Continue to wrist position (through all 7 joints)
    for i = 4, 7 do
        local a, alpha, d, offset = unpack(DH_PARAMS[i])
        local theta = q[i] + offset
        
        local ct, st = math.cos(theta), math.sin(theta)
        local ca, sa = math.cos(alpha), math.sin(alpha)
        
        local trans = Vector3(a * ct, a * st, d)
        current_pos = current_pos + matrix_vector_mult(current_rot, trans)
        
        local R_z = {{ct, -st, 0}, {st, ct, 0}, {0, 0, 1}}
        local R_x = {{1, 0, 0}, {0, ca, -sa}, {0, sa, ca}}
        local R_joint = matrix_mult(R_z, R_x)
        current_rot = matrix_mult(current_rot, R_joint)
    end
    local p_w = current_pos
    
    return p_s, p_e, p_w
end

-- Subproblem 1: Circle and Point
local function subproblem_1(I, p1, p2, k)
    local solutions = {}
    local k_cross_p1 = Vector3.Cross(k, p1)
    local k_cross2_p1 = -Vector3.Cross(k, k_cross_p1)
    
    if k_cross_p1.magnitude < 1e-6 then
        -- p1 is parallel to k, any angle works
        table.insert(solutions, 0)
        return solutions
    end
    
    local theta = math.atan2(Vector3.Dot(k_cross_p1, p2), Vector3.Dot(-k_cross2_p1, p2))
    table.insert(solutions, theta)
    return solutions
end

-- Subproblem 2: Two Circles
local function subproblem_2(I, p1, p2, k1, k2)
    local solutions = {}
    
    -- Project to solve individually
    local theta1_sols = subproblem_4(I, k2, p1, k1)
    local theta2_sols = subproblem_4(I, k1, p2, k2)
    
    -- Pair up solutions (simplified pairing)
    for i, theta1 in ipairs(theta1_sols) do
        if theta2_sols[i] then
            table.insert(solutions, {theta1, theta2_sols[i]})
        end
    end
    
    return solutions
end

-- Subproblem 3: Circle and Sphere
local function subproblem_3(I, p1, p2, r, k)
    local solutions = {}
    
    -- Convert to subproblem 4: solve for theta such that p2^T R(k,theta)p1 = d
    local d = (r*r - p1.magnitude*p1.magnitude - p2.magnitude*p2.magnitude) / 2
    local theta_sols = subproblem_4(I, p2, p1, k, d)
    
    -- Verify solutions actually satisfy the distance constraint
    for _, theta in ipairs(theta_sols) do
        local rotated = matrix_vector_mult(rotation_matrix(k, theta), p1)
        local result_vec = rotated + p2
        if math.abs(result_vec.magnitude - r) < 1e-6 then
            table.insert(solutions, theta)
        end
    end
    
    return solutions
end

-- Subproblem 4: Circle and Plane  
local function subproblem_4(I, h, p, k, d)
    d = d or 0  -- Default to solving h^T R(k,theta)p = 0
    
    local solutions = {}
    local k_norm = k.normalized
    local h_norm = h.normalized
    
    -- Project p onto plane perpendicular to k
    local p_parallel = Vector3.Dot(k_norm, p) * k_norm
    local p_perp = p - p_parallel
    
    if p_perp.magnitude < 1e-6 then
        -- p is parallel to k
        local current_dot = Vector3.Dot(h_norm, p)
        if math.abs(current_dot - d) < 1e-6 then
            table.insert(solutions, 0)  -- Any angle works
        end
        return solutions
    end
    
    -- Solve A*cos(theta) + B*sin(theta) = C
    local p_cross = Vector3.Cross(k_norm, p_perp)
    local A = Vector3.Dot(h_norm, p_perp)  
    local B = Vector3.Dot(h_norm, p_cross)
    local C = d - Vector3.Dot(h_norm, p_parallel)
    
    local R = math.sqrt(A*A + B*B)
    if R < 1e-6 then
        if math.abs(C) < 1e-6 then
            table.insert(solutions, 0)
        end
        return solutions
    end
    
    if math.abs(C) > R + 1e-6 then
        return solutions  -- No solution
    end
    
    local phi = math.atan2(B, A)
    local psi = math.acos(math.max(-1, math.min(1, C / R)))
    
    table.insert(solutions, phi + psi)
    table.insert(solutions, phi - psi)
    
    return solutions
end

-- Solve 3R spherical joint for given rotation matrix
local function solve_spherical_joint(I, R_target, joint_axes)
    local h1, h2, h3 = joint_axes[1], joint_axes[2], joint_axes[3]
    
    -- Using ZYZ Euler angle decomposition
    local r11, r12, r13 = R_target[1][1], R_target[1][2], R_target[1][3]
    local r21, r22, r23 = R_target[2][1], R_target[2][2], R_target[2][3]  
    local r31, r32, r33 = R_target[3][1], R_target[3][2], R_target[3][3]
    
    -- Avoid singularities
    local sy = math.sqrt(r13*r13 + r23*r23)
    
    if sy > 1e-6 then
        local q1 = math.atan2(r23, r13)
        local q2 = math.atan2(sy, r33)
        local q3 = math.atan2(r32, -r31)
        return q1, q2, q3
    else
        -- Gimbal lock case
        local q1 = math.atan2(-r12, r11)
        local q2 = math.atan2(sy, r33)
        local q3 = 0
        return q1, q2, q3
    end
end

-- Main IK solver using SEW parameterization
local function ik_solver_sew(I, target_pos, target_rot, sew_angle, e_t, e_r)
    I:Log("Starting SEW IK solver")
    I:Log("Target position: " .. tostring(target_pos))
    I:Log("SEW angle: " .. tostring(sew_angle))
    
    local solutions = {}
    
    -- Step 1: Compute shoulder-wrist vector
    local p_sw = target_pos  -- Assuming shoulder at origin
    
    if p_sw.magnitude < 1e-6 then
        I:Log("Error: Shoulder and wrist positions are identical")
        return solutions
    end
    
    -- Step 2: Compute SEW reference frame
    local e_sw, e_x, e_y = compute_sew_reference_frame(p_sw, e_t, e_r)
    
    -- Step 3: Compute SEW plane normal from given angle
    local n_sew = compute_sew_plane_normal(sew_angle, e_sw, e_x, e_y)
    
    -- Step 4: Check reachability
    local distance = p_sw.magnitude
    local max_reach = 22  -- dse + dew = 11 + 11
    local min_reach = 0   -- |dse - dew| = |11 - 11|
    
    if distance > max_reach then
        I:Log("Target unreachable: distance " .. distance .. " > max reach " .. max_reach)
        return solutions
    end
    
    -- Step 5: For 7-DOF arm, we need to solve based on kinematic family
    -- This is a general approach for R-2R-2R-2R type robot
    
    -- Use elbow positioning approach
    local elbow_distance = 11  -- dse
    local wrist_distance = 11  -- dew
    
    -- The elbow lies on the intersection of:
    -- 1. Sphere of radius elbow_distance around shoulder
    -- 2. Sphere of radius wrist_distance around wrist  
    -- 3. SEW plane defined by n_sew
    
    -- Find elbow position using geometric constraints
    local elbow_circle_center = p_sw * 0.5  -- Midpoint for equal link lengths
    local elbow_circle_radius = math.sqrt(elbow_distance*elbow_distance - (distance*0.5)*(distance*0.5))
    
    if elbow_circle_radius < 0 then
        I:Log("No geometric solution for elbow position")
        return solutions
    end
    
    -- The elbow lies on circle in plane perpendicular to p_sw
    -- Use SEW constraint to find specific position on this circle
    local temp_dir = Vector3(1, 0, 0)
    if math.abs(Vector3.Dot(e_sw, temp_dir)) > 0.9 then
        temp_dir = Vector3(0, 1, 0)
    end
    local circle_u = Vector3.Cross(e_sw, temp_dir).normalized
    local circle_v = Vector3.Cross(e_sw, circle_u)
    
    -- Project n_sew onto elbow circle plane
    local n_sew_proj = n_sew - Vector3.Dot(n_sew, e_sw) * e_sw
    if n_sew_proj.magnitude > 1e-6 then
        n_sew_proj = n_sew_proj.normalized
    else
        I:Log("SEW plane contains shoulder-wrist line - using default")
        n_sew_proj = circle_u
    end
    
    -- Elbow position on the circle
    local p_se = elbow_circle_center + elbow_circle_radius * n_sew_proj
    
    I:Log("Computed elbow position: " .. tostring(p_se))
    
    -- Step 6: Solve for joint angles
    -- This is simplified - full implementation would depend on exact robot kinematic family
    
    -- Solve shoulder joints (1,2,3) to reach elbow
    local shoulder_target = p_se
    local shoulder_distance = shoulder_target.magnitude
    
    if shoulder_distance > elbow_distance then
        I:Log("Elbow position unreachable from shoulder")
        return solutions
    end
    
    -- Use subproblem approach for spherical shoulder
    -- This is a simplified version - exact implementation depends on joint axes
    local q1 = math.atan2(shoulder_target.y, shoulder_target.x)
    local q2 = math.acos(math.max(-1, math.min(1, shoulder_target.z / shoulder_distance)))  
    local q3 = 0  -- Simplified
    
    -- Solve elbow joint (4) 
    local elbow_to_wrist = p_sw - p_se
    local elbow_angle = math.atan2(elbow_to_wrist.y, elbow_to_wrist.x)  -- Simplified
    local q4 = elbow_angle
    
    -- Solve wrist joints (5,6,7) for orientation
    -- Use spherical wrist solution
    local wrist_rot = target_rot  -- Simplified - should account for arm orientation
    local q5, q6, q7 = solve_spherical_joint(I, wrist_rot, {JOINT_AXES[5], JOINT_AXES[6], JOINT_AXES[7]})
    
    if q5 and q6 and q7 then
        local solution = {q1, q2, q3, q4, q5, q6, q7}
        table.insert(solutions, solution)
        I:Log("Found solution: " .. table.concat(solution, ", "))
    end
    
    return solutions
end

-- Constants and parameters
IDENTITY_MATRIX = {
    {1, 0, 0},
    {0, 1, 0}, 
    {0, 0, 1}
}

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
            I:SetSpinBlockRotationAngle(scid, joint_angles[i])
        end
    end
end

-- Main update function
local iteration = 0
function Update(I)
    local custom_joint_names = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"}

    if iteration == 0 then
        -- Test with reachable target
        local target_pos = Vector3(15, 8, 10)    -- Within reach but not trivial
        local target_rot = IDENTITY_MATRIX        -- No rotation
        local sew_angle = math.pi/6              -- 30 degrees SEW angle
        
        I:Log("=== Starting SEW IK Test ===")
        I:Log("Target: " .. tostring(target_pos))
        I:Log("SEW angle: " .. tostring(sew_angle * 180/math.pi) .. " degrees")
        
        local solutions = ik_solver_sew(I, target_pos, target_rot, sew_angle, E_T, E_R)
        
        if #solutions > 0 then
            local joints = get_joint_list(I, custom_joint_names)
            if #joints == 7 then
                set_joint_angles(I, joints, solutions[1])
                I:Log("Successfully applied IK solution!")
                
                -- Verify the solution by computing forward kinematics
                local p_s, p_e, p_w = get_sew_positions(solutions[1])
                I:Log("Computed wrist position: " .. tostring(p_w))
                I:Log("Position error: " .. tostring((p_w - target_pos).magnitude))
            else
                I:Log("Error: Could not find all 7 joints")
            end
        else
            I:Log("No IK solution found")
        end
    end

    iteration = iteration + 1
    if iteration > 20 then
        iteration = 0
    end
end