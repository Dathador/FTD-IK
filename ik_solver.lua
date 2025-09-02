-- Inverse Kinematic Solver for 7R Humanoid Arm using IK-GEO Algorithm

-- Matrix operations
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

-- rotation matrix from quaternion
local function quaternionToMatrix(q)
    local w, x, y, z = q[1], q[2], q[3], q[4]
    
    -- Precompute terms for efficiency
    local xx = x * x
    local yy = y * y
    local zz = z * z
    local xy = x * y
    local xz = x * z
    local yz = y * z
    local wx = w * x
    local wy = w * y
    local wz = w * z

    -- Construct the rotation matrix
    local matrix = {
        {1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)},
        {2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)},
        {2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)}
    }

    return matrix
end

-- quaternion from rotation matrix
local function matrixToQuaternion(R)
    local r00, r01, r02 = R[1][1], R[1][2], R[1][3]
    local r10, r11, r12 = R[2][1], R[2][2], R[2][3]
    local r20, r21, r22 = R[3][1], R[3][2], R[3][3]

    -- Calculate the trace of the matrix
    local trace = r00 + r11 + r22

    local w, x, y, z

    if trace > 0 then
        -- Positive trace, straightforward case
        local s = math.sqrt(1 + trace) * 2
        w = s / 4
        x = (r21 - r12) / s
        y = (r02 - r20) / s
        z = (r10 - r01) / s
    else
        -- Handle case where the trace is non-positive
        if r00 > r11 and r00 > r22 then
            local s = math.sqrt(1 + r00 - r11 - r22) * 2
            w = (r21 - r12) / s
            x = s / 4
            y = (r01 + r10) / s
            z = (r02 + r20) / s
        elseif r11 > r22 then
            local s = math.sqrt(1 + r11 - r00 - r22) * 2
            w = (r02 - r20) / s
            x = (r01 + r10) / s
            y = s / 4
            z = (r12 + r21) / s
        else
            local s = math.sqrt(1 + r22 - r00 - r11) * 2
            w = (r10 - r01) / s
            x = (r02 + r20) / s
            y = (r12 + r21) / s
            z = s / 4
        end
    end

    return {w, x, y, z}
end

-- Forward kinematics to get p_se (frame 3 origin), p_sw (frame 7 origin)
local function get_p_se_p_sw(q)
    local current_T = IDENTITY_MATRIX
    local current_p = Vector3(0, 0, 0)
    local p_se
    for i = 1, 7 do
        local a, alpha, d, offset = unpack(DH_PARAMS[i])
        local theta = q[i] + offset
        local cos_theta = math.cos(theta)
        local sin_theta = math.sin(theta)
        local cos_alpha = math.cos(alpha)
        local sin_alpha = math.sin(alpha)
        local A = {
            {cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha},
            {sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha},
            {0, sin_alpha, cos_alpha}
        }
        local trans = Vector3(a * cos_theta, a * sin_theta, d)
        current_p = current_p + matrix_vector_mult(current_T, trans)
        current_T = matrix_mult(current_T, A)
        if i == 3 then
            p_se = current_p
        end
    end
    local p_sw = current_p
    return p_se, p_sw
end

-- Compute reference frame and psi
local function compute_reference_frame(p_sw)
    local e_t = Vector3(1, 0, 0)
    local e_r = Vector3(0, 0, -1)
    local e_sw = p_sw.normalized
    local k_rt = Vector3.Cross(e_sw - e_t, e_r)
    local k_x = Vector3.Cross(k_rt, p_sw)
    local e_x = k_x.normalized
    local e_y = Vector3.Cross(e_sw, e_x)
    return e_sw, e_x, e_y
end

-- Subproblem 2: Solve for (theta1, theta2) such that R(n, theta1)^T p = R(e, theta2) q
local function subproblem_2(I, p, q, n, e)
    I:Log("subproblem_2 called")
    local solutions = {}
    local p_prime = matrix_vector_mult(matrix_transpose(rotation_matrix(n, 0)), p)
    local q_norm = q.magnitude
    local p_prime_norm = p_prime.magnitude
    local e_norm = e.magnitude
    if math.abs(Vector3.Dot(p_prime, e) / (p_prime_norm * e_norm)) <= 1 then
        local theta2 = math.acos(Vector3.Dot(p_prime, e) / (p_prime_norm * e_norm))
        local theta1 = math.atan(Vector3.Dot(Vector3.Cross(e, p_prime), n), Vector3.Dot(e, p_prime))
        table.insert(solutions, {theta1, theta2})
        table.insert(solutions, {-theta1, -theta2})
    end
    if #solutions == 0 then
        I:Log("SP2 No solutions found")
    else
        I:Log("SP2 Solutions found: " .. table.concat(solutions, ", "))
    end
    I:Log("end of subproblem_2")
    return solutions
end

-- Subproblem 3: Solve for theta such that ||R(axis, theta)p + q|| = r
local function subproblem_3(I, p, q, r, axis)
    I:Log("subproblem_3 called")
    local solutions = {}
    local u = axis.normalized
    local beta = Vector3.Dot(Vector3.Cross(u, p), q)
    local alpha = Vector3.Dot(p, q) - Vector3.Dot(u, p) * Vector3.Dot(u, q)
    local gamma = Vector3.Dot(u, p) * Vector3.Dot(u, q)
    local pp = (p.magnitude)^2
    local qq = (q.magnitude)^2
    local k = (r^2 - pp - qq) /2
    local d = k - gamma
    local aa = alpha^2 + beta^2
    if aa == 0 then
        if math.abs(d) < 1e-6 then
            -- Infinite solutions; return a representative
            table.insert(solutions, 0)
        end
        I:Log(solutions[1] or "No solutions")
        return solutions
    end
    local rr = math.sqrt(aa)
    if math.abs(d) > rr + 1e-6 then
        return solutions
    end
    local phi = math.atan(beta, alpha)
    local psi = math.acos(d / rr)
    table.insert(solutions, phi + psi)
    table.insert(solutions, phi - psi)
    return solutions
end

-- Subproblem 4: Solve for theta such that n^T R(axis, theta) p = 0
local function subproblem_4(I, n, p, axis)
    I:Log("subproblem_4 called")
    local solutions = {}
    local p_prime = Vector3.Cross(axis, p)
    if p_prime.magnitude > 1e-6 then
        local theta = math.atan(Vector3.Dot(n, p_prime), Vector3.Dot(n, p))
        table.insert(solutions, theta)
        table.insert(solutions, theta + math.pi)
    end
    if #solutions == 0 then
        I:Log("SP4 No solutions found")
    else
        I:Log("SP4 Solutions found: " .. table.concat(solutions, ", "))
    end
    I:Log("end of subproblem_4")
    return solutions
end

-- Solve spherical joint (3R) for angles q1, q2, q3 given rotation matrix R
local function solve_spherical_joint(I, R, axes)
    I:Log("solve_spherical_joint called")
    local p = matrix_vector_mult(R, axes[3])
    local q1_sols = subproblem_4(I, axes[2], p, axes[1])
    for _, q1 in ipairs(q1_sols) do
        local R01 = rotation_matrix(axes[1], q1)
        local p_prime = matrix_vector_mult(matrix_transpose(R01), p)
        local q2_sols = subproblem_4(I, axes[3], p_prime, axes[2])
        for _, q2 in ipairs(q2_sols) do
            local R12 = rotation_matrix(axes[2], q2)
            local R02 = matrix_mult(R01, R12)
            local p_prime2 = matrix_vector_mult(matrix_transpose(R02), p)
            local q3 = math.atan(p_prime2.y, p_prime2.x)
            I:Log("inserted solution " .. tostring({q1, q2, q3}))
            return q1, q2, q3  -- Return one solution; can be extended for all
        end
    end
    I:Log("No solution found in solve_spherical_joint")
    return nil, nil, nil
end

-- Main IK solver with optimization for reference joints
local function ik_solver(I, target_pos, target_rot, q_ref)
    local p_sw = target_pos  -- Assuming shoulder at (0,0,0)

    local p_07 = p_sw

    local psi = 0  -- Default

    local e_sw, e_x, e_y = compute_reference_frame(p_sw)

    if q_ref then
        local p_se_ref, p_sw_ref = get_p_se_p_sw(q_ref)
        local e_sw_ref, e_x_ref, e_y_ref = compute_reference_frame(p_sw_ref)
        local n_sew_ref = (Vector3.Cross(p_se_ref, p_sw_ref)).normalized
        psi = math.atan(-Vector3.Dot(e_x_ref, n_sew_ref), Vector3.Dot(e_y_ref, n_sew_ref))
    end

    -- Compute n_sew using psi
    local R_psi = rotation_matrix(e_sw, psi)
    local n_sew = matrix_vector_mult(R_psi, e_y)

    -- Define constants from DH parameters
    local p_34 = Vector3(0, 0, 0)
    local p_45 = Vector3(0, 0, 11)

    -- Step 1: Solve for q4 using Subproblem 3
    local R_34 = rotation_matrix(JOINT_AXES[4], 0)
    local p_45_rot = matrix_vector_mult(R_34, p_45)
    local q4_sols = subproblem_3(I, p_45_rot, p_34, p_07.magnitude, JOINT_AXES[4])

    local solutions = {}
    for _, q4 in ipairs(q4_sols) do
        R_34 = rotation_matrix(JOINT_AXES[4], q4)
        p_45_rot = matrix_vector_mult(R_34, p_45)
        local p_34_plus_p45 = p_34 + p_45_rot
        
        -- Solve for theta_b, theta_c using Subproblem 2
        local theta_bc_sols = subproblem_2(I, p_07, p_34_plus_p45, n_sew, e_sw)
        for _, bc in ipairs(theta_bc_sols) do
            local theta_b, theta_c = bc[1], bc[2]
            
            -- Solve for theta_a using Subproblem 4
            local p_3e = Vector3(0, 0, 1)  -- Elbow position relative to joint 3
            local R_bc = matrix_mult(rotation_matrix(n_sew, theta_b), rotation_matrix(e_sw, theta_c))
            local p_3e_rot = matrix_vector_mult(R_bc, p_3e)
            local theta_a_sols = subproblem_4(I, n_sew, p_3e_rot, e_sw)
            
            for _, theta_a in ipairs(theta_a_sols) do
                -- Compute R_03
                local R_03 = matrix_mult(matrix_mult(rotation_matrix(e_sw, theta_a), rotation_matrix(n_sew, theta_b)), rotation_matrix(e_sw, theta_c))
                
                -- Step 3: Solve spherical shoulder (q1, q2, q3)
                local q1, q2, q3 = solve_spherical_joint(I, R_03, {JOINT_AXES[1], JOINT_AXES[2], JOINT_AXES[3]})
                if q1 then
                    -- Step 4: Solve wrist (q5, q6, q7)
                    local R_01 = rotation_matrix(JOINT_AXES[1], q1)
                    local R_12 = rotation_matrix(JOINT_AXES[2], q2)
                    local R_23 = rotation_matrix(JOINT_AXES[3], q3)
                    local R_03_comp = matrix_mult(matrix_mult(R_01, R_12), R_23)
                    local R_04 = matrix_mult(R_03_comp, R_34)
                    local R_47 = matrix_mult(matrix_transpose(R_04), target_rot)
                    local q5, q6, q7 = solve_spherical_joint(I, R_47, {JOINT_AXES[5], JOINT_AXES[6], JOINT_AXES[7]})
                    if q5 then
                        I:Log("inserted solution " .. tostring({q1, q2, q3, q4, q5, q6, q7}))
                        table.insert(solutions, {q1, q2, q3, q4, q5, q6, q7})
                    end
                end
            end
        end
    end
    
    if not q_ref or #solutions == 0 then
        I:Log("No IK solution found")
        return solutions
    end
    
    -- Select the solution with minimal loss to q_ref
    local min_loss = math.huge
    local best_sol
    local function angle_diff(a, b)
        local diff = a - b
        diff = diff - 2 * math.pi * math.floor((diff + math.pi) / (2 * math.pi))
        return diff
    end
    for _, sol in ipairs(solutions) do
        local loss = 0
        for i = 1, 7 do
            local d = angle_diff(sol[i], q_ref[i])
            loss = loss + d * d
        end
        if loss < min_loss then
            min_loss = loss
            best_sol = sol
        end
    end
    return {best_sol}
end

-- loop through all the subconstructs and find the specified joints
-- returns a list of joint identifiers in the same order as the input joint names
local function get_joint_list(I, joint_names)
    local joints = {} -- list of the SubConstructIdentifier(s) for the specified joint names
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

-- takes in a list of SubConstructIdentifiers and a list of joint angles
local function set_joint_angles(I, joints, joint_angles)
    for i, scid in ipairs(joints) do
        I:SetSpinBlockRotationAngle(scid, joint_angles[i])
    end
end

IDENTITY_MATRIX = {
    {1, 0, 0},
    {0, 1, 0},
    {0, 0, 1}
}


-- DH Parameters: {a, alpha, d, theta_offset}
local dse = 11 -- distance from shoulder to elbow
local dew = 11 -- distance from elbow to wrist
local dwt = 3 -- distance from wrist to end effector

DH_PARAMS = {
    {0, -math.pi, 0, math.pi},
    {0, math.pi, 0, math.pi},
    {0, -math.pi, dse, -math.pi},
    {0, math.pi, 0, -math.pi},
    {0, -math.pi, dew, 0},
    {0, math.pi, 0, -math.pi},
    {dwt, 0, 0, 0}
}

-- Joint axes in base frame at home position
JOINT_AXES = {
    Vector3(1, 0, 0),  -- Joint 1
    Vector3(0, 0, 1),  -- Joint 2
    Vector3(0, -1, 0), -- Joint 3
    Vector3(-1, 0, 0), -- Joint 4
    Vector3(0, 0, 1),  -- Joint 5
    Vector3(-1, 0, 0), -- Joint 6
    Vector3(0, 1, 0)   -- Joint 7
}

local iteration = 0
function Update(I)
    local custom_joint_names = {"j1", "j2", "j3", "j4", "j5", "j6", "j7"}

    if iteration == 0 then
        local target_pos = Vector3(0, 12, -12) -- Example: home position of joint 7
        local target_rot = IDENTITY_MATRIX -- Identity rotation
        local q_ref = {0, 0, 0, 0, 0, 0, 0} -- Target/reference joints
        local solutions = ik_solver(I, target_pos, target_rot, q_ref)
        local joints = get_joint_list(I, custom_joint_names)
        set_joint_angles(I, joints, solutions[1])
    end

    iteration = iteration + 1
    if iteration > 20 then
        iteration = 0
    end
end