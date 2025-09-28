-- inverse kinematics testing
-- Comprehensive debugging version of IK solver
-- This version adds extensive logging to identify where the algorithm fails
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

-- Enhanced subproblem functions with debugging
local function subproblem_4_debug(h, p, k, d, name, I)
    I:Log(string.format("=== Subproblem 4 Debug: %s ===", name or "unnamed"))
    I:Log(string.format("h: %s", tostring(h)))
    I:Log(string.format("p: %s", tostring(p)))  
    I:Log(string.format("k: %s", tostring(k)))
    I:Log(string.format("d: %.6f", d))
    
    local A_11 = Vector3.Cross(k, p)
    local A_11m = Vector3.Cross(A_11, k)
    
    I:Log(string.format("A_11 (k×p): %s", tostring(A_11)))
    I:Log(string.format("A_11m (A_11×k): %s", tostring(A_11m)))
    
    local A_1 = Matrix:new({
        {A_11.x, A_11m.x},
        {A_11.y, A_11m.y},
        {A_11.z, A_11m.z}
    })

    local A = transpose(h) * A_1
    local b = d - Vector3.Dot(h, k) * (Vector3.Dot(k, p))
    local norm_A_2 = norm_squared(A)
    
    I:Log(string.format("A matrix: [%.4f, %.4f]", A[1][1], A[1][2]))
    I:Log(string.format("b: %.6f", b))
    I:Log(string.format("||A||²: %.6f", norm_A_2))

    local x_ls_tilde = transpose(A_1) * (h * b)
    I:Log(string.format("x_ls_tilde: [%.4f, %.4f]", x_ls_tilde[1][1], x_ls_tilde[2][1]))

    local theta, is_ls

    if norm_A_2 > b * b then
        I:Log("Multiple solutions case")
        local xi = math.sqrt(norm_A_2 - b * b)
        I:Log(string.format("xi: %.6f", xi))
            
        local x_N_prime_tilde = Matrix:new({
            {A[1][2]}, 
            {-A[1][1]}
        })

        local sc_1 = x_ls_tilde + x_N_prime_tilde * xi
        local sc_2 = x_ls_tilde - x_N_prime_tilde * xi

        theta = {math.atan(sc_1[1][1], sc_1[2][1]), math.atan(sc_2[1][1], sc_2[2][1])}
        is_ls = false
        
        I:Log(string.format("Solution 1: %.4f rad (%.1f deg)", theta[1], theta[1]*180/math.pi))
        I:Log(string.format("Solution 2: %.4f rad (%.1f deg)", theta[2], theta[2]*180/math.pi))
    else
        I:Log("Least squares solution case")
        theta = {math.atan(x_ls_tilde[1][1], x_ls_tilde[2][1])}
        is_ls = true
        
        I:Log(string.format("LS Solution: %.4f rad (%.1f deg)", theta[1], theta[1]*180/math.pi))
    end

    I:Log(string.format("is_ls: %s", tostring(is_ls)))
    I:Log("")
    return theta, is_ls
end

local function subproblem_3_debug(p1, p2, k, d, name, I)
    I:Log(string.format("=== Subproblem 3 Debug: %s ===", name or "unnamed"))
    I:Log(string.format("p1: %s", tostring(p1)))
    I:Log(string.format("p2: %s", tostring(p2)))
    I:Log(string.format("k: %s", tostring(k)))
    I:Log(string.format("d: %.6f", d))
    
    local h_val = 1/2 * (Vector3.Dot(p1, p1) + Vector3.Dot(p2, p2) - d * d)
    I:Log(string.format("Calculated h for subproblem 4: %.6f", h_val))
    
    return subproblem_4_debug(p2, p1, k, h_val, name .. "_inner", I)
end

local function subproblem_2_debug(p1, p2, k1, k2, name, I)
    I:Log(string.format("=== Subproblem 2 Debug: %s ===", name or "unnamed"))
    I:Log(string.format("p1: %s (mag: %.4f)", tostring(p1), p1.magnitude))
    I:Log(string.format("p2: %s (mag: %.4f)", tostring(p2), p2.magnitude))
    I:Log(string.format("k1: %s", tostring(k1)))
    I:Log(string.format("k2: %s", tostring(k2)))
    
    local p1_norm = p1.normalized
    local p2_norm = p2.normalized

    I:Log(string.format("p1_norm: %s", tostring(p1_norm)))
    I:Log(string.format("p2_norm: %s", tostring(p2_norm)))

    local theta1, theta1_is_ls = subproblem_4_debug(k2, p1_norm, k1, Vector3.Dot(k2, p2_norm), name .. "_theta1", I)
    local theta2, theta2_is_ls = subproblem_4_debug(k1, p2_norm, k2, Vector3.Dot(k1, p1_norm), name .. "_theta2", I)

    -- reverse theta2 and duplicate any angle with less solutions
    if #theta1 > 1 or #theta2 > 1 then
        theta1 = {theta1[1], theta1[#theta1]}
        theta2 = {theta2[#theta2], theta2[1]}
        I:Log("Adjusted solutions for pairing")
    end

    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-8 or theta1_is_ls or theta2_is_ls
    
    I:Log(string.format("Final theta1 solutions: %d", #theta1))
    I:Log(string.format("Final theta2 solutions: %d", #theta2))
    I:Log(string.format("Overall is_ls: %s", tostring(is_ls)))
    I:Log("")

    return theta1, theta2, is_ls
end

local function subproblem_1_debug(p1, p2, k, name, I)
    I:Log(string.format("=== Subproblem 1 Debug: %s ===", name or "unnamed"))
    I:Log(string.format("p1: %s (mag: %.4f)", tostring(p1), p1.magnitude))
    I:Log(string.format("p2: %s (mag: %.4f)", tostring(p2), p2.magnitude))
    I:Log(string.format("k: %s", tostring(k)))
    
    local kxp = Vector3.Cross(k, p1)
    local kxm = Vector3.Cross(kxp, k)
    
    I:Log(string.format("k×p1: %s", tostring(kxp)))
    I:Log(string.format("(k×p1)×k: %s", tostring(kxm)))

    local A = Matrix:new{
        {kxp.x, kxm.x},
        {kxp.y, kxm.y},
        {kxp.z, kxm.z}
    }

    local x = transpose(A) * p2
    I:Log(string.format("A^T * p2: [%.4f, %.4f]", x[1][1], x[1][2]))

    local theta = math.atan(x[1][1], x[1][2])
    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-6 or math.abs(Vector3.Dot(k, p1) - Vector3.Dot(k, p2)) > 1e-6

    I:Log(string.format("Solution: %.4f rad (%.1f deg)", theta, theta*180/math.pi))
    I:Log(string.format("is_ls: %s", tostring(is_ls)))
    I:Log("")

    return theta, is_ls
end

-- Enhanced SEW function with debugging
local function inv_kin_debug(S, W, psi, I)
    I:Log("=== SEW Parameterization Debug ===")
    I:Log(string.format("S (shoulder): %s", tostring(S)))
    I:Log(string.format("W (wrist): %s", tostring(W)))
    I:Log(string.format("psi: %.4f rad (%.1f deg)", psi, psi*180/math.pi))
    
    local p_SW = W-S
    local e_SW = p_SW.normalized
    
    I:Log(string.format("p_SW: %s (mag: %.4f)", tostring(p_SW), p_SW.magnitude))
    I:Log(string.format("e_SW: %s", tostring(e_SW)))
    
    local k_r = Vector3.Cross(e_SW - E_T, E_R)
    local k_x = Vector3.Cross(k_r, p_SW)
    local e_x = k_x.normalized
    
    I:Log(string.format("E_T: %s", tostring(E_T)))
    I:Log(string.format("E_R: %s", tostring(E_R)))
    I:Log(string.format("k_r: %s", tostring(k_r)))
    I:Log(string.format("k_x: %s", tostring(k_x)))
    I:Log(string.format("e_x: %s", tostring(e_x)))
    
    local e_CE = rot(e_SW, psi)*e_x
    local n_SEW = Vector3.Cross(e_SW, e_CE)

    I:Log(string.format("e_CE: %s", tostring(e_CE)))
    I:Log(string.format("n_SEW: %s", tostring(n_SEW)))
    I:Log("")

    return e_CE, n_SEW
end

-- Main debugging IK function
local function IK_3R_R_3R_debug(R_07, p_0T, psi, kin, I)
    I:Log("========================================")
    I:Log("IK SOLVER DEBUG SESSION")
    I:Log("========================================")
    
    local Q = {}
    local is_LS_vec = {}

    I:Log("=== Input Validation ===")
    I:Log(string.format("Target position p_0T: %s", tostring(p_0T)))
    I:Log("Target rotation R_07:")
    I:Log(R_07:tostring())
    I:Log(string.format("SEW angle psi: %.4f rad (%.1f deg)", psi, psi*180/math.pi))

    -- Find wrist position
    I:Log("=== Wrist Position Calculation ===")
    local R_07_times_P8 = R_07 * kin.P[8]
    I:Log(string.format("R_07 * P[8]: %s", tostring(R_07_times_P8)))
    
    local W = p_0T - R_07_times_P8
    I:Log(string.format("Wrist position W: %s", tostring(W)))

    -- Find shoulder position  
    local S = kin.P[1]
    I:Log(string.format("Shoulder position S: %s", tostring(S)))
    
    local p_SW = W - S
    I:Log(string.format("Shoulder-to-wrist vector p_SW: %s", tostring(p_SW)))
    
    -- Check reachability
    local reach_dist = p_SW.magnitude
    local max_reach = dse + dew
    I:Log(string.format("Reach distance: %.4f, Max reach: %.4f", reach_dist, max_reach))
    
    if reach_dist > max_reach * 1.01 then
        I:Log("❌ Target unreachable - returning empty solution")
        return Q, is_LS_vec
    end
    
    if reach_dist < 0.01 then
        I:Log("❌ Target too close to shoulder - returning empty solution")
        return Q, is_LS_vec
    end

    local e_SW = p_SW.normalized
    local e_CE, n_SEW = inv_kin_debug(S, W, psi, I)

    -- Check if SEW vectors are reasonable
    if e_CE.magnitude < 0.01 or n_SEW.magnitude < 0.01 then
        I:Log("❌ Invalid SEW parameterization - vectors too small")
        return Q, is_LS_vec
    end

    -- Solve for q4 using subproblem 3
    I:Log("=== Solving for q4 (elbow joint) ===")
    local t4, t4_is_LS = subproblem_3_debug(kin.P[5], -kin.P[4], kin.H[4], reach_dist, "q4", I)
    
    if not t4 or #t4 == 0 then
        I:Log("❌ No solution for q4 - algorithm failed at elbow joint")
        return Q, is_LS_vec
    end

    I:Log(string.format("✓ Found %d solution(s) for q4", #t4))

    for i_4 = 1, #t4 do
        local q4 = t4[i_4]
        I:Log(string.format("--- Trying q4 solution %d: %.4f rad (%.1f deg) ---", i_4, q4, q4*180/math.pi))

        -- Solve for theta_b, theta_c using subproblem 2
        I:Log("=== Solving for theta_b, theta_c ===")
        local p4_rotated = rot(kin.H[4], q4) * kin.P[5]
        local combined_vec = kin.P[4] + p4_rotated
        
        I:Log(string.format("P[4]: %s", tostring(kin.P[4])))
        I:Log(string.format("Rotated P[5]: %s", tostring(p4_rotated)))
        I:Log(string.format("Combined vector: %s", tostring(combined_vec)))
        
        local t_b, t_c, t_bc_is_LS = subproblem_2_debug(p_SW, combined_vec, -n_SEW, e_SW, "theta_bc", I)
        
        if not t_b or #t_b == 0 then
            I:Log("❌ No solution for theta_b, theta_c")
            goto continue_q4
        end

        local theta_b = t_b[1]
        local theta_c = t_c[1]
        I:Log(string.format("✓ Using theta_b: %.4f, theta_c: %.4f", theta_b, theta_c))

        -- Solve for theta_a using subproblem 4
        I:Log("=== Solving for theta_a ===")
        local rot_bc = rot(n_SEW, theta_b) * rot(e_SW, theta_c)
        local rotated_P4 = rot_bc * kin.P[4]
        
        local t_a, t_a_is_LS = subproblem_4_debug(n_SEW, rotated_P4, e_SW, 0, "theta_a", I)
        
        if not t_a or #t_a == 0 then
            I:Log("❌ No solution for theta_a")
            goto continue_q4
        end

        for i_a = 1, #t_a do
            local theta_a = t_a[i_a]
            I:Log(string.format("--- Trying theta_a solution %d: %.4f ---", i_a, theta_a))

            local R_03 = rot(e_SW, theta_a) * rot(n_SEW, theta_b) * rot(e_SW, theta_c)
            
            -- Check elbow constraint
            local elbow_check = Vector3.Dot(e_CE, R_03 * kin.P[4])
            I:Log(string.format("Elbow constraint check: %.4f (should be > 0)", elbow_check))
            
            if elbow_check < 0 then
                I:Log("❌ Failed elbow constraint")
                goto continue_theta_a
            end

            I:Log("✓ Passed elbow constraint, proceeding to solve remaining joints...")

            -- [Continue with the rest of the joint solving...]
            -- For debugging, you might want to stop here first and verify the first 4 joints work
            
            I:Log("🎯 PARTIAL SUCCESS: Found valid solution for first 4 joints")
            I:Log(string.format("Theta values so far: a=%.3f, b=%.3f, c=%.3f, q4=%.3f", 
                  theta_a, theta_b, theta_c, q4))
            
            -- Temporary: return partial solution for testing
            local partial_solution = {0, 0, 0, q4, 0, 0, 0}  -- You'd need to convert theta_a,b,c to q1,q2,q3
            table.insert(Q, partial_solution)
            table.insert(is_LS_vec, false)
            
            ::continue_theta_a::
        end
        ::continue_q4::
    end
    
    I:Log("========================================")
    I:Log(string.format("IK DEBUG COMPLETE: Found %d solution(s)", #Q))
    I:Log("========================================")
    
    return Q, is_LS_vec
end

-- Test function to debug a specific case
function debug_ik_case(I)
    local target_pos = Vector3(8, 10, 15)  -- Choose a reachable target
    local target_rot = Matrix.Identity
    local psi = 0  -- Start with simple case
    
    I:Log("Starting IK debug with simple test case...")
    
    local Q, is_ls_vec = IK_3R_R_3R_debug(target_rot, target_pos, psi, KIN, I)
    
    if #Q > 0 then
        I:Log("Debug completed - found potential solution!")
    else
        I:Log("Debug completed - no solution found")
    end
end

function Update(I)
    if not DEBUG_ITERATION then
        DEBUG_ITERATION = 0
    end
    
    if DEBUG_ITERATION == 0 then
        debug_ik_case(I)
    end
    
    DEBUG_ITERATION = DEBUG_ITERATION + 1
    if DEBUG_ITERATION > 100 then  -- Only run once, then pause
        DEBUG_ITERATION = 100
    end
end