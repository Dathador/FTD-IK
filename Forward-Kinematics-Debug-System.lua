-- Forward Kinematics Debug System
-- This will help identify errors in math and robot parameters

-- Forward kinematics calculation
local function compute_forward_kinematics(joint_angles, kin)
    local transforms = {}
    local positions = {}
    local rotations = {}
    
    -- Initialize with identity
    transforms[0] = Matrix.identity
    positions[0] = Vector3(0, 0, 0)
    rotations[0] = Matrix.identity
    
    -- Compute cumulative transforms
    for i = 1, #joint_angles do
        -- Current joint rotation
        local R_i = rot(kin.H[i], joint_angles[i])
        
        -- Cumulative rotation up to joint i
        rotations[i] = rotations[i-1] * R_i
        
        -- Position of joint i (transform the joint position vector)
        positions[i] = positions[i-1] + rotations[i-1] * kin.P[i]
        
        -- Store transform matrix for debugging
        transforms[i] = rotations[i]
    end
    
    -- End effector position
    local end_effector_pos = positions[#joint_angles] + rotations[#joint_angles] * kin.P[#kin.P]
    
    return positions, rotations, end_effector_pos
end

-- DH parameter based forward kinematics (alternative method)
local function compute_dh_forward_kinematics(joint_angles, dh_params)
    local transforms = {}
    local positions = {}
    
    transforms[0] = Matrix.identity
    positions[0] = Vector3(0, 0, 0)
    
    for i = 1, #joint_angles do
        local a, alpha, d, theta_offset = dh_params[i][1], dh_params[i][2], dh_params[i][3], dh_params[i][4]
        local theta = joint_angles[i] + theta_offset
        
        -- DH transformation matrix
        local ct, st = math.cos(theta), math.sin(theta)
        local ca, sa = math.cos(alpha), math.sin(alpha)
        
        local T_i = Matrix:new({
            {ct, -st*ca, st*sa, a*ct},
            {st, ct*ca, -ct*sa, a*st},
            {0, sa, ca, d},
            {0, 0, 0, 1}
        })
        
        -- Extract 3x3 rotation and position
        local R_i = Matrix:new({
            {T_i[1][1], T_i[1][2], T_i[1][3]},
            {T_i[2][1], T_i[2][2], T_i[2][3]},
            {T_i[3][1], T_i[3][2], T_i[3][3]}
        })
        
        local pos_i = Vector3(T_i[1][4], T_i[2][4], T_i[3][4])
        
        if i == 1 then
            transforms[i] = R_i
            positions[i] = pos_i
        else
            transforms[i] = transforms[i-1] * R_i
            positions[i] = positions[i-1] + transforms[i-1] * pos_i
        end
    end
    
    return positions, transforms
end

-- Get actual joint positions from the game
local function get_actual_joint_positions(I, joints)
    local actual_positions = {}
    local actual_rotations = {}
    
    for i, scid in ipairs(joints) do
        local info = I:GetSubConstructInfo(scid)
        actual_positions[i] = info.LocalPosition
        -- Note: You might need to extract rotation from the construct info
        -- This depends on your game's API
        actual_rotations[i] = info.LocalRotation or Matrix.identity
    end
    
    return actual_positions, actual_rotations
end

-- Compare calculated vs actual positions
local function compare_positions(I, calc_positions, actual_positions, joint_names)
    I:Log("=== Position Comparison ===")
    
    for i = 1, math.min(#calc_positions, #actual_positions) do
        local calc = calc_positions[i]
        local actual = actual_positions[i]
        
        local error = Vector3(
            calc.x - actual.x,
            calc.y - actual.y,
            calc.z - actual.z
        )
        
        local error_magnitude = math.sqrt(error.x^2 + error.y^2 + error.z^2)
        
        I:Log(string.format("%s (Joint %d):", joint_names[i] or "Joint", i))
        I:Log(string.format("  Calculated: (%.3f, %.3f, %.3f)", calc.x, calc.y, calc.z))
        I:Log(string.format("  Actual:     (%.3f, %.3f, %.3f)", actual.x, actual.y, actual.z))
        I:Log(string.format("  Error:      (%.3f, %.3f, %.3f) - Magnitude: %.3f", error.x, error.y, error.z, error_magnitude))
        
        if error_magnitude > 0.1 then
            I:Log("  ⚠ WARNING: Large position error!")
        end
    end
end

-- Validate joint axes by checking if they're unit vectors
local function validate_joint_axes(I, kin)
    I:Log("=== Joint Axes Validation ===")
    
    for i, axis in ipairs(kin.H) do
        local magnitude = math.sqrt(axis.x^2 + axis.y^2 + axis.z^2)
        I:Log(string.format("Joint %d axis: (%.3f, %.3f, %.3f) - Magnitude: %.3f", i, axis.x, axis.y, axis.z, magnitude))
        
        if math.abs(magnitude - 1.0) > 0.001 then
            I:Log("  ⚠ WARNING: Joint axis is not a unit vector!")
        end
    end
end

-- Check if joint angles are reasonable
local function validate_joint_angles(I, joint_angles)
    I:Log("=== Joint Angles Validation ===")
    
    for i, angle in ipairs(joint_angles) do
        local angle_deg = angle * 180 / math.pi
        I:Log(string.format("Joint %d: %.3f rad (%.1f deg)", i, angle, angle_deg))
        
        if math.abs(angle) > math.pi then
            I:Log("  ⚠ WARNING: Joint angle exceeds ±180 degrees!")
        end
    end
end

-- Test with simple known configuration
local function test_zero_configuration(I, kin, joints)
    I:Log("=== Testing Zero Configuration ===")
    
    local zero_angles = {0, 0, 0, 0, 0, 0, 0}
    
    -- Set joints to zero
    set_joint_angles(I, joints, zero_angles)
    
    -- Wait a frame for the physics to update (you might need to call this multiple times)
    -- Calculate expected positions
    local calc_positions, calc_rotations, calc_end_effector = compute_forward_kinematics(zero_angles, kin)
    
    I:Log("Calculated positions at zero configuration:")
    for i, pos in ipairs(calc_positions) do
        I:Log(string.format("  Joint %d: (%.3f, %.3f, %.3f)", i, pos.x, pos.y, pos.z))
    end
    
    I:Log(string.format("End effector: (%.3f, %.3f, %.3f)", calc_end_effector.x, calc_end_effector.y, calc_end_effector.z))
end

-- Check workspace reachability
local function check_workspace_reachability(I, target_pos, kin)
    I:Log("=== Workspace Reachability Check ===")
    
    local shoulder_pos = kin.P[1]
    local reach_distance = Vector3(target_pos.x - shoulder_pos.x, target_pos.y - shoulder_pos.y, target_pos.z - shoulder_pos.z)
    local reach_magnitude = math.sqrt(reach_distance.x^2 + reach_distance.y^2 + reach_distance.z^2)
    
    -- Maximum reach = shoulder-to-elbow + elbow-to-wrist + wrist-to-tool
    local max_reach = dse + dew + dwt
    local min_reach = math.abs(dse - dew - dwt)  -- Fully folded
    
    I:Log(string.format("Target distance from shoulder: %.3f", reach_magnitude))
    I:Log(string.format("Maximum reach: %.3f", max_reach))
    I:Log(string.format("Minimum reach: %.3f", min_reach))
    
    if reach_magnitude > max_reach then
        I:Log("  ✗ FAIL: Target is outside workspace (too far)")
        return false
    elseif reach_magnitude < min_reach then
        I:Log("  ✗ FAIL: Target is outside workspace (too close)")
        return false
    else
        I:Log("  ✓ PASS: Target is within workspace")
        return true
    end
end

-- Compare different FK methods
local function compare_fk_methods(I, joint_angles, kin, dh_params)
    I:Log("=== Comparing FK Methods ===")
    
    local pos1, rot1, end1 = compute_forward_kinematics(joint_angles, kin)
    local pos2, rot2 = compute_dh_forward_kinematics(joint_angles, dh_params)
    
    I:Log("Method 1 (kin.P/kin.H) vs Method 2 (DH parameters):")
    
    for i = 1, math.min(#pos1, #pos2) do
        local diff = Vector3(pos1[i].x - pos2[i].x, pos1[i].y - pos2[i].y, pos1[i].z - pos2[i].z)
        local diff_mag = math.sqrt(diff.x^2 + diff.y^2 + diff.z^2)
        
        if diff_mag > 0.001 then
            I:Log(string.format("  Joint %d: Methods disagree by %.3f", i, diff_mag))
        end
    end
end

-- Main debug function
function debug_forward_kinematics(I, target_pos, target_rot, psi, kin, joints, joint_names)
    I:Log("========================================")
    I:Log("Forward Kinematics Debug Session")
    I:Log("========================================")
    
    -- Basic validations
    validate_joint_axes(I, kin)
    check_workspace_reachability(I, target_pos, kin)
    
    -- Test zero configuration
    test_zero_configuration(I, kin, joints)
    
    -- Run IK
    local Q, is_ls_vec = IK_3R_R_3R(I, target_rot, target_pos, psi, kin)
    
    if not Q or #Q == 0 then
        I:Log("✗ FAIL: No IK solution found")
        return
    end
    
    I:Log("✓ IK found " .. #Q .. " solutions")
    
    -- Test first solution
    local solution = Q[1]
    validate_joint_angles(I, solution)
    
    -- Set joints and calculate FK
    set_joint_angles(I, joints, solution)
    
    -- Calculate expected positions
    local calc_positions, calc_rotations, calc_end_effector = compute_forward_kinematics(solution, kin)
    
    I:Log("=== Forward Kinematics Results ===")
    I:Log(string.format("Target position: (%.3f, %.3f, %.3f)", target_pos.x, target_pos.y, target_pos.z))
    I:Log(string.format("Calculated end effector: (%.3f, %.3f, %.3f)", calc_end_effector.x, calc_end_effector.y, calc_end_effector.z))
    
    -- Key joint positions
    if calc_positions[4] then
        I:Log(string.format("Calculated elbow (Joint 4): (%.3f, %.3f, %.3f)", calc_positions[4].x, calc_positions[4].y, calc_positions[4].z))
    end
    
    if calc_positions[7] then
        I:Log(string.format("Calculated wrist (Joint 7): (%.3f, %.3f, %.3f)", calc_positions[7].x, calc_positions[7].y, calc_positions[7].z))
    end
    
    -- Compare methods if DH parameters are consistent
    compare_fk_methods(I, solution, kin, DH_PARAMS)
    
    -- Get actual positions and compare (you'll need to wait for physics update)
    -- local actual_positions, actual_rotations = get_actual_joint_positions(I, joints)
    -- compare_positions(I, calc_positions, actual_positions, joint_names)
    
    I:Log("========================================")
    I:Log("Debug Session Complete")
    I:Log("========================================")
end