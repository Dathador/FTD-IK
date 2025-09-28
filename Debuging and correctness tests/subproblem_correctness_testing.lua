function Update(I)

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

    -- reverse theta2 and duplicate any angle with less solutions
    if #theta1 > 1 or #theta2 > 1 then
        theta1 = {theta1[1], theta1[#theta1]}
        theta2 = {theta2[#theta2], theta2[1]}
    end

    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-8 or theta1_is_ls or theta2_is_ls

    return theta1, theta2, is_ls
end

local function subproblem_1(p1, p2, k, I)
    local KxP = Vector3.Cross(k, p1)
    I:Log("KxP: \n" .. KxP.ToString(KxP))
    local kxm = Vector3.Cross(KxP, k)
    -- make a 3x2 matrix

    local A = Matrix:new{
        {KxP.x, kxm.x},
        {KxP.y, kxm.y},
        {KxP.z, kxm.z}
    }
    I:Log("A: \n" .. A:tostring())

    local x = transpose(A) * p2
    I:Log("x: \n" .. x:tostring())

    local theta = math.atan2(x[1][1], x[2][1])
    local is_ls = math.abs(p1.magnitude - p2.magnitude) > 1e-6 or math.abs(Vector3.Dot(k, p1) - Vector3.Dot(k, p2)) > 1e-6

    I:Log("theta: " .. theta)

    return theta, is_ls
end


-- functions taken from the subproblem correctness tests

local function rand_vector3()
    v = Vector3(math.random(), math.random(), math.random())
    return v * 2 - Vector3(1, 1, 1)
end

local function rand_normal_vector3()
    return rand_vector3().normalized
end

local function rand_angle()
    return math.random() * 2 * math.pi - math.pi
end

local sp_1 = {
    setup = function()
        P = {}
        S = {}
        -- Define parameters for subproblem 1
        P.p1 = Vector3(0.688288,0.425473,-0.597813) -- rand_vector3()
        P.k = Vector3(0.771667,-0.580631,-0.259611) -- rand_normal_vector3()
        S.theta = 1.9868619468534 -- rand_angle()

        -- I:Log(P.p1.ToString(P.p1))
        -- I:Log(P.k.ToString(P.k))
        -- I:Log(S.theta)

        P.p2 = rot(P.k, S.theta) * P.p1

        return P, S
    end,

    run = function(P)
        -- Call the subproblem_1 function
        local theta, is_LS = subproblem_1(P.p1, P.p2, P.k, I)
        return {theta = theta, is_LS = is_LS}
    end,

    error = function(P, S)
        local e = (P.p2 - rot(P.k, S.theta) * P.p1).magnitude
        return e
    end
}

local sp_2 = {
    setup = function()
        P = {}
        S = {}

        P.p1 = rand_vector3()

        P.k1 = rand_normal_vector3()
        P.k2 = rand_normal_vector3()

        S.theta1 = rand_angle()
        S.theta2 = rand_angle()

        P.p2 = rot(P.k2, -S.theta2) * rot(P.k1, S.theta1) * P.p1

        return P, S
    end,

    run = function(P)
        local theta1, theta2 = subproblem_2(P.p1,P.p2,P.k1,P.k2)
        return {theta1 = theta1, theta2 = theta2}
    end,

    error = function(P, S)
        local e
        for i = 1, #S.theta1 do
            local e_i = (rot(P.k2, S.theta2[i]) * P.p2 - rot(P.k1, S.theta1[i]) * P.p1).magnitude
            if i == 1 then
                e = e_i
            else
                e = e + e_i
            end
        end
        return e
    end
}

local sp_3 = {
        setup = function()
            P.p1 = rand_vector3()
            P.p2 = rand_vector3()
            P.k = rand_normal_vector3()
            S.theta = rand_angle()

            P.d = (P.p2-rot(P.k,S.theta)*P.p1).magnitude
            return P, S
        end,

        setup_LS = function()
            P.p1 = rand_vector3()
            P.p2 = rand_vector3()
            P.k = rand_normal_vector3()

            P.d = math.random()
        end,

        run = function(P)
            local theta, is_ls = subproblem_3(P.p1,P.p2,P.k,P.d)
            return {theta = theta, is_ls = is_ls}
        end,

        error = function(P, S)
            local e
            for i = 1, #S.theta do
                e_i = math.abs((P.p2-rot(P.k,S.theta[i])*P.p1).magnitude - P.d)
                if i == 1 then
                    e = e_i
                else
                    e = e + e_i
                end
            end
            return e
        end
}

local sp_4 = {
    setup = function()
        P = {}
        S = {}

        P.p = rand_vector3()
        P.k = rand_normal_vector3()
        P.h = rand_normal_vector3()
        S.theta = rand_angle()

        P.d = Vector3.Dot(P.h, rot(P.k, S.theta) * P.p)
        return P, S
    end,

    setup_LS = function()
        P.p = rand_vector3()
        P.k = rand_normal_vector3()
        P.h = rand_normal_vector3()

        P.d = math.random()
        return P, S
    end,

    run = function(P)
        local theta = subproblem_4(P.h,P.p,P.k,P.d);
        return {theta = theta}
    end,

    error = function(P, S)
        local e
        for i = 1, #S.theta do
            e_i = ((transpose(P.h) * rot(P.k, S.theta[i]) * P.p)[1][1] - P.d)
            if i == 1 then
                e = e_i
            else
                e = e + e_i
            end
        end
        return e
    end
}

local function correctness_test(prob_setup)
    local P, S = prob_setup.setup()
    local S_t = prob_setup.run(P)
    local e = prob_setup.error(P, S_t)
    return e
end

local setups = {
    sp_1,
    -- sp_2,
    -- sp_3,
    -- sp_4
}


local errors = {}
for i = 1, #setups do
    errors[i] = correctness_test(setups[i])
end

-- N_trials = 1e4;
-- errors = NaN(#setups, N_trials)
-- for i = 1, #setups do
--     for j = 1, N_trials do
--         errors[i][j] = correctness_test(setups[i])
--     end
-- end

max = -math.huge
sum = 0
for i = 1, #errors do
    sum = sum + errors[i]
    if errors[i] > max then
        max = errors[i]
    end
end

mean = sum / #errors

-- histogram(errors[1,:]); hold on
-- histogram(errors[2,:]);
-- histogram(errors[3,:]);
-- histogram(errors[4,:]);
-- histogram(errors[5,:]);
-- histogram(errors[6,:]); hold off
-- set(gca,'XScale','log')
-- legend("SP " + string(1:6))

-- plot(sort(errors[1,:])); hold on
-- plot(sort(errors[2,:]));
-- plot(sort(errors[3,:]));
-- plot(sort(errors[4,:]));
-- plot(sort(errors[5,:]));
-- plot(sort(errors[6,:])); hold off
-- set(gca,'YScale','log')
-- legend("SP " + string(1:6))

    I:Log("test")
    I:Log(max)
    I:Log(mean)
    I:Log(sum)
end