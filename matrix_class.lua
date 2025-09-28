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

-- Fixed transpose function for Vector3
function transpose(m)
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

function Matrix:get_column(col)
    local result = Vector3(0, 0, 0)
    for i = 1, self.rows do
        result[i] = self[i][col]
    end
    return result
end

function Matrix:mulVector(other)
    local result = {}
    for i = 1, self.rows do
        result[i] = {}
        result[i][1] = self[i][1] * other.x + self[i][2] * other.y + self[i][3] * other.z
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

        -- For 3x3 matrix * Vector3, return a Vector3
        if self.rows == 3 then
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

local function skew_symmetric(v) -- Create a skew-symmetric matrix from a vector
    return Matrix:new{
        {0, -v.z, v.y},
        {v.z, 0, -v.x},
        {-v.y, v.x, 0}
    }
end