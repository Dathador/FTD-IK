-- animation controller

-- this function eases in and out over two time periods
function Easing3(timer,x1,x2,x3,x4,a,b,c,d)
    --[[
    x1~x2 changes from 0 to 1 with x3~x4 Changes from 1 to 0　
    a=1,b=1 proportional
    a>1,b=1 power function
    a<1,b>1 exponential  ( a=(10-b)/10 rather pretty )
    a>1,b<1 logarithmic  ( a=1.3,b=0.5,a=1.7,b=0.3,a=2,b=0.2,a=3.2,b=0.1) ( a=( 1+(1-b)/2 )*10/9 rather pretty )
    ]]
    local var1 = Mathf.Clamp((timer-x1)/(x2-x1),0,1)
    local var2 = Mathf.Clamp((timer-x3)/(x4-x3),0,1)
    local var3 = var1+var2
    local var4
    if var3 <= 0.5 then
        var4 = 0.5*(2*var3)^a
    elseif var3 <= 1 then
        var4 = 1-0.5*(-2*var3+2)^a
    elseif var3 <= 1.5 then
        var4 = 1-0.5*(2*(var3-1))^c
    elseif var3 <= 2 then
        var4 = 0.5*(-2*(var3-1)+2)^c
    end
    if var3 <= 1 then
        return var4^b
    else
        return var4^d
    end
end

-- this function is taken from the code controling the Depthian https://steamcommunity.com/sharedfiles/filedetails/?id=2792198306&searchtext=depthian
-- but has been made into a closure so easing functions can be saved into the keyframes.
-- function to produce a smoth transition between two values
-- x1 and x2 change the start and stop of the smoth curve
-- a and b change the shape of the curve
function new_easing_function(x1,x2,a,b)
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
        local var1 = Mathf.Clamp((time-x1)/(x2-x1),0,1)
        
        local var2 = 1
        if var1 <= 0.5 then -- first half of the curve
            var2 = 0.5*(2*var1)^a
        elseif var1 <= 1 then -- second half of the curve
            var2 = 1-0.5*(-2*var1+2)^a
        end
        return var2^b
    end
end

local function Vector3_ease(v1, v2, easing_function, t)
    return Vector3.Lerp(v1, v2, easing_function(t))
end

local function Quaternion_ease(q1, q2, easing_function, t)
    return Quaternion.Slerp(q1, q2, easing_function(t))
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

keyframe = {}
function keyframe:new(obj)
    local obj = obj or {
        time = 0,
        right_arm = {
            position = Vector3(0, 0, 0),
            rotation = Quaternion.Identity, -- Quaternion so we can do slerp
            sew_angle = 0,
            -- interpolations between keyframe i and i+1 will use the easing function of keyframe i+1
            easing_function = nil,
            ik_solution = 1
        },
        left_arm = {
            position = Vector3(0, 0, 0),
            rotation = Quaternion.Identity,
            sew_angle = 0,
            easing_function = nil,
            ik_solution = 1
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

animation = {}
function animation:new(obj)
    local obj = {
        keyframes = {},
        duration = 0, -- in milliseconds
        start_time = 0,
        playing = false,
        loop = false
    } or obj
    setmetatable(obj, self)
    self.__index = self
    return obj
end

-- get current keyframe index and next keyframe index based on current_time
function animation:get_keyframe_indices(current_time)
    local kf1_index = 1
    local kf2_index = 2
    for i = 1, #self.keyframes - 1 do
        if current_time >= self.keyframes[i].time and current_time < self.keyframes[i + 1].time then
            kf1_index = i
            kf2_index = i + 1
            break
        end
    end
    return kf1_index, kf2_index
end

-- interpolate between two keyframes based on current_time
function animation:interpolate_keyframes(current_time, I)
    local kf1_index, kf2_index = self:get_keyframe_indices(current_time)
    I:Log("Interpolating between keyframes " .. kf1_index .. " and " .. kf2_index)
    local kf1 = self.keyframes[kf1_index]
    local kf2 = self.keyframes[kf2_index]

    local t = (current_time - kf1.time) / (kf2.time - kf1.time)
    t = Mathf.Clamp(t, 0, 1)

    -- right arm
    local right_arm_pos = Vector3_ease(kf1.right_arm.position, kf2.right_arm.position, kf2.right_arm.easing_function, t)
    local right_arm_rot = Quaternion_ease(kf1.right_arm.rotation, kf2.right_arm.rotation, kf2.right_arm.easing_function, t)
    local right_arm_sew = angle_ease(kf1.right_arm.sew_angle, kf2.right_arm.sew_angle, kf2.right_arm.easing_function, t)

    move_arm_to_target(quaternion_to_rotation_matrix(right_arm_rot), right_arm_pos, right_arm_sew, kf2.right_arm.ik_solution, RIGHT_ARM_PARAMETERS, I)

    -- left arm
    local left_arm_pos = Vector3_ease(kf1.left_arm.position, kf2.left_arm.position, kf2.left_arm.easing_function, t)
    local left_arm_rot = Quaternion_ease(kf1.left_arm.rotation, kf2.left_arm.rotation, kf2.left_arm.easing_function, t)
    local left_arm_sew = angle_ease(kf1.left_arm.sew_angle, kf2.left_arm.sew_angle, kf2.left_arm.easing_function, t)

    move_arm_to_target(quaternion_to_rotation_matrix(left_arm_rot), left_arm_pos, left_arm_sew, kf2.left_arm.ik_solution, LEFT_ARM_PARAMETERS, I)
end



-- useful FTD interface functions
-- I:GetTime()
-- I:GetTimeSinceSpawn()
-- I:GetGameTime()

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
        keyframes = {
            keyframe:new{
                time = 0,
                right_arm = {
                    position = Vector3(-10, 17, 25),
                    rotation = Quaternion.identity,
                    sew_angle = math.pi * 0.48,
                    easing_function = new_easing_function(0, 200, 2, 2),
                    ik_solution = 1
                },
                left_arm = {
                    position = Vector3(-13, 15, 5),
                    rotation = Quaternion.identity,
                    sew_angle = math.pi * 0.48,
                    easing_function = new_easing_function(0, 200, 2, 2),
                    ik_solution = 1
                }
            },
            keyframe:new{
                time = 200,
                right_arm = {
                    position = Vector3(-5, 20, 30),
                    rotation = Quaternion.Euler(0, math.rad(45), 0),
                    sew_angle = math.pi * 0.48,
                    easing_function = new_easing_function(200, 400, 2, 2),
                    ik_solution = 1
                },
                left_arm = {
                    position = Vector3(-13, 15, 5),
                    rotation = Quaternion.identity,
                    sew_angle = math.pi * 0.48,
                    easing_function = new_easing_function(200, 400, 2, 2),
                    ik_solution = 1
                }
            },
            keyframe:new{
                time = 400,
                right_arm = {
                    position = Vector3(-10, 17, 25),
                    rotation = Quaternion.identity,
                    sew_angle = math.pi * 0.48,
                    easing_function = new_easing_function(400, 600, 2, 2),
                    ik_solution = 1
                },
                left_arm = {
                    position = Vector3(-13, 15, 5),
                    rotation = Quaternion.identity,
                    sew_angle = math.pi * 0.48,
                    easing_function = new_easing_function(400, 600, 2, 2),
                    ik_solution = 1
                }
            }
        },
        duration = 600,
        playing = false,
        loop = false
    }
)



function Update(I)
    local current_time = I:GetTimeSinceSpawn()

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
        else -- animation finished
            state = "idle"
        end
    end
end