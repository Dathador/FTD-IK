-- object testing

internal_object = {}
function internal_object:new(data)
    local obj = data
    setmetatable(obj, self)
    self.__index = self
    return obj
end

external_object = {}
function external_object:new(obj)
    local obj = obj or {
        
    }
    setmetatable(obj, self)
    self.__index = self
    obj.value = 0
    return obj
end



function main()

end
