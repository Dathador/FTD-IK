-- test rotating a spinblock to a specific angle
function Update(I)
    local target_angle = 45
    local scid = I:GetSubConstructIdentifier(0)
    I:SetSpinBlockRotationAngle(scid, target_angle)
end