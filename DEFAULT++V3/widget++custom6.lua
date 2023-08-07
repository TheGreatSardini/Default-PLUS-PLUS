--DEFAULT ++ windows colors variables:
--------------------------------------
--params.Menu_Settings.TITTLE_COLOR.value
--params.Menu_Settings.TITTLE_COLOR_A.value
--params.Menu_Settings.TITTLE_TEXT_COLOR.value
--params.Menu_Settings.WINDOW_COLOR.value
--params.Menu_Settings.WINDOW_COLOR_A.value
--params.Menu_Settings.WINDOW_TEXT_COLOR.value
--params.Menu_Settings.BUTTON_COLOR.value
--params.Menu_Settings.BUTTON_BORDER_COLOR.value
--params.Menu_Settings.BUTTON_COLOR_A.value
--params.Menu_Settings.BUTTON_TEXT_COLOR.value
--params.Menu_Settings.WIDGET_TEXT_COLOR.value
--params.Menu_Settings.WIDGET_ANIM_COLOR.value
--params.Menu_Settings.WIDGET_FIXED_COLOR.value

--DEFAULT ++ updated variables:
-------------------------------
--currentTime = num
--inspace = 0 in atmo 1 in space
--xSpeedKPH = num kmph
--ySpeedKPH = num kmph
--zSpeedKPH = num kmph
--xyzSpeedKPH = num kmph
--Az = drift rot angle in deg
--Ax = drift pitch angle in deg
--Ax0 = pitch angle in deg
--Ay0 = roll angle in deg
--ThrottlePos = num
--MasterMode = string ("CRUISE" / "TRAVEL" / "PARKING")
--closestPlanetIndex = num (planet index for Helios library)
--atmofueltank = JSON
--spacefueltank = JSON
--rocketfueltank = JSON
--fueltanks = table (all fueltanks JSON data)
--fueltanks_size = num (total number of fuel tanks)

--DEFAULT ++ keybind variables:
-------------------------------
--CLICK = bool
--CTRL = bool
--ALT = bool
--SHIFT = bool
--pitchInput = num (-1 / 0 / 1)
--rollInput = num (-1 / 0 / 1)
--yawInput = num (-1 / 0 / 1)
--brakeInput = num (-1 / 0 / 1)
--strafeInput = num (-1 / 0 / 1)
--upInput = num (-1 / 0 / 1)
--forwardInput = num (-1 / 0 / 1)
--boosterInput = num (-1 / 0 / 1)

local widget_font = "Play"
local utils = require("cpml/utils")
local floor, sqrt, format, asin, clamp = math.floor, math.sqrt, string.format, math.asin, utils.clamp
WidgetsPlusPlusCustom = {}
WidgetsPlusPlusCustom.__index = WidgetsPlusPlusCustom

function WidgetsPlusPlusCustom.new(core, unit, DB, antigrav, warpdrive, shield, switch)
    local self = setmetatable({}, WidgetsPlusPlusCustom)
    self.core = core
    self.unit = unit
    self.DB = DB
    self.antigrav = antigrav
    self.warpdrive = warpdrive
    self.shield = shield
    self.switch = switch
    
    self.modeOn = true
    
    self.buttons = {} -- list of buttons to be implemented in widget
    self.name = "DRONE FLIGHT++" -- name of the widget
    self.SVGSize = {x=250,y=100} -- size of the window to fit the svg, in pixels
    self.pos = {x=500, y=500}
    self.class = "widgetnopadding"  --class = "widgets" (only svg)/ class = "widgetnopadding" (default++ widget style)
    self.draggable = true  --allow widget to be dragged
    self.fixed = false  --prevent widget from going over others
    self.tittle = "DRONE FLIGHT++" --tittle for default++ widget style
    return self
end

function WidgetsPlusPlusCustom.getSize(self) --returns the svg size
    return self.SVGSize
end

function WidgetsPlusPlusCustom.getName(self) --returns the widget name
    return self.name
end

function WidgetsPlusPlusCustom.getTittle(self) --returns the widget name
    return self.tittle
end

function WidgetsPlusPlusCustom.getPos(self) --returns the widget name
    return self.pos
end

function WidgetsPlusPlusCustom.getButtons(self) --returns buttons list
    return self.buttons
end

local abs, floor, asin, sqrt, cos, acos, sin, deg, atan, rad, sign, clamp, rad2deg = math.abs, math.floor, math.asin, math.sqrt, math.cos, math.acos, math.sin, math.deg, math.atan, math.rad, utils.sign, utils.clamp, constants.rad2deg
----------------------------------------------------------------
--               Thrust Management library                    --
----------------------------------------------------------------
-- Vectors manipulations --
---------------------------
local function normalizeVec(x,y,z)
    local l = sqrt(x*x + y*y + z*z)
    return x/l, y/l, z/l
end

local function dotVec(x1,y1,z1,x2,y2,z2)
    return x1*x2 + y1*y2 + z1*z2
end

local function rotateVec(vx, vy, vz, phi, ax, ay, az)
    local l = sqrt(ax*ax + ay*ay + az*az)
    local ux, uy, uz = ax/l, ay/l, az/l
    local c, s = cos(phi), sin(phi)
    local m1x, m1y, m1z = (c + ux * ux * (1-c)), (ux * uy * (1-c) - uz * s), (ux * uz * (1-c) + uy * s)
    local m2x, m2y, m2z = (uy * ux * (1-c) + uz * s), (c + uy * uy * (1-c)), (uy * uz * (1-c) - ux * s)
    local m3x, m3y, m3z = (uz * ux * (1-c) - uy * s), (uz * uy * (1-c) + ux * s), (c + uz * uz * (1-c))
    return m1x*vx+m1y*vy+m1z*vz, m2x*vx+m2y*vy+m2z*vz, m3x*vx+m3y*vy+m3z*vz
end

local function vectorLen(x,y,z)
    return sqrt(x * x + y * y + z * z)
end

local function cross(x, y, z, vx, vy, vz)
    return y*vz - z*vy, z*vx - x*vz, x*vy - y*vx
end

local function local2World(vx,vy,vz)
    local x = vx * cWORx + vy * cWOFx + vz * cWOUPx + cWCOMx
    local y = vx * cWORy + vy * cWOFy + vz * cWOUPy + cWCOMy
    local z = vx * cWORz + vy * cWOFz + vz * cWOUPz + cWCOMz
    return x,y,z
end

-- Rotations control --
-----------------------
local function getConstructRot(x, y, z)
    if x == nil then x, y, z = -1,0,0 end
    x, y, z = normalizeVec(x,y,z)
    local CRx, CRy, CRz = cWORx, cWORy, cWORz
    local CUx, CUy, CUz = cWOUPx, cWOUPy, cWOUPz
    local cx, cy, cz = cross(x, y, z, CUx, CUy, CUz)
    local rAx, rAy, rAz =  normalizeVec(cx, cy, cz) -- rot axis
    local ConstructRot = acos(clamp(dotVec(rAx, rAy, rAz,CRx, CRy, CRz), -1, 1)) * rad2deg
    cx, cy, cz = cross(rAx, rAy, rAz, CRx, CRy, CRz)
    if dotVec(cx, cy, cz, CUx, CUy, CUz) > 0 then ConstructRot = -ConstructRot end
    return ConstructRot
end

local function getConstructPitch(x, y, z)
    if x == nil then x, y, z = 0,0,1 end
    x, y, z = normalizeVec(x,y,z)
    local CRx, CRy, CRz = cWORx, cWORy, cWORz
    local CFx, CFy, CFz = cWOFx, cWOFy, cWOFz
    local cx, cy, cz = cross(x, y, z, CRx, CRy, CRz)
    local pAx, pAy, pAz =  normalizeVec(cx, cy, cz) --pith axis
    local ConstructPitch = acos(clamp(dotVec(pAx, pAy, pAz, CFx, CFy, CFz), -1, 1)) * rad2deg
    cx, cy, cz = cross(pAx, pAy, pAz, CFx, CFy, CFz)
    if dotVec(cx, cy, cz, CRx, CRy, CRz) < 0 then ConstructPitch = -ConstructPitch end
    return ConstructPitch
end

local function getConstructRoll(x,y,z)
    if x == nil then x, y, z = 0,0,1 end
    x, y, z = normalizeVec(x,y,z)
    local CRx, CRy, CRz = cWORx, cWORy, cWORz
    local CFx, CFy, CFz = -cWOFx, -cWOFy, -cWOFz
    local cx, cy, cz = cross(x, y, z, CFx, CFy, CFz)
    local rAx, rAy, rAz =  normalizeVec(cx, cy, cz) --roll Axis
    local ConstructRoll = acos(clamp(dotVec(rAx, rAy, rAz, CRx, CRy, CRz), -1, 1)) * rad2deg
    cx, cy, cz = cross(rAx, rAy, rAz, CRx, CRy, CRz)
    if dotVec(cx, cy, cz, CFx, CFy, CFz) < 0 then ConstructRoll = -ConstructRoll end
    return ConstructRoll
end

local function rollAngularVelocity(x,y,z, angle, speed)
    if x == nil then x, y, z = 0,0,1 end
    x, y, z = normalizeVec(x,y,z)
    local CFx, CFy, CFz = -cWOFx, -cWOFy, -cWOFz
    if angle ~= 0 then x, y, z = rotateVec(x, y, z, rad(-angle), CFx, CFy, CFz) end
    local RollDeg = getConstructRoll(x, y, z)
    if (RollPID == nil) then 
        RollPID = pid.new(0.05 * speed, 0, 10 * speed)
    end
    RollPID:inject(0 - RollDeg)
    local PIDget = RollPID:get()
    return PIDget * CFx, PIDget * CFy, PIDget * CFz
end

local function pitchAngularVelocity (x,y,z, angle, speed)
    if x == nil then x, y, z = 0,0,1 end
    x, y, z = normalizeVec(x,y,z)
    local CRx, CRy, CRz = cWORx, cWORy, cWORz
    if angle ~= 0 then x, y, z = rotateVec(x, y, z, rad(-angle), CRx, CRy, CRz) end
    local PitchDeg = getConstructPitch(x, y, z)
        if (PitchPID == nil) then 
        PitchPID = pid.new(0.05 * speed, 0, 10 * speed)
        end
    PitchPID:inject(0-PitchDeg)
    local PIDget = PitchPID:get() --DUSystem.print(angle.." / "..PitchDeg.." / "..PIDget)
    return PIDget * CRx, PIDget * CRy, PIDget * CRz
end

local function yawAngularVelocity (x,y,z, angle, speed)
    if x == nil then x, y, z = -1,0,0 end
    x, y, z = normalizeVec(x,y,z)
    local CUx, CUy, CUz = -cWOUPx, -cWOUPy, -cWOUPz
    if angle ~= 0 then x, y, z = rotateVec(x, y, z, rad(-angle), CUx, CUy, CUz) end
    local YawDeg = getConstructRot(x, y, z)
        if (YawPID == nil) then 
        YawPID = pid.new(0.05 * speed, 0, 1 * speed)
        end
    YawPID:inject(0 - YawDeg)
    local PIDget = YawPID:get()
    return PIDget * CUx, PIDget * CUy, PIDget * CUz
end

local function getAAR(ox, oy, oz, nx, ny, nz, px, py, pz)
    ox, oy, oz = normalizeVec(ox, oy, oz)
    nx, ny, nz = normalizeVec(nx, ny, nz)
    local ax, ay, az = cross(ox, oy, oz, nx, ny, nz)
    local axisLen = vectorLen(ax, ay, az)
    local angle = 0
    ax, ay, az = normalizeVec(ax, ay, az)
    if axisLen > 0.000001
    then
        angle = asin(clamp(axisLen, 0, 1))
    else
        ax, ay, az = px, py, pz
    end
    if dotVec(ox, oy, oz, nx, ny, nz) < 0
    then
        angle = math.pi - angle
    end
    return ax, ay, az, angle
end


local cOMO = 10 --center of mass offset: will increase or decrease final vertical thrust

function WidgetsPlusPlusCustom.flushOverRide(self) --replace the flush thrust
    --DUSystem.print("test flush")
    if self.modeOn == true then
        -- Final inputs --
        ------------------
        local finalPitchInput = pitchInput + DUSystem.getControlDeviceForwardInput()
        local finalRollInput = rollInput + DUSystem.getControlDeviceYawInput()
        local finalYawInput = yawInput - DUSystem.getControlDeviceLeftRightInput()
        local finalRotationInput = finalPitchInput + finalRollInput + finalYawInput
        local finalPRInput = abs(finalPitchInput) + abs(finalRollInput)

        -- Globals --
        -------------
        local cWCOM = DUConstruct.getWorldCenterOfMass()
        cWCOMx, cWCOMy, cWCOMz = cWCOM[1], cWCOM[2], cWCOM[3]
        local cPCx, cPCy, cPCz = currentPlanetCenter[1], currentPlanetCenter[2], currentPlanetCenter[3]

        local cWOUP = {0,0,0} --core.getConstructWorldOrientationUp()
        local cWOF = {0,0,0} --core.getConstructWorldOrientationForward()
        local cWOR = {0,0,0} --core.getConstructWorldOrientationRight()
        local wVx, wVy, wVz = cWCOMx-cPCx, cWCOMy-cPCy, cWCOMz-cPCz  -- world vertical

        if DUgyro and params.Engines_Settings.gyroAxis.value == true then
            if DUgyro.isActive() ~= true then DUgyro.activate() end
            cWOUP = DUgyro.worldUp()
            cWOF = DUgyro.worldForward()
            cWOR = DUgyro.worldRight()
        else
            cWOUP = DUConstruct.getWorldOrientationUp()
            cWOF = DUConstruct.getWorldOrientationForward()
            cWOR = DUConstruct.getWorldOrientationRight()
        end

        -- ROTATIONS --
        ---------------
        local tAVx = finalYawInput *  params.Engines_Settings.yawSpeedFactor.value * cWOUPx
        local tAVy = finalYawInput *  params.Engines_Settings.yawSpeedFactor.value * cWOUPy
        local tAVz = finalYawInput *  params.Engines_Settings.yawSpeedFactor.value * cWOUPz

        -- DRONE FLIGHT VARIABLES --
        ----------------------------
        local targetLongSpeed = 0 -- default braking if no input
        local targetLatSpeed = 0 -- default braking if no input
        local targetVertSpeed = 0 -- default braking if no input
        local navSpeed = 1000
        local strafeSpeed = 100

        local cOMOF = 0 --center of mass offset forward
        local cOMOR = 0 --center of mass offset right

        -- Shift Adjustments --
        local rollModifier = 0
        local pitchModifier = 0
        if SHIFT == false then 
            rollModifier = 22.5
            pitchModifier = 22.5
        elseif SHIFT == true then
            rollModifier = 45
            pitchModifier = 45
        end

        -- Roll Stabilization & Drone Rolling --
        local rollOffset = (finalRollInput * rollModifier) + params.Engines_Settings.rollAngleAdjustment.value
        local autoRollRollThreshold = 0.0
        if abs(Ay0 - rollOffset) >= autoRollRollThreshold then
            local rAVx, rAVy, rAVz = rollAngularVelocity(wVx, wVy, wVz, 0 + rollOffset, params.Engines_Settings.rollSpeedFactor.value)
            tAVx = tAVx + rAVx
            tAVy = tAVy + rAVy
            tAVz = tAVz + rAVz
            cOMOF = cOMO
        end
        
        --Pitch Stabilization & Drone Pitching --
        local pitchOffset = (finalPitchInput * pitchModifier) - params.Engines_Settings.pitchAngleAdjustment.value
        local autoPitchAmplitude = 90
        if abs(Ax0) < autoPitchAmplitude then
            local pAVx, pAVy, pAVz = pitchAngularVelocity(wVx, wVy, wVz, 0 - pitchOffset , params.Engines_Settings.pitchSpeedFactor.value)
            tAVx = tAVx + pAVx
            tAVy = tAVy + pAVy
            tAVz = tAVz + pAVz
            cOMOR = cOMO
        end
        -- Commented out to remove offset center calculations
        --local cCOM = DUConstruct.getCenterOfMass()
        --local cCOMx, cCOMy, cCOMz = cCOM[1], cCOM[2], cCOM[3]
        --local cOMOWPx, cOMOWPy, cOMOWPz = local2World(cCOMx+cOMOR,cCOMy+cOMOF,0)-- center of mass with offset world pos
        --local altM = vectorLen(cOMOWPx-cPCx, cOMOWPy-cPCy, cOMOWPz-cPCz) - currentPlanetRadius or 0
        --DUSystem.print(altM.."/"..cCOMx.."/"..cOMOR.."/"..cCOMy.."/"..cOMOF)
        local altM = vectorLen(cWCOMx-cPCx, cWCOMy-cPCy, cWCOMz-cPCz) - currentPlanetRadius or 0 --Altitude to center of mass

        -- Altitude Stabilization --
        if VStabAltMLock ~= nil and (finalPRInput == 0 or upInput ~= 0) then
            VStabAltMLock = nil
            --DUSystem.print(finalPRInput.."/"..upInput)
            --DUSystem.print("Altitude stabilisation unlocked")
        elseif finalPRInput ~= 0 and VStabAltMLock == nil then
            VStabAltMLock = altM
            --DUSystem.print("Altitude locked at: "..floor(VStabAltMLock))
        end

        -- TODO lock to always use cruise???
        -- TODO This should be enclosed in only while pitching function?
        local pitchStrengthFactor = sign(Ax0)*clamp(Ax0,-90,90)/90 --0 to 1
        local rollStrengthFactor = sign(Ay0)*clamp(Ay0,-90,90)/90 --0 to 1
        --DUSystem.print(90-math.deg(math.atan( 1 / math.sqrt( math.tan(math.rad(pitchStrengthFactor))^2 + math.tan(math.rad(rollStrengthFactor))^2 ) )))
    
        --targetLongSpeed = upImput ~= 0 and 0 or targetVertSpeed
        --targetLatSpeed = upImput ~= 0 and 0 or targetVertSpeed
        --DUSystem.print(targetVertSpeed)
        --#####################################################
        local accAdd = 10 -- increment to set target speed to above current speed
        local accMulti = 2 -- multiple of current speed to set target speed

        -- Longitudinal Speed Calculation --
        ------------------------------------
        local pitchSensitivity = 0.05
        local yzSpeedKPH = sqrt(ySpeedKPH^2+zSpeedKPH^2)
        local longSpeedStep = navSpeed
        if pitchStrengthFactor > pitchSensitivity then
            if abs(yzSpeedKPH*accMulti)+accAdd < navSpeed then
                longSpeedStep = abs(yzSpeedKPH*accMulti)+accAdd
            end
            targetLongSpeed = (longSpeedStep*((sign(Ax0)*-1)-pitchStrengthFactor))
            --DUSystem.print(targetLongSpeed)
        end
        
        -- Laterial Speed Calculation --
        --------------------------------
        local rollSensitivity = 0.05
        local xzSpeedKPH = sqrt(xSpeedKPH^2+zSpeedKPH^2)
        local latSpeedStep = navSpeed
        if rollStrengthFactor > rollSensitivity then
            if abs(xzSpeedKPH*accMulti)+accAdd < navSpeed then
                latSpeedStep = abs(xzSpeedKPH*accMulti)+accAdd
            end
            targetLatSpeed = (latSpeedStep*((sign(Ay0)*-1)-rollStrengthFactor))
        end

        -- Vertical Speed Calculation --
        --------------------------------
        local vertSpeedStep = navSpeed
        local prDegrees = 90-deg(atan( 1 / sqrt( math.tan(rad(Ax0))^2 + math.tan(rad(Ay0))^2 ) ))
        local prStrengthFactor = clamp(prDegrees,-90,90)/90 -- 0 to 1 rotation between 0 - 90 degrees
        if pitchStrengthFactor > pitchSensitivity or rollStrengthFactor > rollSensitivity then
            if abs(xyzSpeedKPH*accMulti)+accAdd < navSpeed then
                vertSpeedStep = abs(xzSpeedKPH*accMulti)+accAdd
            end
            --DUSystem.print("targetVertSpeed: "..targetVertSpeed)
            targetVertSpeed = targetVertSpeed+(vertSpeedStep*prStrengthFactor)
        end

        -- Altitude Adjustment --
        local altAdjMultBoost = 5 --multiply the power of altitude adjustment boosting
        local prSlideSensitivity = 10 --degrees before reducing altitude by sliding vs decreasing throttle.
        if upInput ~= 0 then
            targetVertSpeed = targetVertSpeed + upInput*strafeSpeed
        elseif VStabAltMLock ~= nil then
            if pitchStrengthFactor > pitchSensitivity or rollStrengthFactor > rollSensitivity then
                local distToLock = VStabAltMLock-altM
                if sign(distToLock) < 0 then -- need to reduce altitude
                    --DUSystem.print("REDUCE: "..targetVertSpeed)
                    if prDegrees > prSlideSensitivity then
                        targetLatSpeed = targetLatSpeed + (targetLatSpeed * abs(distToLock) * (1-rollStrengthFactor))
                        targetLongSpeed = targetLongSpeed + (targetLongSpeed * abs(distToLock) * (1-pitchStrengthFactor))
                        targetVertSpeed = targetVertSpeed + distToLock
                        --DUSystem.print("longSpeed: "..targetLongSpeed.." latSpeed: "..targetLatSpeed)
                    else -- mostly flat, just reduce throttle
                        targetVertSpeed = targetVertSpeed + distToLock
                    end
                else -- need to gain altitude
                    local speedBoost = distToLock+(distToLock*prStrengthFactor*altAdjMultBoost)
                    --DUSystem.print("INCREASE: "..speedBoost)
                    targetVertSpeed = targetVertSpeed + speedBoost
                end
            else
                targetVertspeed = targetVertSpeed + VStabAltMLock-altM
            end
        end
        --DUSystem.print("final: "..targetVertSpeed.." up: "..upInput)
        --targetVertSpeed = upInput ~= 0 and upInput*strafeSpeed or VStabAltMLock ~= nil and (VStabAltMLock-altM)*abs(VStabAltMLock-altM) or 0
        --DUSystem.print(VStabAltMLock ~= nil and (VStabAltMLock-altM)*abs(VStabAltMLock-altM))


        --#####################################################

        --DUSystem.print(longSpeed.." / "..latSpeed.." / "..vertSpeed.." / "..otAVx.." / "..otAVy.." / "..otAVz)
        return targetLongSpeed, targetLatSpeed, targetVertSpeed, tAVx, tAVy, tAVz
    else
        return nil
    end
end

----------------
-- WIDGET SVG --
----------------
function WidgetsPlusPlusCustom.SVG_Update(self)
    local WTC = params.Menu_Settings.WIDGET_TEXT_COLOR.value

        local bf = function() return function()
                            if mouseWheel == 0 then
                                self.modeOn =  not self.modeOn
                                DUSystem.print("Drone mode has been toggled")
                            elseif mouseWheel > 0 then
                                cOMO = clamp(cOMO+1,0,50)
                            elseif mouseWheel < 0 then
                                cOMO = clamp(cOMO-1,0,50)
                            end
                            windowsShow()
                    end end

        local btText = "Center of Mass: "..cOMO.."m / "..tostring(self.modeOn)
        self.buttons = {
            {btText, bf(), {name = "drone mode++", class = nil, width = 225, height = 25, posX = 0, posY = 70}},   -- class = "separator"   (for invisible background button)
            }

    local SVG = [[
        <text x="0" y="10" font-size="30" text-anchor="start" font-family="]]..widget_font..[[" alignment-baseline="baseline" stroke-width="0" fill="]]..WTC..[[">]].."text"..[[</text> 
        <text x="0" y="30" font-size="20" text-anchor="start" font-family="]]..widget_font..[[" alignment-baseline="baseline" stroke-width="0" fill="]]..WTC..[[">]].."text"..[[</text> 
    ]]
    
    SVG = '<div><svg viewBox="0 0 '.. self.SVGSize.x ..' '.. self.SVGSize.y ..'">'..SVG..'</svg></div>'
    return SVG
end
