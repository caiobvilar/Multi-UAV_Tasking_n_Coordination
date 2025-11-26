draw_polygon = function(poly_data)
	--print(jdata)
	local lineSize = 5 -- in points
	local maximumLines = 9999
	local color = { 1, 1, 0 }
	drawingContainer = sim.addDrawingObject(sim.drawing_lines, lineSize, 0, -1, maximumLines, color) -- adds a line

	for j = 1, #poly_data - 1 do
		local pt0 = poly_data[j]
		local pt1 = poly_data[j + 1]
		local data = { pt0[1], pt0[2], pt0[3], pt1[1], pt1[2], pt1[3] }
		sim.addDrawingObjectItem(drawingContainer, data)
	end
	local pt0 = poly_data[#poly_data]
	local pt1 = poly_data[1]
	local data = { pt0[1], pt0[2], pt0[3], pt1[1], pt1[2], pt1[3] }
	sim.addDrawingObjectItem(drawingContainer, data)
	return drawingContainer
end

--[[draw_lines = function(inInts,inFloats,inStrings,inBuffer)
    local filename = inStrings[1]]
draw_lines = function(filename)
	json = require("json")
	print(filename)
	local file = io.open(filename, "r")
	local str_data = file:read("*a")
	file:close()
	local jdata = json.decode(str_data)

	for i = 1, 3 do
		print(i, jdata["areas"][tostring(i - 1)])
		local area = jdata["areas"][tostring(i - 1)]
		local poly = {}
		for i, v in pairs(area["y"]) do
			-- coordinates need to map from (0, 100) (-5, 5)
			--local x = (area['x'][i]/100)*12 - 6
			--local y = (area['y'][i]/100)*12 - 6
			local x = (area["x"][i] / 100) * 10 - 5
			local y = (area["y"][i] / 100) * 10 - 5
			poly[i] = { x, y, 0.1 }
		end
		draw_polygon(poly)
	end
end

function sysCall_init()
	--local filename = '/home/redwan/PycharmProjects/AreaDecomposition/result.json'
	local filename =
		"/home/hellscoffe/Development/Multi-UAV_Tasking_n_Coordination/MotionPlanner/coppeliaSim/test/area_decomposition_02.json"
	draw_lines(filename)
end

function sysCall_actuation()
	-- put your actuation code here
	--
	-- For example:
	--[[
    local data = sim.getStringSignal('fileName')
    if data then
        draw_lines(data)
        print(data)
        sim.clearStringSignal('fileName')
    end
]]
	-- local position=sim.getObjectPosition(handle,-1)
	-- position[1]=position[1]+0.001
	-- sim.setObjectPosition(handle,-1,position)
end

function sysCall_sensing()
	-- put your sensing code here
end

function sysCall_cleanup()
	-- do some clean-up here
end

-- You can define additional system calls here:
--[[
function sysCall_suspend()
end

function sysCall_resume()
end

function sysCall_dynCallback(inData)
end

function sysCall_jointCallback(inData)
    return outData
end

function sysCall_contactCallback(inData)
    return outData
end

function sysCall_beforeCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be copied")
    end
end

function sysCall_afterCopy(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was copied")
    end
end

function sysCall_beforeDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." will be deleted")
    end
    -- inData.allObjects indicates if all objects in the scene will be deleted
end

function sysCall_afterDelete(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..key.." was deleted")
    end
    -- inData.allObjects indicates if all objects in the scene were deleted
end

function sysCall_afterCreate(inData)
    for key,value in pairs(inData.objectHandles) do
        print("Object with handle "..value.." was created")
    end
end
--]]
