local getPositions = function(quadHandles)
	local results = {}
	for i, v in pairs(quadHandles) do
		results[i] = sim.getObjectPosition(v, -1)
	end
	return results
end

function sysCall_init()
	-- do some initialization here
	local quadNames = { "/Quadcopter[0]", "/Quadcopter[1]", "/Quadcopter[2]" }
	quadHandles = {}
	drawingHandles = {}
	for i = 1, #quadNames do
		table.insert(quadHandles, sim.getObject(quadNames[i]))
		local color = { 0, 0, 0 }
		color[i] = 1
		local dr = sim.addDrawingObject(sim.drawing_lines | sim.drawing_cyclic, 2, 0, -1, 9999, color)
		table.insert(drawingHandles, dr)
	end
	--prev = getPositions(quadHandles)
	prev = nil
end

function sysCall_actuation()
	-- put your actuation code here
end

function sysCall_sensing()
	-- put your sensing code here
	if not prev then
		prev = getPositions(quadHandles)
	end
	local curr = getPositions(quadHandles)
	for i, dr in pairs(drawingHandles) do
		for j = 1, 3 do
			curr[i][3 + j] = prev[i][j]
		end
		sim.addDrawingObjectItem(dr, curr[i])
	end
	prev = getPositions(quadHandles)
end

function sysCall_cleanup()
	-- do some clean-up here
end

-- See the user manual or the available code snippets for additional callback functions and details
