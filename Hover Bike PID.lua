-- VERSION:: "Hover Bike PID"

if STARTED ~= nil then
	if input() then
		local ran, err = pcall(onTick)

		if not ran then
			onError()
			error(err)
		end
	else
		onStop()
	end
else
	-- CALIBRATION SETTINGS --
	EQUALI = 75  	  				   -- For equalibrium val, recalibrate when changing weight
	DESIRED_OFF = 20  				   -- Desired offset to achieve from ground
	CONTROL_GAIN = 20 				   -- Multiplier for W/S Controls, I'd say max 50
	SIDEWAYS_HEIGHT_CORRECTION = true  -- Increase offset when travelling sideways
	SIDEWAYS_HEIGHT_MULT = 0.15		   -- Multiplier for said^ offset
	AUTOPILOT_YAW_CORRECTION = true	   -- Attempt to keep pointing one direction in autopilot
	AUTOPILOT_YAW_WINDOW = 7 		   -- Amount of degrees it can rotate in each direction before correction
	
	-- NORMAL VALS --
	Kp_all   = 2.00 -- ALL sensitivity
	Kd_all   = 2.50 -- ALL damping
	
	Kp_pitch = 2.00 -- PITCH sensitivity
	Ki_pitch = 0.50 -- PITCH integral
	Kd_pitch = 0.50 -- PITCH damping

	Kp_roll  = 1.00 -- ROLL sensitivity
	Ki_roll  = 0.50 -- ROLL integral
	Kd_roll  = 0.20 -- ROLL damping
	
	Kp_VELO_X  = 1.50 -- VELOCITY X sensitivity
	
	-- AUTOPILOT VALS --
	Ap_Kp_pitch = 3.50 -- PITCH sensitivity
	Ap_Ki_pitch = 1.00 -- PITCH integral
	Ap_Kd_pitch = 0.50 -- PITCH damping
	
	Ap_Kp_roll  = 2.00 -- ROLL sensitivity
	Ap_Ki_roll  = 1.00 -- ROLL integral
	Ap_Kd_roll  = 0.25 -- ROLL damping
	
	Ap_Kp_VELO_X  = 6.00 -- VELOCITY X (ROLL) sensitivity
	Ap_Kp_VELO_Y  = 4.00 -- VELOCITY Y (PITCH) sensitivity
	
	
	-- DO NOT TOUCH --
	TICKSPEED = 0.050 -- 2x tickspeed in an attempt to reduce lag
	AUTOPILOT = -1
	DEBUG = false
	ALTITUDE = 0
	CUR_MIN_OFFSET = 0
	PITCH_ROT = 0
	ROLL_ROT = 0
	YAW_ROT = 0		 -- If AUTOPILOT_YAW_CORRECTION is on
	LAST_YAW_ROT = 0 -- If AUTOPILOT_YAW_CORRECTION is on
	VELO_X = 0
	SEAT = SEAT or getComponent("wasd")
	WATER_OVERRIDE = 0
	THRUST_ALL = 0
	THRUST_FL = 0
	THRUST_FR = 0
	THRUST_BL = 0
	THRUST_BR = 0
	
	prev_pitch_err = 0
	regular_pitch_int = 0
	regular_pitch_int_max = 50
	autopil_pitch_int = 0
	autopil_pitch_int_max = 100
	prev_roll_err = 0
	regular_roll_int = 0
	regular_roll_int_max = 50
	autopil_roll_int = 0
	autopil_roll_int_max = 100
	prev_all_err = 0

	function onStart() -- Invokes when computer turns on
		clearregs()
		SEAT = getComponent("wasd")
	end

	function onTick() -- Invokes every tick computer is active
		-- Get var values
		getVars()
		
		-- ALL THRUSTERS
		THRUST_ALL = calcAllErr()
		
		-- PITCH
		local pitch_adjustment = calcPitchErr()

		-- ROLL
		local roll_adjustment = calcRollErr()
		
		-- Adjust thrusters
		THRUST_FL = clamp(THRUST_ALL + pitch_adjustment + roll_adjustment, 0, 250) --  - ws_adjustment
		THRUST_FR = clamp(THRUST_ALL + pitch_adjustment - roll_adjustment, 0, 250) --  - ws_adjustment
		THRUST_BL = clamp(THRUST_ALL - pitch_adjustment + roll_adjustment, 0, 250) --  + ws_adjustment
		THRUST_BR = clamp(THRUST_ALL - pitch_adjustment - roll_adjustment, 0, 250) --  + ws_adjustment
		
		if DEBUG then
			if AUTOPILOT == 0 then
				print("KI_ROLL : " .. regular_roll_int)
				print("KI_PITCH: " .. regular_pitch_int)
			else
				print("KI_ROLL : " .. autopil_roll_int)
				print("KI_PITCH: " .. autopil_pitch_int)
			end
		end
		
		-- Set Thrusters
		setreg("thrusterFL", THRUST_FL)
		setreg("thrusterFR", THRUST_FR)
		setreg("thrusterBL", THRUST_BL)
		setreg("thrusterBR", THRUST_BR)
		
		setYawRegs()
	end
	
	function setYawRegs()
		if AUTOPILOT == 0 then
			setreg("thrusterYL", (SEAT.getADvalue() == 1) and 1 or 0)
			setreg("thrusterYR", (SEAT.getADvalue() == -1) and 1 or 0)
		elseif AUTOPILOT_YAW_CORRECTION then
			-- Calculate normalized yaw difference
			local yaw_diff = YAW_ROT - LAST_YAW_ROT
			yaw_diff = (yaw_diff > 180) and (yaw_diff - 360) or (yaw_diff < -180) and (yaw_diff + 360) or yaw_diff
			
			-- Apply
			local yaw_correction = (yaw_diff > AUTOPILOT_YAW_WINDOW) and 1 or (yaw_diff < -AUTOPILOT_YAW_WINDOW) and -1 or 0
			setreg("thrusterYL", (yaw_correction == 1) and 1 or 0)
			setreg("thrusterYR", (yaw_correction == -1) and 1 or 0)
		else
			setreg("thrusterYL", 0)
			setreg("thrusterYR", 0) 
		end
	end
	
	function calcRollErr() -- Calculate error for roll control
		if AUTOPILOT == 0 then
			local desired_roll = VELO_X * Kp_VELO_X
			local roll_err = -desired_roll - ROLL_ROT
			local roll_derivative = (roll_err - prev_roll_err) / TICKSPEED
			prev_roll_err = roll_err
			
			regular_roll_int = clamp(regular_roll_int + (roll_err * TICKSPEED), -regular_roll_int_max, regular_roll_int_max)
			
			return (Kp_roll * roll_err) + (Kd_roll * roll_derivative) + (Ki_roll * regular_roll_int)					
		else
			local desired_roll = VELO_X * Ap_Kp_VELO_X
			local roll_err = -desired_roll - ROLL_ROT
			local roll_derivative = (roll_err - prev_roll_err) / TICKSPEED
			prev_roll_err = roll_err 
			
			autopil_roll_int = clamp(autopil_roll_int + (roll_err * TICKSPEED), -autopil_roll_int_max, autopil_roll_int_max)
			
			return (Ap_Kp_roll * roll_err) + (Ap_Kd_roll * roll_derivative) + (Ap_Ki_roll * autopil_roll_int)
		end
	end
	
	function calcPitchErr() -- Calculate error for pitch control
		if AUTOPILOT == 0 then
			local pitch_err = PITCH_ROT - (SEAT.getWSvalue() * CONTROL_GAIN)
			local pitch_derivative = (pitch_err - prev_pitch_err) / TICKSPEED
			prev_pitch_err = pitch_err
			
			regular_pitch_int = clamp(regular_pitch_int + (pitch_err * TICKSPEED), -regular_pitch_int_max, regular_pitch_int_max)
			
			return (Kp_pitch * pitch_err) + (Kd_pitch * pitch_derivative) + (Ki_pitch * regular_pitch_int)
		else
			local desired_pitch = VELO_Y * Ap_Kp_VELO_Y
			local pitch_err = -desired_pitch + PITCH_ROT
			local pitch_derivative = (pitch_err - prev_pitch_err) / TICKSPEED
			prev_pitch_err = pitch_err
			
			autopil_pitch_int = clamp(autopil_pitch_int + (pitch_err * TICKSPEED), -autopil_pitch_int_max, autopil_pitch_int_max)
			
			return (Ap_Kp_pitch * pitch_err) + (Ap_Kd_pitch * pitch_derivative) + (Ap_Ki_pitch * autopil_pitch_int)
		end
	end
	
	function calcAllErr()
		-- Calculate error for all thrusters, up/down
		-- local all_err = DESIRED_OFF - CUR_MIN_OFFSET -- NO WATER ADAPT
		
		local all_err = 0
		--WATER ADAPTATION, SLOPPY BUT WHATEVER
		--if (CUR_MIN_OFFSET - ALTITUDE) > 12 then
		--	if AUTOPILOT == 1 then
		--		all_err = 1 - ALTITUDE - 9
		--	else
		--		all_err = 1 - ALTITUDE + (DESIRED_OFF / 2)
		--	end
		--else
		--	if AUTOPILOT == 0 then
		--		if SIDEWAYS_HEIGHT_CORRECTION then
		--			all_err = (DESIRED_OFF + math.abs(VELO_X * SIDEWAYS_HEIGHT_MULT)) - CUR_MIN_OFFSET
		--		else
		--			all_err = DESIRED_OFF - CUR_MIN_OFFSET
		--		end
		--	else
		--		all_err = 3 - CUR_MIN_OFFSET
		--	end
		--end
		
		-- ChatBBC generated
		local altitude_diff = CUR_MIN_OFFSET - ALTITUDE
		local base_err = (AUTOPILOT == 1) and (1 - ALTITUDE - 10) or (1 - ALTITUDE + (DESIRED_OFF / 2))

		if altitude_diff > 12 and WATER_OVERRIDE == 0 then
			all_err = base_err
		else
			local sideways_correction = (SIDEWAYS_HEIGHT_CORRECTION and (math.abs(VELO_X * SIDEWAYS_HEIGHT_MULT))) or 0
			local desired_offset = (AUTOPILOT == 1) and 1 or (DESIRED_OFF + sideways_correction)
			all_err = desired_offset - CUR_MIN_OFFSET
		end
		-- End ChatBBC generation
		
		local all_derivative = (all_err - prev_all_err) / TICKSPEED  -- Calculate the rate of change
		prev_all_err = all_err  -- Update for next frame
		return EQUALI + (Kp_all * all_err) + (Kd_all * all_derivative)
	end
	
	function getVars()
		AUTOPILOT = getreg("autopilot")	  -- AUTOPILOT SWITCH STATE
		ALTITUDE = getreg("altitude") 	  -- YOULL NEVER GUESS
		CUR_MIN_OFFSET = math.min(getreg("frontOffset"), getreg("backOffset"))
		PITCH_ROT = getreg("pitchOffset") -- PITCH ROTATION
		ROLL_ROT = getreg("rollOffset")   -- ROLL ROTATION
		YAW_ROT = getreg("yawOffset")     -- YAW ROTATION
		VELO_X = getreg("veloX")		  -- VELOCITY X
		VELO_Y = getreg("veloY")		  -- VELOCITY Y
		WATER_OVERRIDE = getreg("water")  -- WATER OVERRIDE
		
		-- For AUTOPILOT_YAW_CORRECTION
		-- Can be used to toggle things on button press
		if AUTOPILOT == 1 and LAST_YAW_ROT == 0 then
			LAST_YAW_ROT = YAW_ROT
			autopil_pitch_int = 0
			autopil_roll_int = 0
			regular_pitch_int = 0
			regular_roll_int = 0

		elseif AUTOPILOT == 0 then
			LAST_YAW_ROT = 0
		end
	end

	function onStop()
		clearregs()
	end

	function onError()
		onStop()
	end
	
	function clamp(num, min, max)
		return math.max(min, math.min(num, max))
	end

	onStart()
	STARTED = true
end
