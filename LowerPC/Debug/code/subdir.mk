################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
"../code/AnoScope.c" \
"../code/CH100.c" \
"../code/MPU6050_DMP.c" \
"../code/Serial.c" \
"../code/balance.c" \
"../code/circular_buffer.c" \
"../code/flash.c" \
"../code/imu.c" \
"../code/init.c" \
"../code/key.c" \
"../code/lane_change.c" \
"../code/odrive.c" \
"../code/pid_pro.c" \
"../code/servo.c" \
"../code/time.c" 

COMPILED_SRCS += \
"code/AnoScope.src" \
"code/CH100.src" \
"code/MPU6050_DMP.src" \
"code/Serial.src" \
"code/balance.src" \
"code/circular_buffer.src" \
"code/flash.src" \
"code/imu.src" \
"code/init.src" \
"code/key.src" \
"code/lane_change.src" \
"code/odrive.src" \
"code/pid_pro.src" \
"code/servo.src" \
"code/time.src" 

C_DEPS += \
"./code/AnoScope.d" \
"./code/CH100.d" \
"./code/MPU6050_DMP.d" \
"./code/Serial.d" \
"./code/balance.d" \
"./code/circular_buffer.d" \
"./code/flash.d" \
"./code/imu.d" \
"./code/init.d" \
"./code/key.d" \
"./code/lane_change.d" \
"./code/odrive.d" \
"./code/pid_pro.d" \
"./code/servo.d" \
"./code/time.d" 

OBJS += \
"code/AnoScope.o" \
"code/CH100.o" \
"code/MPU6050_DMP.o" \
"code/Serial.o" \
"code/balance.o" \
"code/circular_buffer.o" \
"code/flash.o" \
"code/imu.o" \
"code/init.o" \
"code/key.o" \
"code/lane_change.o" \
"code/odrive.o" \
"code/pid_pro.o" \
"code/servo.o" \
"code/time.o" 


# Each subdirectory must supply rules for building sources it contributes
"code/AnoScope.src":"../code/AnoScope.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/AnoScope.o":"code/AnoScope.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/CH100.src":"../code/CH100.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/CH100.o":"code/CH100.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/MPU6050_DMP.src":"../code/MPU6050_DMP.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/MPU6050_DMP.o":"code/MPU6050_DMP.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/Serial.src":"../code/Serial.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/Serial.o":"code/Serial.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/balance.src":"../code/balance.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/balance.o":"code/balance.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/circular_buffer.src":"../code/circular_buffer.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/circular_buffer.o":"code/circular_buffer.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/flash.src":"../code/flash.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/flash.o":"code/flash.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/imu.src":"../code/imu.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/imu.o":"code/imu.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/init.src":"../code/init.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/init.o":"code/init.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/key.src":"../code/key.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/key.o":"code/key.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/lane_change.src":"../code/lane_change.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/lane_change.o":"code/lane_change.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/odrive.src":"../code/odrive.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/odrive.o":"code/odrive.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/pid_pro.src":"../code/pid_pro.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/pid_pro.o":"code/pid_pro.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/servo.src":"../code/servo.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/servo.o":"code/servo.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"
"code/time.src":"../code/time.c" "code/subdir.mk"
	cctc -cs --dep-file="$*.d" --misrac-version=2004 -D__CPU__=tc37x "-fC:/Users/29441/Desktop/车赛/室外专项-室外无人驾驶自行车/autonomous_bicycle_workspace/TC377_Win/Debug/TASKING_C_C___Compiler-Include_paths__-I_.opt" --iso=99 --c++14 --language=+volatile --exceptions --anachronisms --fp-model=3 -O0 --tradeoff=4 --compact-max-size=200 -g -Wc-w544 -Wc-w557 -Ctc37x -Y0 -N0 -Z0 -o "$@" "$<"
"code/time.o":"code/time.src" "code/subdir.mk"
	astc -Og -Os --no-warnings= --error-limit=42 -o  "$@" "$<"

clean: clean-code

clean-code:
	-$(RM) ./code/AnoScope.d ./code/AnoScope.o ./code/AnoScope.src ./code/CH100.d ./code/CH100.o ./code/CH100.src ./code/MPU6050_DMP.d ./code/MPU6050_DMP.o ./code/MPU6050_DMP.src ./code/Serial.d ./code/Serial.o ./code/Serial.src ./code/balance.d ./code/balance.o ./code/balance.src ./code/circular_buffer.d ./code/circular_buffer.o ./code/circular_buffer.src ./code/flash.d ./code/flash.o ./code/flash.src ./code/imu.d ./code/imu.o ./code/imu.src ./code/init.d ./code/init.o ./code/init.src ./code/key.d ./code/key.o ./code/key.src ./code/lane_change.d ./code/lane_change.o ./code/lane_change.src ./code/odrive.d ./code/odrive.o ./code/odrive.src ./code/pid_pro.d ./code/pid_pro.o ./code/pid_pro.src ./code/servo.d ./code/servo.o ./code/servo.src ./code/time.d ./code/time.o ./code/time.src

.PHONY: clean-code

