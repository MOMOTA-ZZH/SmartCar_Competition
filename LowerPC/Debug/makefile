################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include user/subdir.mk
-include libraries/zf_driver/subdir.mk
-include libraries/zf_device/subdir.mk
-include libraries/zf_components/subdir.mk
-include libraries/zf_common/subdir.mk
-include libraries/infineon_libraries/subdir.mk
-include code/subdir.mk
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++M_DEPS)),)
-include $(C++M_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(CCM_DEPS)),)
-include $(CCM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXXM_DEPS)),)
-include $(CXXM_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

OPTIONAL_TOOL_DEPS := \
$(wildcard ../makefile.defs) \
$(wildcard ../makefile.init) \
$(wildcard ../makefile.targets) \


BUILD_ARTIFACT_NAME := TC377_Win
BUILD_ARTIFACT_EXTENSION := elf
BUILD_ARTIFACT_PREFIX :=
BUILD_ARTIFACT := $(BUILD_ARTIFACT_PREFIX)$(BUILD_ARTIFACT_NAME)$(if $(BUILD_ARTIFACT_EXTENSION),.$(BUILD_ARTIFACT_EXTENSION),)

# Add inputs and outputs from these tool invocations to the build variables 
SECONDARY_SIZE += \
TC377_Win.siz \


# All Target
all: main-build

# Main-build Target
main-build: TC377_Win.elf secondary-outputs

# Tool invocations
TC377_Win.elf: $(OBJS) makefile $(OPTIONAL_TOOL_DEPS) .TC377_Win.elf.opt
	@echo 'Building target: $@'
	@echo 'Invoking: TASKING Linker'
	cctc -f .TC377_Win.elf.opt
	@echo 'Finished building target: $@'
	@echo ' '

.TC377_Win.elf.opt:
	@argfile .TC377_Win.elf.opt --lsl-file="../Lcf_Tasking_Tricore_Tc.lsl" -Wl-Oc -Wl-OL -Wl-Ot -Wl-Ox -Wl-Oy "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\zf_device\zf_device_config.a" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxDmu_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxAsclin_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCan_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Cfg_Ssw.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Cfg_SswBmhd.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\CompilerTasking.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\CompilerGnuc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\CompilerGhs.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Ssw_Tc0.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\CompilerDcc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Ssw_Tc2.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Ssw_Infra.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Ssw_Tc1.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\SpiIf.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxStdIf_Timer.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxStdIf_PwmHl.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxStdIf_Pos.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxStdIf_DPipe.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Bsp.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Assert.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_WndF32_HannTable.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_WndF32_BlackmanHarrisTable.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_RampF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LutSincosF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LutSincosF32_Table.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LutLinearF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LutAtan2F32_Table.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LutLSincosF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_IntegralF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LutAtan2F32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_LowPassPt1F32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_FftF32_TwiddleTable.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_FftF32_BitReverseTable.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxAsclin_Asc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_FftF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_AngleTrkF32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Crc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Cf32.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxAsclin.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCcu6.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCcu6_Timer.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCpu.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCpu_Irq.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxDma_Dma.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEvadc_Adc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxDma.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEvadc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGpt12_IncrEnc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxFlash.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGpt12.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Atom_Pwm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Tom.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Tim.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Tbu.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Spe.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Psm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Dtm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Dpll.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Atom.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPmsPm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxMtu.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_Cmu.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxQspi_SpiMaster.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPort.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPmsEvr.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScuWdt.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxQspi.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScuLbist.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScuCcu.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScuEru.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScuRcu.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxSrc.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxStm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxStm_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxQspi_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxSmu_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxSent_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScu_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPsi5_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxSrc_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxMtu_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPort_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxHssl_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxI2c_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGeth_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxMsc_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxFlash_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEvadc_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxDma_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEray_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCcu6_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCpu_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCan_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxAsclin_cfg.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_Fifo.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxSmu_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_CircularBuffer.asm.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\Ifx_CircularBuffer.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxSent_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGtm_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxScu_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxQspi_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPsi5s_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPsi5_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPms_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxPort_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGeth_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxMsc_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxI2c_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxGpt12_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEvadc_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEdsadc_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxEray_PinMap.o" "C:\Users\29441\Desktop\车赛\室外专项-室外无人驾驶自行车\autonomous_bicycle_workspace\TC377_Win\libraries\infineon_libraries\IfxCcu6_PinMap.o" -Wl--map-file="TC377_Win.map" -Wl-mc -Wl-mf -Wl-mi -Wl-mk -Wl-ml -Wl-mm -Wl-md -Wl-mr -Wl-mu --no-warnings= -Wl--error-limit=42 --fp-model=3 -lcs_fpu -lfp_fpu -lrt --lsl-core=vtc --exceptions --strict --anachronisms --force-c++ -Ctc37x -o"TC377_Win.elf" -Wl-o"TC377_Win.hex:IHEX" $(OBJS)

.PHONY: .TC377_Win.elf.opt

TC377_Win.siz: TC377_Win.elf makefile $(OPTIONAL_TOOL_DEPS)
	@echo 'Invoking: Print Size'
	elfsize  "TC377_Win.elf"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) TC377_Win.siz TC377_Win.elf
	-@echo ' '

secondary-outputs: $(SECONDARY_SIZE)

.PHONY: all clean dependents main-build

-include ../makefile.targets
