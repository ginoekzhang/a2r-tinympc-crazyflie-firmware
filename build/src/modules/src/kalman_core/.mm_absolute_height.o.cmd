cmd_src/modules/src/kalman_core/mm_absolute_height.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/kalman_core/.mm_absolute_height.o.d    -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/kalman_core -Isrc/modules/src/kalman_core -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/platform/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/cpx   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/controller   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/estimator   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/kve   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/FatFS   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/include/Eigen   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/src -DEIGEN_INITIALIZE_MATRICES_BY_ZERO -DEIGEN_NO_MALLOC -DNDEBUG -DEIGEN_FAST_MATH   -c -o src/modules/src/kalman_core/mm_absolute_height.o /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/kalman_core/mm_absolute_height.c

source_src/modules/src/kalman_core/mm_absolute_height.o := /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/kalman_core/mm_absolute_height.c

deps_src/modules/src/kalman_core/mm_absolute_height.o := \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/kalman_core/mm_absolute_height.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/kalman_core/kalman_core.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/cf_math.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math_types.h \
    $(wildcard include/config/tables.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/string.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_locale.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/strings.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/string.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/math.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/float.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include-fixed/limits.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include-fixed/syslimits.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/limits.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/syslimits.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math_memory.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/none.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/utils.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/basic_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math_memory.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/none.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/utils.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/interpolation_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/bayes_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/statistics_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/basic_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/fast_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/matrix_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/complex_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/statistics_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/controller_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/support_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/distance_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/matrix_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/svm_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/svm_defines.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/fast_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/transform_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/complex_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/filtering_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/support_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/quaternion_math_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/dsp/window_functions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/cfassert.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/imu_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \

src/modules/src/kalman_core/mm_absolute_height.o: $(deps_src/modules/src/kalman_core/mm_absolute_height.o)

$(deps_src/modules/src/kalman_core/mm_absolute_height.o):
