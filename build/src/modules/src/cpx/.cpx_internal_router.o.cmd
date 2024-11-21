cmd_src/modules/src/cpx/cpx_internal_router.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/cpx/.cpx_internal_router.o.d    -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/cpx -Isrc/modules/src/cpx -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/platform/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/cpx   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/controller   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/estimator   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/kve   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/FatFS   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/include/Eigen   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/src -DEIGEN_INITIALIZE_MATRICES_BY_ZERO -DEIGEN_NO_MALLOC -DNDEBUG -DEIGEN_FAST_MATH   -c -o src/modules/src/cpx/cpx_internal_router.o /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/cpx/cpx_internal_router.c

source_src/modules/src/cpx/cpx_internal_router.o := /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/cpx/cpx_internal_router.c

deps_src/modules/src/cpx/cpx_internal_router.o := \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/trace.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/usec_time.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/cfassert.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/config.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/console.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/eprintf.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/crtp.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/cpx/cpx_internal_router.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/cpx/cpx.h \

src/modules/src/cpx/cpx_internal_router.o: $(deps_src/modules/src/cpx/cpx_internal_router.o)

$(deps_src/modules/src/cpx/cpx_internal_router.o):
