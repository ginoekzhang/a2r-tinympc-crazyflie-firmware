cmd_src/modules/src/stabilizer.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/.stabilizer.o.d    -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src -Isrc/modules/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/platform/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/cpx   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/controller   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/estimator   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/kve   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/FatFS   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/include/Eigen   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/src -DEIGEN_INITIALIZE_MATRICES_BY_ZERO -DEIGEN_NO_MALLOC -DNDEBUG -DEIGEN_FAST_MATH   -c -o src/modules/src/stabilizer.o /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/stabilizer.c

source_src/modules/src/stabilizer.o := /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/src/stabilizer.c

deps_src/modules/src/stabilizer.o := \
    $(wildcard include/config/log/motor/cap/warning.h) \
    $(wildcard include/config/deck/usd.h) \
    $(wildcard include/config/motors/esc/protocol/dshot.h) \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/math.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
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
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/system.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/param.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/param_logic.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/crtp.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/config.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/console.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/eprintf.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/motors.h \
    $(wildcard include/config/motors/esc/protocol/oneshot125.h) \
    $(wildcard include/config/motors/esc/protocol/oneshot42.h) \
    $(wildcard include/config/motors/dshot/pwm/150khz.h) \
    $(wildcard include/config/motors/dshot/pwm/300khz.h) \
    $(wildcard include/config/motors/dshot/pwm/600khz.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/stm32fxxx.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/pm.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/adc.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/syslink.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_core.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/imu_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_constants.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_digital.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_analog.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_spi.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/string.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_locale.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/strings.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/string.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/platform/interface/platform.h \
    $(wildcard include/config/sensors/bmi088/bmp3xx.h) \
    $(wildcard include/config/sensors/bmi088/spi.h) \
    $(wildcard include/config/sensors/mpu9250/lps25h.h) \
    $(wildcard include/config/sensors/bosch.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/stabilizer.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/sensors.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/commander.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/crtp_commander_high_level.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/math3d.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/crtp_localization_service.h \
    $(wildcard include/config/deck/lighthouse.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/pulse_processor.h \
    $(wildcard include/config/deck/lighthouse/max/n/bs.h) \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/stdlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/alloca.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/ootx_decoder.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_calibration.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_geometry.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/cf_math.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math_types.h \
    $(wildcard include/config/tables.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
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
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/controller/controller.h \
    $(wildcard include/config/controller/oot.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/power_distribution.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/collision_avoidance.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/health.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/supervisor.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/interface/usddeck.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/quatcompress.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/statsCnt.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/static_mem.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/rateSupervisor.h \

src/modules/src/stabilizer.o: $(deps_src/modules/src/stabilizer.o)

$(deps_src/modules/src/stabilizer.o):
