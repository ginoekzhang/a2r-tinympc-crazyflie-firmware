cmd_src/deck/drivers/src/oa.o := arm-none-eabi-gcc -Wp,-MD,src/deck/drivers/src/.oa.o.d    -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/src -Isrc/deck/drivers/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/platform/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/cpx   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/controller   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/estimator   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/kve   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/FatFS   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/include/Eigen   -I/Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/TinyMPC/src -DEIGEN_INITIALIZE_MATRICES_BY_ZERO -DEIGEN_NO_MALLOC -DNDEBUG -DEIGEN_FAST_MATH   -c -o src/deck/drivers/src/oa.o /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/src/oa.c

source_src/deck/drivers/src/oa.o := /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/drivers/src/oa.c

deps_src/deck/drivers/src/oa.o := \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_core.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/imu_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_constants.h \
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
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_digital.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_analog.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/deck/interface/deck_spi.h \
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
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/param.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/param_logic.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/crtp.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/system.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/trace.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/usec_time.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/console.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/eprintf.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/hal/interface/pca95x4.h \
    $(wildcard include/config/reg.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/vl53l0x.h \
    $(wildcard include/config/gpio.h) \
    $(wildcard include/config/control.h) \
    $(wildcard include/config/min/snr.h) \
    $(wildcard include/config/valid/phase/low.h) \
    $(wildcard include/config/valid/phase/high.h) \
    $(wildcard include/config/min/count/rate/rtn/limit.h) \
    $(wildcard include/config/sigma/thresh/hi.h) \
    $(wildcard include/config/sigma/thresh/lo.h) \
    $(wildcard include/config/vcsel/period.h) \
    $(wildcard include/config/timeout/macrop/hi.h) \
    $(wildcard include/config/timeout/macrop/lo.h) \
    $(wildcard include/config/initial/phase/select.h) \
    $(wildcard include/config/readout/ctrl.h) \
    $(wildcard include/config/timeout/macrop.h) \
    $(wildcard include/config/vcsel/width.h) \
    $(wildcard include/config/spad/enables/ref/0.h) \
    $(wildcard include/config/spad/enables/ref/1.h) \
    $(wildcard include/config/spad/enables/ref/2.h) \
    $(wildcard include/config/spad/enables/ref/3.h) \
    $(wildcard include/config/spad/enables/ref/4.h) \
    $(wildcard include/config/spad/enables/ref/5.h) \
    $(wildcard include/config/ref/en/start/select.h) \
    $(wildcard include/config/pad/scl/sda//extsup/hv.h) \
    $(wildcard include/config/timeout.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/config/config.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/utils/interface/cfassert.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /Users/char_chen/F24/TinyMPC/a2r-tinympc-crazyflie-firmware/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/stdlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/alloca.h \

src/deck/drivers/src/oa.o: $(deps_src/deck/drivers/src/oa.o)

$(deps_src/deck/drivers/src/oa.o):