cmd_src/modules/src/outlierfilter/built-in.o :=  arm-none-eabi-g++ --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib   -r -o src/modules/src/outlierfilter/built-in.o src/modules/src/outlierfilter/outlierFilterTdoa.o src/modules/src/outlierfilter/outlierFilterLighthouse.o
