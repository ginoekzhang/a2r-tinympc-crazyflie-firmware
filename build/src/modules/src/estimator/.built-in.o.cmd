cmd_src/modules/src/estimator/built-in.o :=  arm-none-eabi-g++ --specs=nosys.specs --specs=nano.specs -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -nostdlib   -r -o src/modules/src/estimator/built-in.o src/modules/src/estimator/estimator_complementary.o src/modules/src/estimator/estimator_kalman.o src/modules/src/estimator/estimator.o src/modules/src/estimator/position_estimator_altitude.o
