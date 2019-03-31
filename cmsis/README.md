These files come from the STM32CubeMX F1 V1.7

I changed the file directory structure to make it easier to
include with one -I arg for gcc and I also created an empty
system_stm32f1xx.h.

The stm32f103xb.h includes system_stm32f1xx.h, which we don't use,
so I just made an empty one.

