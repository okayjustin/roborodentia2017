# roborodentia2017

To rebuild the newlib libraries (embedded standard C libs):
	cd [REPOSITORY TOP LEVEL DIRECTORY]
	mkdir newlib_arm-none-eabi              					// Or whatever folder name you like
	cd newlib_arm-none-eabi
	../newlib-cygwin/configure
         --target=arm-none-eabi \
         --with-cpu=cortex-m4 \
         --with-fpu=fpv4-sp-d16 \
         --with-float=hard \
         --with-mode=thumb \
         --enable-interwork \        // Not sure if you need this one
         --enable-multilib \         // Not sure if you need this one
         --with-gnu-as \
         --with-gnu-ld \
         --disable-nls \
         --disable-newlib-supplied-syscalls  // Or whatever version of newlib source you have
    sudo make -j4 all

../newlib-cygwin/configure --target=arm-none-eabi --with-cpu=cortex-m4 --with-fpu=fpv4-sp-d16 --with-float=hard --with-mode=thumb --enable-interwork --enable-multilib --with-gnu-as --with-gnu-ld --disable-nls --disable-newlib-supplied-syscalls

Then take the libc.a and libm.a from newlib_arm-none-eabi/arm-none-eabi/newlib and place them in buildTools.
Also take libnosys.a from newlib_arm-none-eabi/arm-none-eabi/thumb/libgloss/libnosys and place it in buildTools.

Then take the libc.a and libm.a from newlib_arm-none-eabi/arm-none-eabi/fpu/newlib and place them in buildTools.
Also take libnosys.a from newlib_arm-none-eabi/arm-none-eabi/fpu/libgloss/libnosys and place it in buildTools.

Debugging tips:
> openocd -f firmware/buildTools/st_nucleo_f4.cfg
> arm-none-eabi-gdb firmware/build/program.elf
(gdb) target remote localhost:3333
(gdb) monitor reset halt
