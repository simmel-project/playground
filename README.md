Throw-away code used to test minimum power consumption on Simmel

This replaces the bootloader.  The only interesting file is `main.c`.

To use it, enter this directory and run the following:

```sh
$ git submodule init
$ git submodule update
$ make
$ arm-non-eabi-gdb _build/build-simmel/power-test-nosd.elf -ex 'tar rem your-openocd-server:3333'
(gdb)
```

You can then make changes and reload in gdb:

```
(gdb) make
(gdb) load
(gdb) mon reset halt
(gdb) continue
```

Note that when debugging, the board only does a simulated power off.  So you should conduct tests without the debugger attached.
