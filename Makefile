mercury2_prog: mercury2_prog.c
	gcc -O3 -o bin/mercury2_prog mercury2_prog.c d2xx_linux/libftd2xx.a -pthread -ldl -Id2xx_linux
	x86_64-w64-mingw32-gcc-7.3-win32 -O3 -o bin/mercury2_prog_x86-64.exe mercury2_prog.c d2xx_win64/ftd2xx.lib -Id2xx_win64
	i686-w64-mingw32-gcc-7.3-win32   -O3 -o bin/mercury2_prog_x86-32.exe mercury2_prog.c d2xx_win32/ftd2xx.lib -Id2xx_win32
clean: 
	rm -fv bin/mercury2_prog*
