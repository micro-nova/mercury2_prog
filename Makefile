mercury2_prog: mercury2_prog.c
	gcc -O3 -o bin/mercury2_prog mercury2_prog.c d2xx_linux/libftd2xx.a -pthread -ldl -Id2xx_linux
	test -x $(command -v x86_64-w64-mingw32-gcc-win32) && x86_64-w64-mingw32-gcc-win32 -O3 -o bin/mercury2_prog.exe mercury2_prog.c d2xx_win/ftd2xx.lib -Id2xx_win || echo "WARNING: missing x86_64-w64-mingw32-gcc-win32 for compiling windows executable"
clean: 
	rm -f bin/mercury2_prog
