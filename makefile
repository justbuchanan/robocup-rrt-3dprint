build/rrtgen: main.cpp CMakeLists.txt makefile rrt
	mkdir -p build
	cd build && cmake .. -DCMAKE_INSTALL_PREFIX:PATH="" -GNinja && ninja

out.scad: build/rrtgen
	build/rrtgen

out.stl: out.scad main.scad
	openscad -o out.stl main.scad

clean:
	rm -rf build
