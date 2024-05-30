pyinstall:
	@python3 -m pip install --upgrade pip
	@pip install numpy
	@pip install --verbose ./python/

pyinstall_with_demo: pyinstall
	@pip install open3d==0.18.0

cppinstall:
	@cmake -Bcpp/build cpp/
	@cmake --build cpp/build -j$(nproc --all)

cppinstall_with_demo:
	@cmake -Bcpp/build cpp/ -DINCLUDE_CPP_EXAMPLES=ON
	@cmake --build cpp/build -j$(nproc --all)