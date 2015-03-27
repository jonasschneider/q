## Make sure to run `make` within `controller/build` before trying this
pylib: ../downlink.proto
	rm -fr $@~
	mkdir $@~
	protoc --python_out=$@~ -I../ -I../controller/build/nanopb/generator/proto $<
	mv $@~ $@
