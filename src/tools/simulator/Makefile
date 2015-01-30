
VERSION=$$(cat version)
BUILD_FOLDER=csim-$(VERSION)/build
RM=rm -f

csim:
	test -d $(BUILD_FOLDER) || mkdir $(BUILD_FOLDER)
	cd $(BUILD_FOLDER) && cmake ..
	cd $(BUILD_FOLDER) && make


install:
	cd $(BUILD_FOLDER) && make install

clean:
	@echo "cleaning CAMBADA Simulator build folder, includes, config files, libs and bins"
	$(RM) -r $(BUILD_FOLDER)
	$(RM) -r ../include/gazebo
	$(RM)  ../config/sim.conf.yaml
	$(RM) -r ../config/worlds
	$(RM) -r ../lib/libgazebo* ../bin/csim


