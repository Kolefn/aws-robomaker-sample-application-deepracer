.PHONY: all setup build build_simulation bundle bundle_simulation clean clean_simulation_build clean_simulation_bundle
.DEFAULT_GOAL := all

all: bundle

# This forces each step in 'ci' to run in serial, but each step will run all of its commands in parallel
ci:
	$(MAKE) setup
	$(MAKE) build
	$(MAKE) bundle

setup:
	scripts/setup.sh

build: build_simulation

build_simulation:
	scripts/build.sh ./workspace

bundle: bundle_simulation

bundle_simulation: build_simulation
	scripts/bundle.sh ./workspace

clean: clean_simulation_build clean_simulation_bundle

clean_simulation_build:
	rm -rf ./workspace/build ./workspace/install

clean_simulation_bundle:
	rm -rf ./workspace/bundle
