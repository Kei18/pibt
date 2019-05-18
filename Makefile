#!/bin/make
DIR = ./src/
PROGRAM = ./bin/testapp
MAIN = $(DIR)testapp.cpp
SRC = $(MAIN) $(wildcard $(DIR)*/*.cpp)
STATUS_FILE = $(DIR)dummy.h
CC = g++
CFLAGS = -Wall -std=c++11 -Os -march=native
LDFLAGS =
OF_INCLUDE = $(OF_ROOT)/libs/openFrameworksCompiled/project/makefileCommon/compile.project.mk

ifdef OF_ROOT
	include $(OF_INCLUDE)
endif

# compile without visualization
.PHONY: c
c:
	@if [ -e  $(STATUS_FILE) ]; then rm $(STATUS_FILE); fi
	@touch $(STATUS_FILE)
	$(CC) $(CFLAGS) $(LDFLAGS) $(SRC) -o $(PROGRAM)
	@rm $(STATUS_FILE)

# compile with openFrameworks
.PHONY: of
of:
	@if [ -e  $(STATUS_FILE) ]; then rm $(STATUS_FILE); fi
	@touch $(STATUS_FILE)
	@echo \#define OF 1 >> $(STATUS_FILE)
	$(MAKE)
	@rm $(STATUS_FILE)

# implement without visualization
.PHONY: crun
crun:
	$(PROGRAM) -p ${param}

# implement with openFrameworks
.PHONY: ofrun
ofrun:
	@./bin/$(APPNAME).app/Contents/MacOS/$(APPNAME) ${file} -p ${param}

.PHONY: clean
clean:
	rm -rf ./bin/$(APPNAME).app
	rm -f $(PROGRAM)
