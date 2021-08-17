# Uncomment lines below if you have problems with $PATH
#SHELL := /bin/bash
#PATH := /usr/local/bin:$(PATH)

all:
		platformio  run

upload:
		platformio  run --target upload

clean:
		platformio  run --target clean

update:
		platformio  update
