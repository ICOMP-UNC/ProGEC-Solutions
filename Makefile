.PHONY: all build upload monitor clean

all: build

build:
    pio run

upload:
    pio run --target upload

monitor:
    pio device monitor

clean:
    pio run --target clean