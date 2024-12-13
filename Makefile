
build:
	rm -rfd bin/
	mkdir bin/
	go build -o bin/picassim

install: build
	sudo cp bin/picassim /usr/local/bin