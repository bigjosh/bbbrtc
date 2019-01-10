.PHONY: all
all: bbbrtc install

bbbrtc: bbbrtc.c
	g++ bbbrtc.c -o bbbrtc
	chmod +x bbbrtc

.PHONY: install
install: bbbrtc
	mkdir -p $(DESTDIR)/usr/bin
	sudo install bbbrtc /usr/bin/
	rm bbbrtc
