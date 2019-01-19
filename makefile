.PHONY: all
all: bbbrtc install

bbbrtc: bbbrtc.c
	g++ bbbrtc.c -o bbbrtc
	chmod +x bbbrtc

.PHONY: install
install: bbbrtc
	mkdir -p $(DESTDIR)/usr/sbin
	sudo install bbbrtc $(DESTDIR)/usr/sbin/
	sudo install bbb-long-reset $(DESTDIR)/usr/sbin/
