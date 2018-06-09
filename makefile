bbbrtc: bbbrtc.c
	g++ bbbrtc.c -o bbbrtc
	chmod +x bbbrtc
	mkdir -p $(DESTDIR)/usr/bin
	sudo install bbbrtc /usr/bin/
	rm bbbrtc
	
