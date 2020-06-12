g++ -g -I./  parec.cpp -DOSC_DETECT_ENDIANESS -c -o objects/parec.o

g++ -Wl,-O1 -o build/parec objects/pa_unix_util.o objects/pa_ringbuffer.o objects/pa_debugprint.o objects/pa_front.o objects/pa_unix_hostapis.o objects/parec.o -L/usr/lib/arm-linux-gnueabihf -lncurses -lportaudio -lasound -pthread -lsndfile


#~/letsbuild/portaudio/portaudio/src/common/pa_ringbuffer.c
