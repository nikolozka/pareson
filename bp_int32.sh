g++ -g -I./  pareson_int32.cpp -DOSC_DETECT_ENDIANESS -c -o objects/pareson_int32.o

g++ -Wl,-O1 -o build/pareson_int32 objects/pareson_int32.o objects/RTMath.o objects/RTIMUHal.o objects/RTFusion.o objects/RTFusionKalman4.o objects/RTFusionRTQF.o objects/RTIMUSettings.o objects/RTIMUAccelCal.o objects/RTIMUMagCal.o objects/RTIMU.o objects/RTIMUNull.o objects/RTIMUMPU9150.o objects/RTIMUMPU9250.o objects/RTIMUGD20HM303D.o objects/RTIMUGD20M303DLHC.o objects/RTIMUGD20HM303DLHC.o objects/RTIMULSM9DS0.o objects/RTIMULSM9DS1.o objects/RTIMUBMX055.o objects/RTIMUBNO055.o objects/RTPressure.o objects/RTPressureBMP180.o objects/RTPressureLPS25H.o objects/RTPressureMS5611.o objects/RTPressureMS5637.o objects/OscTypes.o  objects/OscOutboundPacketStream.o objects/UdpSocket.o objects/IpEndpointName.o objects/NetworkingUtils.o -L/usr/lib/arm-linux-gnueabihf -lncurses -lportaudio -lasound -pthread -lsndfile libResonanceAudioStatic.a


