#include <iostream> 
#include <sndfile.hh>
#include "portaudio.h"
#include "resonance_audio_api.h"
#include "binaural_surround_renderer.h"
#include "room_effects_utils.cc"
#include "RTIMULib.h"
#include "/usr/local/include/oscpack/osc/OscOutboundPacketStream.h"
#include "/usr/local/include/oscpack/ip/UdpSocket.h"
#include "quaternion.h"

//#define ADDRESS "127.0.0.1"
//#define ADDRESS "192.168.43.230"
//#define ADDRESS "192.168.43.59"
//#define ADDRESS "192.168.188.62"
#define ADDRESS "192.168.188.37"
#define PORT 9000
#define OUTPUT_BUFFER_SIZE 1024

#define PI 3.141592653589793238462643

#define SAMPLE_RATE 48000
#define NUM_SECONDS 500
#define NUM_FRAMES 512
#define NUM_BLOCKS 10
#define BUFFER_LEN NUM_FRAMES*NUM_BLOCKS
//#define BUFFER_LEN SAMPLE_RATE*NUM_SECONDS

RTQuaternion quat;
RTQuaternion quat_s;

float slerp = 0.5;


using namespace vraudio;

typedef struct
{
    float left_phase;
    float right_phase;
} paTestData;

static paTestData data;
const   PaDeviceInfo *deviceInfo;
int dev;

int err;
unsigned int i;

std::size_t quad = 4;
std::size_t stereo = 2;
std::size_t mono = 1;

std::size_t num_frames = NUM_FRAMES;

int n_frames = NUM_FRAMES;
int sample_rate_hz = SAMPLE_RATE;

int smoke_l;
int smoke_r;
int roll;
int fridge;
int heater;
int window;
int outside;
int water;
int yt;
int watch;
int ambi;

float* smoke_l_buff;
float* smoke_r_buff;
float* roll_buff;
float* fridge_buff;
float* heater_buff;
float* window_buff;
float* outside_buff;
float* water_buff;
float* yt_buff;
float* watch_buff;
float* ambi_buff;

float* smoke_l_p;
float* smoke_r_p;
float* roll_p;
float* fridge_p;
float* heater_p;
float* window_p;
float* outside_p;
float* water_p;
float* yt_p;
float* watch_p;
float* ambi_p;

float* src_buffer_ptr;
float* amb_buffer_ptr;

int seekoffset = 1;
int blockcounter = 0;
bool readnext = false;
int bufferoffset = 0;

ResonanceAudioApi* reson;
BinauralSurroundRenderer* renderer;

RenderingMode quality = kBinauralHighQuality;

SNDFILE* roll_file;
SNDFILE* smoke_r_file;
SNDFILE* smoke_l_file;
SNDFILE* fridge_file;
SNDFILE* heater_file;
SNDFILE* window_file;
SNDFILE* outside_file;
SNDFILE* water_file;
SNDFILE* yt_file;
SNDFILE* watch_file;
SNDFILE* ambi_file;

static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData ){

   	reson->SetInterleavedBuffer(smoke_l,smoke_l_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(smoke_r,smoke_r_buff, mono, num_frames);
        reson->SetInterleavedBuffer(roll,roll_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(fridge,fridge_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(heater,heater_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(window,window_buff, mono, num_frames);
        reson->SetInterleavedBuffer(outside,outside_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(water,water_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(yt,yt_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(watch,watch_buff, mono, num_frames);
//	renderer->AddInterleavedInput(ambi_buff,  quad, num_frames);

	if(blockcounter == 1 ){
		readnext=true;
		bufferoffset = 1;
	}
	if(blockcounter == NUM_BLOCKS+1){
		readnext=true;
		bufferoffset = 0;
	}
	if(blockcounter == 2*NUM_BLOCKS){
		blockcounter = 0;
		smoke_l_buff=smoke_l_p;
		smoke_r_buff=smoke_r_p;
		roll_buff=roll_p;
		fridge_buff=fridge_p;
		heater_buff=heater_p;
		window_buff=window_p;
		outside_buff=outside_p;
		water_buff=water_p;
		yt_buff=yt_p;
		watch_buff=watch_p;
		ambi_buff=ambi_p;
	}
	else{
		smoke_l_buff+=num_frames;
		smoke_r_buff+=num_frames;
		roll_buff+=num_frames;
		fridge_buff+=num_frames;
		heater_buff+=num_frames;
		window_buff+=num_frames;
		outside_buff+=num_frames;
		water_buff+=num_frames;
		yt_buff+=num_frames;
		watch_buff+=num_frames;
		ambi_buff+=4*num_frames;
	}
        blockcounter++;
  	reson->FillInterleavedOutputBuffer(stereo,num_frames,src_buffer_ptr);
//	renderer->GetInterleavedStereoOutput(amb_buffer_ptr,num_frames);

    	long *out = (long*)outputBuffer;
    	unsigned int i;
    	(void) inputBuffer; /* Prevent unused variable warning. */
    	for( i=0; i<framesPerBuffer*2; i++ ){
//        	*out++ = amb_buffer_ptr[i++]/2.0;  /* left */
//       	*out++ = amb_buffer_ptr[i]/2.0;/* right */
//        	*out++ = src_buffer_ptr[i++]/2.0;  /* left */
//	       	*out++ = src_buffer_ptr[i]/2.0;/* right */

//        	*out++ = (long)((src_buffer_ptr[i]/1.5 + amb_buffer_ptr[i++]/1.5)*1073741823.0);  /* left */
//        	*out++ = (long)((src_buffer_ptr[i]/1.5  + amb_buffer_ptr[i]/1.5)*1073741823.0);/* right */
        	*out++ = (long)((src_buffer_ptr[i++]/1.5) *1073741823.0);  /* left */
        	*out++ = (long)((src_buffer_ptr[i]/1.5) *1073741823.0);/* right */

//        	*out++ = (long)((src_buffer_ptr[i]/1.5 );  /* left */
//        	*out++ = (long)((src_buffer_ptr[i]/1.5 );/* right */
    	}
    	return 0;
}

int main(int argc, char** argv) {

	const char * fsmoke_l= "resources/smoke_l.wav" ;
	const char * fsmoke_r= "resources/smoke_r.wav" ;
	const char * froll= "resources/roll.wav" ;
	const char * ffridge= "resources/fridge.wav" ;
	const char * fheater= "resources/heater.wav" ;
	const char * fwindow= "resources/window.wav" ;
	const char * foutside= "resources/outside.wav" ;
	const char * fwater= "resources/water.wav" ;
	const char * fyt= "resources/yt.wav" ;
	const char * fwatch= "resources/watch.wav" ;
	const char * fambi= "resources/interstate.wav" ;

	int sampleCount = 0;
    	int sampleRate = 0;

  	uint64_t rateTimer;
    	uint64_t displayTimer;
    	uint64_t now;

	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );
    	char buffer[OUTPUT_BUFFER_SIZE];
    	Quaternion<float> q;

	RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
    	RTIMU *imu = RTIMU::createIMU(settings);

	RoomProperties* properties = new RoomProperties();
	ReflectionProperties* refp = new ReflectionProperties();
	ReverbProperties* revp = new ReverbProperties();

        PaError err;
	PaStream *stream;

  	src_buffer_ptr = (float*) malloc( sizeof(float) * num_frames*2 );
  	amb_buffer_ptr = (float*) malloc( sizeof(float) * num_frames*2 );

  	smoke_l_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	smoke_r_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	roll_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	fridge_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	heater_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	window_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	outside_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	water_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	yt_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	watch_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );

  	ambi_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*8 );

	smoke_l_p = smoke_l_buff;
	smoke_r_p = smoke_r_buff;
	roll_p = roll_buff;
	fridge_p = fridge_buff;
	heater_p = heater_buff;
	window_p = window_buff;
	outside_p = outside_buff;
	water_p = water_buff;
	yt_p = yt_buff;
	watch_p = watch_buff;
	ambi_p = ambi_buff;

	SF_INFO* info = new SF_INFO();

	smoke_l_file = sf_open(fsmoke_l,SFM_READ,info) ;
	smoke_r_file = sf_open(fsmoke_r,SFM_READ,info) ;
	roll_file = sf_open(froll,SFM_READ,info) ;
	fridge_file = sf_open(ffridge,SFM_READ,info) ;
	heater_file = sf_open(fheater,SFM_READ,info) ;
	window_file = sf_open(fwindow,SFM_READ,info) ;
	outside_file = sf_open(foutside,SFM_READ,info) ;
	water_file = sf_open(fwater,SFM_READ,info) ;
	yt_file = sf_open(fyt,SFM_READ,info) ;
	watch_file = sf_open(fwatch,SFM_READ,info) ;

	SF_INFO* quadinfo = new SF_INFO();

	ambi_file  = sf_open(fambi,   SFM_READ,quadinfo) ;
	std::cout<<quadinfo->format<<"\n";
	std::cout<<quadinfo->channels<<"\n";
	std::cout<<quadinfo->samplerate<<"\n";

	sf_read_float(smoke_l_file, smoke_l_buff, BUFFER_LEN*2) ;
	sf_read_float(smoke_r_file, smoke_r_buff, BUFFER_LEN*2) ;
	sf_read_float(roll_file, roll_buff, BUFFER_LEN*2) ;
	sf_read_float(fridge_file, fridge_buff, BUFFER_LEN*2) ;
	sf_read_float(heater_file, heater_buff, BUFFER_LEN*2) ;
	sf_read_float(window_file, window_buff, BUFFER_LEN*2) ;
	sf_read_float(outside_file, outside_buff, BUFFER_LEN*2) ;
	sf_read_float(water_file, water_buff, BUFFER_LEN*2) ;
	sf_read_float(yt_file, yt_buff, BUFFER_LEN*2) ;
	sf_read_float(watch_file, watch_buff, BUFFER_LEN*2) ;

	sf_readf_float(ambi_file,  ambi_buff,  BUFFER_LEN*2) ;

	err=Pa_Initialize();
	if( err != paNoError )
        {
		std::cout<<"how did we get here?";
		printf( "ERROR: Pa_Initialize returned 0x%x\n", err );
                goto error;
        }
	dev =  1;

	deviceInfo = Pa_GetDeviceInfo( dev );

        printf( "--------------------------------------- device #%d\n", dev );

	printf( "Host API                    = %s\n",  Pa_GetHostApiInfo( deviceInfo->hostApi )->name );
        printf( "Max inputs = %d", deviceInfo->maxInputChannels  );
        printf( ", Max outputs = %d\n", deviceInfo->maxOutputChannels  );
        printf( "Default low input latency   = %8.4f\n", deviceInfo->defaultLowInputLatency  );
        printf( "Default low output latency  = %8.4f\n", deviceInfo->defaultLowOutputLatency  );
        printf( "Default high input latency  = %8.4f\n", deviceInfo->defaultHighInputLatency  );
        printf( "Default high output latency = %8.4f\n", deviceInfo->defaultHighOutputLatency  );

	err = Pa_OpenDefaultStream( &stream,
	                                0,          /* no input channels */
        	                        2,          /* stereo output */
                	                paInt32,  /* 32 bit floating point output */
                        	        SAMPLE_RATE,/* sample rate */
                                	n_frames,        /* frames per buffer */
	                                patestCallback, /* this is your callback function */
        	                        &data ); /*This is a pointer that will be passed to your callback*/

        if( err != paNoError ) goto error;
  	reson = CreateResonanceAudioApi(stereo,num_frames,sample_rate_hz);

	renderer = renderer->Create(num_frames,sample_rate_hz,renderer->kFirstOrderAmbisonics);

  	reson->SetMasterVolume(0.7);
  	reson->SetHeadPosition(0,1,0);

  	smoke_l = reson->CreateSoundObjectSource(quality);
  	smoke_r = reson->CreateSoundObjectSource(quality);
  	roll = reson->CreateSoundObjectSource(quality);
  	fridge = reson->CreateSoundObjectSource(quality);
  	heater = reson->CreateSoundObjectSource(quality);
  	window = reson->CreateSoundObjectSource(quality);
  	outside = reson->CreateSoundObjectSource(quality);
  	water = reson->CreateSoundObjectSource(quality);
  	yt = reson->CreateSoundObjectSource(quality);
  	watch = reson->CreateSoundObjectSource(quality);


  	reson->SetSourcePosition(smoke_l,0.1,1,0);
  	reson->SetSourcePosition(smoke_r,-0.1,1,0);
  	reson->SetSourcePosition(roll,0,0.5,-0.3);
  	reson->SetSourcePosition(fridge,0,1,-3);
  	reson->SetSourcePosition(heater,0,1,-5);
  	reson->SetSourcePosition(window,2,1,0);
  	reson->SetSourcePosition(outside,3,1,0);
  	reson->SetSourcePosition(water,2,1,-3);
  	reson->SetSourcePosition(yt,-2,1,-2);
  	reson->SetSourcePosition(watch,-0.3,0.5,0);

    	if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        	std::cout<<"No IMU found\n";
		goto error;
    	}

    	imu->IMUInit();

    	imu->setSlerpPower(slerp);
    	imu->setGyroEnable(true);
    	imu->setAccelEnable(true);
    	imu->setCompassEnable(true);

    	rateTimer = RTMath::currentUSecsSinceEpoch();



	err = Pa_StartStream( stream );
	if( err != paNoError ) goto error;

	

	while(1){

	usleep(imu->IMUGetPollInterval() * 10000);
        while (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
            sampleCount++;

            if(imuData.fusionQPoseValid){
                quat = imuData.fusionQPose;
            }

            q = Quaternion<float>(quat.scalar(), quat.y(), -quat.z(), -quat.x());
            q.Normalize();


           if ((now - rateTimer) > 1000000) {
                sampleRate = sampleCount;
                sampleCount = 0;
                rateTimer = now;
           }
        }

        //q.Print();

        osc::OutboundPacketStream p( buffer, OUTPUT_BUFFER_SIZE );
        p << osc::BeginBundleImmediate

                << osc::BeginMessage( "/w" )
                << (float)(q.Gets())
//              << (float)1
                << osc::EndMessage

                << osc::BeginMessage( "/x" )
                << (float)q.Getx()
//              << (float)0
                << osc::EndMessage

                << osc::BeginMessage( "/y" )
                << (float)(q.Gety())
//              << (float)0
                << osc::EndMessage

                << osc::BeginMessage( "/z" )
                << (float)(q.Getz())
//              << (float)0
                << osc::EndMessage

                << osc::EndBundle;
        transmitSocket.Send( p.Data(), p.Size() );

	reson->SetHeadRotation(q.Getx(),q.Gety(),q.Getz(),q.Gets());
	renderer->SetHeadRotation(q.Gets(),q.Getx(),q.Gety(),q.Getz());

	if(sf_seek(smoke_l_file,1,SEEK_CUR)==-1) sf_seek(smoke_l_file,0,SEEK_SET); else  sf_seek(smoke_l_file,-1,SEEK_CUR);
	if(sf_seek(smoke_r_file,1,SEEK_CUR)==-1) sf_seek(smoke_r_file,0,SEEK_SET); else  sf_seek(smoke_r_file,-1,SEEK_CUR);
	if(sf_seek(roll_file,1,SEEK_CUR)==-1) sf_seek(roll_file,0,SEEK_SET); else  sf_seek(roll_file,-1,SEEK_CUR);
	if(sf_seek(fridge_file,1,SEEK_CUR)==-1) sf_seek(fridge_file,0,SEEK_SET); else  sf_seek(fridge_file,-1,SEEK_CUR);
	if(sf_seek(heater_file,1,SEEK_CUR)==-1) sf_seek(heater_file,0,SEEK_SET); else  sf_seek(heater_file,-1,SEEK_CUR);
	if(sf_seek(window_file,1,SEEK_CUR)==-1) sf_seek(window_file,0,SEEK_SET); else  sf_seek(window_file,-1,SEEK_CUR);
	if(sf_seek(outside_file,1,SEEK_CUR)==-1) sf_seek(outside_file,0,SEEK_SET); else  sf_seek(outside_file,-1,SEEK_CUR);
	if(sf_seek(water_file,1,SEEK_CUR)==-1) sf_seek(water_file,0,SEEK_SET); else  sf_seek(water_file,-1,SEEK_CUR);
	if(sf_seek(yt_file,1,SEEK_CUR)==-1) sf_seek(yt_file,0,SEEK_SET); else  sf_seek(yt_file,-1,SEEK_CUR);
	if(sf_seek(watch_file,1,SEEK_CUR)==-1) sf_seek(watch_file,0,SEEK_SET); else  sf_seek(watch_file,-1,SEEK_CUR);
	if(sf_seek(ambi_file,1,SEEK_CUR)==-1) sf_seek(ambi_file,0,SEEK_SET);   else  sf_seek(ambi_file,-1,SEEK_CUR);

	if(readnext){
		readnext = false;
		sf_read_float(smoke_l_file, smoke_l_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(smoke_r_file, smoke_r_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(roll_file, roll_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(fridge_file, fridge_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(heater_file, heater_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(window_file, window_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(outside_file, outside_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(water_file, water_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(yt_file, yt_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(watch_file, watch_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_readf_float(ambi_file,  ambi_p+bufferoffset*BUFFER_LEN*4,  BUFFER_LEN) ;
		}
	}
	

	//Pa_Sleep(NUM_SECONDS*1000);

	err = Pa_StopStream( stream );
	if( err != paNoError ) goto error;
	err = Pa_CloseStream( stream );
	if( err != paNoError ) goto error;

	sf_close(smoke_l_file);
	sf_close(smoke_r_file);
	sf_close(roll_file);
	sf_close(fridge_file);
	sf_close(heater_file);
	sf_close(window_file);
	sf_close(outside_file);
	sf_close(water_file);
	sf_close(yt_file);
	sf_close(watch_file);
	sf_close(ambi_file);

	free(src_buffer_ptr);
	free(amb_buffer_ptr);
  	free(smoke_l_buff);
  	free(smoke_r_buff);
  	free(roll_buff);
  	free(fridge_buff);
  	free(heater_buff);
  	free(window_buff);
  	free(outside_buff);
  	free(water_buff);
  	free(yt_buff);
  	free(watch_buff);
  	free(ambi_buff);
  	return 0;

  error:

	std::cout<<"do we ever get here on normal execution?!\n";
  	Pa_Terminate();

	sf_close(smoke_l_file);
	sf_close(smoke_r_file);
	sf_close(roll_file);
	sf_close(fridge_file);
	sf_close(heater_file);
	sf_close(window_file);
	sf_close(outside_file);
	sf_close(water_file);
	sf_close(yt_file);
	sf_close(watch_file);
	sf_close(ambi_file);

	free(src_buffer_ptr);
	free(amb_buffer_ptr);
	free(src_buffer_ptr);
	free(amb_buffer_ptr);
  	free(smoke_l_buff);
  	free(smoke_r_buff);
  	free(roll_buff);
  	free(fridge_buff);
  	free(heater_buff);
  	free(window_buff);
  	free(outside_buff);
  	free(water_buff);
  	free(yt_buff);
  	free(watch_buff);
  	free(ambi_buff);


  	fprintf( stderr, "Error number: %d\n", err );
  	fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
  	return err;
} 


