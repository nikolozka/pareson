#include <iostream> 
#include <sndfile.hh>
#include "portaudio.h"
#include "resonance_audio_api.h"
#include "binaural_surround_renderer.h"

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

int err;
unsigned int i;

std::size_t stereo = 2;
std::size_t mono = 1;

std::size_t num_frames = NUM_FRAMES;

int n_frames = NUM_FRAMES;
int sample_rate_hz = SAMPLE_RATE;

int ebd_l;
int ebd_r;
int birb1;
int birb2;
int birb3;

float* ebd_l_buff;
float* ebd_r_buff;
float* birb1_buff;
float* birb2_buff;
float* birb3_buff;

float* ebd_l_p;
float* ebd_r_p;
float* birb1_p;
float* birb2_p;
float* birb3_p;

float* out_buffer_ptr;

int seekoffset = 1;
int blockcounter = 0;
bool readnext = false;
int bufferoffset = 0;

ResonanceAudioApi* reson;

RenderingMode quality = kBinauralHighQuality;

SNDFILE* ebd_l_file;
SNDFILE* ebd_r_file;
SNDFILE* birb1_file;
SNDFILE* birb2_file;
SNDFILE* birb3_file;


static int patestCallback( const void *inputBuffer, void *outputBuffer,
                           unsigned long framesPerBuffer,
                           const PaStreamCallbackTimeInfo* timeInfo,
                           PaStreamCallbackFlags statusFlags,
                           void *userData ){

   	reson->SetInterleavedBuffer(ebd_l,ebd_l_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(ebd_r,ebd_r_buff, mono, num_frames);
        reson->SetInterleavedBuffer(birb1,birb1_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(birb2,birb2_buff, mono, num_frames);
   	reson->SetInterleavedBuffer(birb3,birb3_buff, mono, num_frames);
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
		ebd_l_buff=ebd_l_p;
		ebd_r_buff=ebd_r_p;
		birb1_buff=birb1_p;
		birb2_buff=birb2_p;
		birb3_buff=birb3_p;
	}
	else{
		ebd_l_buff+=num_frames;
		ebd_r_buff+=num_frames;
		birb1_buff+=num_frames;
		birb2_buff+=num_frames;
		birb3_buff+=num_frames;
	}
        blockcounter++;
  	reson->FillInterleavedOutputBuffer(stereo,num_frames,out_buffer_ptr);

    	float *out = (float*)outputBuffer;
    	unsigned int i;
    	(void) inputBuffer; /* Prevent unused variable warning. */
    	for( i=0; i<framesPerBuffer*2; i++ ){
        	*out++ = out_buffer_ptr[i++];  /* left */
        	*out++ = out_buffer_ptr[i];  /* right */
    	}
    	return 0;
}

int main(int argc, char** argv) {

	const char * febd_l= "resources/ebd_l.wav" ;
	const char * febd_r= "resources/ebd_r.wav" ;
	const char * fbirb1= "resources/birb1.wav" ;
	const char * fbirb2= "resources/birb2.wav" ;
	const char * fbirb3= "resources/birb3.wav" ;
        PaError err;
	PaStream *stream;

  	out_buffer_ptr = (float*) malloc( sizeof(float) * num_frames*2 );

  	ebd_l_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	ebd_r_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	birb1_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	birb2_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	birb3_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );

	ebd_l_p = ebd_l_buff;
	ebd_r_p = ebd_r_buff;
	birb1_p = birb1_buff;
	birb2_p = birb2_buff;
	birb3_p = birb3_buff;

	SF_INFO* info = new SF_INFO();

	ebd_l_file = sf_open(febd_l,SFM_READ,info) ;
	ebd_r_file = sf_open(febd_r,SFM_READ,info) ;
	birb1_file = sf_open(fbirb1,SFM_READ,info) ;
	birb2_file = sf_open(fbirb2,SFM_READ,info) ;
	birb3_file = sf_open(fbirb3,SFM_READ,info) ;

	sf_read_float(ebd_l_file, ebd_l_buff, BUFFER_LEN) ;
	sf_read_float(ebd_r_file, ebd_r_buff, BUFFER_LEN) ;
	sf_read_float(birb1_file, birb1_buff, BUFFER_LEN) ;
	sf_read_float(birb2_file, birb2_buff, BUFFER_LEN) ;
	sf_read_float(birb3_file, birb3_buff, BUFFER_LEN) ;


	err=Pa_Initialize();
	if( err != paNoError )
        {
		std::cout<<"how did we get here?";
		printf( "ERROR: Pa_Initialize returned 0x%x\n", err );
                goto error;
        }

	err = Pa_OpenDefaultStream( &stream,
	                                0,          /* no input channels */
        	                        2,          /* stereo output */
                	                paFloat32,  /* 32 bit floating point output */
                        	        SAMPLE_RATE,/* sample rate */
                                	n_frames,        /* frames per buffer */
	                                patestCallback, /* this is your callback function */
        	                        &data ); /*This is a pointer that will be passed to your callback*/

        if( err != paNoError ) goto error;
  	reson = CreateResonanceAudioApi(stereo,num_frames,sample_rate_hz);

	ReflectionProperties* properties = new ReflectionProperties();

	properties->room_dimensions[0] = 10.0;
	properties->room_dimensions[1] = 10.0;
	properties->room_dimensions[2] = 10.0;

	properties->coefficients[0] = 5;
	properties->coefficients[1] = 5;
	properties->coefficients[2] = 5;
	properties->coefficients[3] = 5;
	properties->coefficients[4] = 5;
	properties->coefficients[5] = 5;

	reson->SetReflectionProperties(*properties);

  	reson->SetMasterVolume(1.0);
  	reson->SetHeadPosition(0,1,0);

  	ebd_l = reson->CreateSoundObjectSource(quality);
  	ebd_r = reson->CreateSoundObjectSource(quality);
  	birb1 = reson->CreateSoundObjectSource(quality);
  	birb2 = reson->CreateSoundObjectSource(quality);
  	birb3 = reson->CreateSoundObjectSource(quality);

  	reson->SetSourcePosition(ebd_l,1,1,-0.5);
  	reson->SetSourcePosition(ebd_r,-1,1,-0.5);
  	reson->SetSourcePosition(birb1,-2,1,-3);
  	reson->SetSourcePosition(birb2,2,1,4);
  	reson->SetSourcePosition(birb3,0,1,-4);

	int sampleCount = 0;
    	int sampleRate = 0;

  	uint64_t rateTimer;
    	uint64_t displayTimer;
    	uint64_t now;

	RTIMUSettings *settings = new RTIMUSettings("RTIMULib");
    	RTIMU *imu = RTIMU::createIMU(settings);
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

	UdpTransmitSocket transmitSocket( IpEndpointName( ADDRESS, PORT ) );

    	char buffer[OUTPUT_BUFFER_SIZE];
    	Quaternion<float> q;


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

        q.Print();

	reson->SetHeadRotation(q.Getx(),q.Gety(),q.Getz(),q.Gets());

	if(readnext){
		readnext = false;
		sf_read_float(ebd_l_file, ebd_l_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(ebd_r_file, ebd_r_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(birb1_file, birb1_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(birb2_file, birb2_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(birb3_file, birb3_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		}
	}
	//Pa_Sleep(NUM_SECONDS*1000);

	err = Pa_StopStream( stream );
	if( err != paNoError ) goto error;
	err = Pa_CloseStream( stream );
	if( err != paNoError ) goto error;

	sf_close(ebd_l_file);
	sf_close(ebd_r_file);
	sf_close(birb1_file);
	sf_close(birb2_file);
	sf_close(birb3_file);

	free(out_buffer_ptr);
  	free(ebd_l_buff);
  	free(ebd_r_buff);
  	free(birb1_buff);
  	free(birb2_buff);
  	free(birb3_buff);
  	return 0;

  error:

	std::cout<<"do we ever get here on normal execution?!\n";
  	Pa_Terminate();

	sf_close(ebd_l_file);
	sf_close(ebd_r_file);
	sf_close(birb1_file);
	sf_close(birb2_file);
	sf_close(birb3_file);

	free(out_buffer_ptr);
  	free(ebd_l_buff);
  	free(ebd_r_buff);
  	free(birb1_buff);
  	free(birb2_buff);
  	free(birb3_buff);
  	fprintf( stderr, "Error number: %d\n", err );
  	fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
  	return err;
} 


