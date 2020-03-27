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

int ebd_l;
int ebd_r;
int birb1;
int birb2;
int birb3;
int ambi;

float* ebd_l_buff;
float* ebd_r_buff;
float* birb1_buff;
float* birb2_buff;
float* birb3_buff;
float* ambi_buff;

float* ebd_l_p;
float* ebd_r_p;
float* birb1_p;
float* birb2_p;
float* birb3_p;
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

SNDFILE* ebd_l_file;
SNDFILE* ebd_r_file;
SNDFILE* birb1_file;
SNDFILE* birb2_file;
SNDFILE* birb3_file;
SNDFILE* ambi_file;

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
	renderer->AddInterleavedInput(ambi_buff,  quad, num_frames);

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
		ambi_buff=ambi_p;
	}
	else{
		ebd_l_buff+=num_frames;
		ebd_r_buff+=num_frames;
		birb1_buff+=num_frames;
		birb2_buff+=num_frames;
		birb3_buff+=num_frames;
		ambi_buff+=4*num_frames;
	}
        blockcounter++;
  	reson->FillInterleavedOutputBuffer(stereo,num_frames,src_buffer_ptr);
	renderer->GetInterleavedStereoOutput(amb_buffer_ptr,num_frames);

    	long *out = (long*)outputBuffer;
    	unsigned int i;
    	(void) inputBuffer; /* Prevent unused variable warning. */
    	for( i=0; i<framesPerBuffer*2; i++ ){
//        	*out++ = amb_buffer_ptr[i++]/2.0;  /* left */
//       	*out++ = amb_buffer_ptr[i]/2.0;/* right */
//        	*out++ = src_buffer_ptr[i++]/2.0;  /* left */
//       	*out++ = src_buffer_ptr[i]/2.0;/* right */

        	*out++ = (long)((src_buffer_ptr[i]/1.5 + amb_buffer_ptr[i++]/1.5)*1073741823.0);  /* left */
        	*out++ = (long)((src_buffer_ptr[i]/1.5  + amb_buffer_ptr[i]/1.5)*1073741823.0);/* right */
    	}
    	return 0;
}

int main(int argc, char** argv) {

	const char * febd_l= "resources/ebd_l.wav" ;
	const char * febd_r= "resources/ebd_r.wav" ;
	const char * fbirb1= "resources/birb1.wav" ;
	const char * fbirb2= "resources/birb2.wav" ;
	const char * fbirb3= "resources/birb3.wav" ;
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

  	ebd_l_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	ebd_r_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	birb1_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	birb2_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );
  	birb3_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*2 );

  	ambi_buff = (float*) malloc( sizeof(float) * BUFFER_LEN*8 );

	ebd_l_p = ebd_l_buff;
	ebd_r_p = ebd_r_buff;
	birb1_p = birb1_buff;
	birb2_p = birb2_buff;
	birb3_p = birb3_buff;
	ambi_p = ambi_buff;

	SF_INFO* info = new SF_INFO();

	ebd_l_file = sf_open(febd_l,SFM_READ,info) ;
	ebd_r_file = sf_open(febd_r,SFM_READ,info) ;
	birb1_file = sf_open(fbirb1,SFM_READ,info) ;
	birb2_file = sf_open(fbirb2,SFM_READ,info) ;
	birb3_file = sf_open(fbirb3,SFM_READ,info) ;

	SF_INFO* quadinfo = new SF_INFO();

	ambi_file  = sf_open(fambi,   SFM_READ,quadinfo) ;
	std::cout<<quadinfo->format<<"\n";
	std::cout<<quadinfo->channels<<"\n";
	std::cout<<quadinfo->samplerate<<"\n";

	sf_read_float(ebd_l_file, ebd_l_buff, BUFFER_LEN*2) ;
	sf_read_float(ebd_r_file, ebd_r_buff, BUFFER_LEN*2) ;
	sf_read_float(birb1_file, birb1_buff, BUFFER_LEN*2) ;
	sf_read_float(birb2_file, birb2_buff, BUFFER_LEN*2) ;
	sf_read_float(birb3_file, birb3_buff, BUFFER_LEN*2) ;

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

	properties->dimensions[0] = 10.0;
	properties->dimensions[1] = 10.0;
	properties->dimensions[2] = 10.0;

	properties->material_names[0] = kMarble; //left
	properties->material_names[1] = kMarble; //right
	properties->material_names[2] = kMarble; //bottom
	properties->material_names[3] = kMarble; //top
	properties->material_names[4] = kMarble; //front
	properties->material_names[5] = kMarble; //back

	properties->reflection_scalar = 0.5;
	properties->reverb_gain = 0.5;
	properties->reverb_time = 5.0;
	properties->reverb_brightness = 3.0;

	*refp = ComputeReflectionProperties(*properties);
	*revp = ComputeReverbProperties(*properties);

	reson->SetReflectionProperties(*refp);
	//reson->SetReverbProperties(*revp); //causes stutter?

	reson->EnableRoomEffects(true);
  	reson->SetMasterVolume(1.0);
  	reson->SetHeadPosition(0,1,0);

  	ebd_l = reson->CreateSoundObjectSource(quality);
  	ebd_r = reson->CreateSoundObjectSource(quality);
  	birb1 = reson->CreateSoundObjectSource(quality);
  	birb2 = reson->CreateSoundObjectSource(quality);
  	birb3 = reson->CreateSoundObjectSource(quality);


  	reson->SetSourcePosition(ebd_l,1,1,-2.5);
  	reson->SetSourcePosition(ebd_r,-1,1,-2.5);
  	reson->SetSourcePosition(birb1,-2,1,-2);
  	reson->SetSourcePosition(birb2,2,1,3);
  	reson->SetSourcePosition(birb3,0,1,-3);

	reson->SetSourceRoomEffectsGain(ebd_l,1);
	reson->SetSourceRoomEffectsGain(ebd_r,1);
	reson->SetSourceRoomEffectsGain(birb1,1);
	reson->SetSourceRoomEffectsGain(birb2,1);
	reson->SetSourceRoomEffectsGain(birb3,1);

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

	if(sf_seek(ebd_l_file,1,SEEK_CUR)==-1) sf_seek(ebd_l_file,0,SEEK_SET); else  sf_seek(ebd_l_file,-1,SEEK_CUR);
	if(sf_seek(ebd_r_file,1,SEEK_CUR)==-1) sf_seek(ebd_r_file,0,SEEK_SET); else  sf_seek(ebd_r_file,-1,SEEK_CUR);
	if(sf_seek(birb1_file,1,SEEK_CUR)==-1) sf_seek(birb1_file,0,SEEK_SET); else  sf_seek(birb1_file,-1,SEEK_CUR);
	if(sf_seek(birb2_file,1,SEEK_CUR)==-1) sf_seek(birb2_file,0,SEEK_SET); else  sf_seek(birb2_file,-1,SEEK_CUR);
	if(sf_seek(birb3_file,1,SEEK_CUR)==-1) sf_seek(birb3_file,0,SEEK_SET); else  sf_seek(birb3_file,-1,SEEK_CUR);
	if(sf_seek(ambi_file,1,SEEK_CUR)==-1) sf_seek(ambi_file,0,SEEK_SET);   else  sf_seek(ambi_file,-1,SEEK_CUR);

	if(readnext){
		readnext = false;
		sf_read_float(ebd_l_file, ebd_l_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(ebd_r_file, ebd_r_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(birb1_file, birb1_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(birb2_file, birb2_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_read_float(birb3_file, birb3_p+bufferoffset*BUFFER_LEN, BUFFER_LEN) ;
		sf_readf_float(ambi_file,  ambi_p+bufferoffset*BUFFER_LEN*4,  BUFFER_LEN) ;
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
	sf_close(ambi_file);

	free(src_buffer_ptr);
	free(amb_buffer_ptr);
  	free(ebd_l_buff);
  	free(ebd_r_buff);
  	free(birb1_buff);
  	free(birb2_buff);
  	free(birb3_buff);
  	free(ambi_buff);
  	return 0;

  error:

	std::cout<<"do we ever get here on normal execution?!\n";
  	Pa_Terminate();

	sf_close(ebd_l_file);
	sf_close(ebd_r_file);
	sf_close(birb1_file);
	sf_close(birb2_file);
	sf_close(birb3_file);
	sf_close(ambi_file);

	free(src_buffer_ptr);
	free(amb_buffer_ptr);

  	free(ebd_l_buff);
  	free(ebd_r_buff);
  	free(birb1_buff);
  	free(birb2_buff);
  	free(birb3_buff);
  	free(ambi_buff);

  	fprintf( stderr, "Error number: %d\n", err );
  	fprintf( stderr, "Error message: %s\n", Pa_GetErrorText( err ) );
  	return err;
} 


