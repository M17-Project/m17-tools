#include "AudioWrapper.h"

void AudioSink::ChangeDevice(int NewDevId){
    Close();
    SetDeviceId(NewDevId);
    Open();
}

void AudioSink::SetDeviceId(int DeviceId){
    dev_id = DeviceId;
    parameters.deviceId = device_ids[dev_id];
 }

size_t AudioSink::GetNumDevices(){
    return device_names.size();
}

std::string AudioSink::GetDeviceName(int DeviceId){
    return device_names[DeviceId];
}

const char* AudioSink::GetDeviceNameC(int DeviceId){
    return device_names[DeviceId].c_str();
}

void AudioSink::Start(){
    if(!isOpen){
        Open();
    }
    std::cerr << "Starting AudioSink\n";
    dev.startStream();
}

void AudioSink::Stop(){
    std::cerr << "Stoping AudioSink\n";
    dev.stopStream();
}

void AudioSink::SetCallback(int (*Callback)(void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData)){
    callback_fn = Callback;
}

void AudioSink::Open(){
    isOpen=true;
    std::cerr << "Opening AudioSink\n";
    dev.openStream( &parameters,  NULL, RTAUDIO_SINT16, sampleRate, &bufferFrames, callback_fn);
}

void AudioSink::Close(){
    if(isOpen){
        std::cerr << "Closing AudioSink\n";
        dev.closeStream();
    }
}

int AudioSink::GetCurrentDeviceId(){
    return dev_id;
}

std::string AudioSink::GetCurrentDeviceName(){return GetDeviceName(dev_id);}
const char* AudioSink::GetCurrentDeviceNameC(){return GetDeviceNameC(dev_id);}

void AudioSink::SetSampleRate(unsigned int newSampleRate){
    Close();
    sampleRate = newSampleRate;
    Open();
}

void AudioSink::SetNoFrames(unsigned int newNumBuffers){
    Close();
    bufferFrames = newNumBuffers;
    Open();
}

AudioSink::AudioSink(unsigned int SampleRate, unsigned int nBuffers, int DefaultDvcId)
{
    ids = dev.getDeviceIds();
    if ( ids.size() == 0 ) {
      std::cout << "No devices found." << std::endl;
    }

    for ( unsigned int n=0; n<ids.size(); n++ ) {
        info = dev.getDeviceInfo( ids[n] );
        if(info.outputChannels>0){
            device_names.push_back(std::string(info.name));
            device_ids.push_back(ids[n]);
            if(info.isDefaultInput)
                dev_id = device_ids.size()-1;
        }
    }

    sampleRate = SampleRate;
    bufferFrames = nBuffers;
    dev_id = DefaultDvcId;

    parameters.deviceId = device_ids[dev_id];
    parameters.nChannels = 2;
    parameters.firstChannel = 0;
    isOpen=false;
}
AudioSink::~AudioSink()
{
    Close();
}

void AudioSource::ChangeDevice(int NewDevId){
    Close();
    SetDeviceId(NewDevId);
    Open();
}

void AudioSource::SetDeviceId(int DeviceId){
    dev_id = DeviceId;
    parameters.deviceId = device_ids[dev_id];
 }

size_t AudioSource::GetNumDevices(){
    return device_names.size();
}

std::string AudioSource::GetDeviceName(int DeviceId){
    return device_names[DeviceId];
}

const char* AudioSource::GetDeviceNameC(int DeviceId){
    return device_names[DeviceId].c_str();
}

void AudioSource::Start(){
    if(!isOpen){
        Open();
    }
    std::cerr << "Starting AudioSource\n";
    dev.startStream();
}

void AudioSource::Stop(){
    std::cerr << "Stoping AudioSource\n";
    dev.stopStream();
}

void AudioSource::SetCallback(int (*Callback)(void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData)){
    callback_fn = Callback;
}

void AudioSource::Open(){
    isOpen=true;
    std::cerr << "Opening AudioSource\n";
    dev.openStream(NULL, &parameters, RTAUDIO_SINT16, sampleRate, &bufferFrames, callback_fn);
}

void AudioSource::Close(){
    if(isOpen){
        std::cerr << "Closing AudioSource\n";
        dev.closeStream();
    }
}

int AudioSource::GetCurrentDeviceId(){
    return dev_id;
}

std::string AudioSource::GetCurrentDeviceName(){return GetDeviceName(dev_id);}
const char* AudioSource::GetCurrentDeviceNameC(){return GetDeviceNameC(dev_id);}


void AudioSource::SetSampleRate(unsigned int newSampleRate){
    Close();
    sampleRate = newSampleRate;
    Open();
}

void AudioSource::SetNoFrames(unsigned int newNumBuffers){
    bufferFrames = newNumBuffers;
}

AudioSource::AudioSource(unsigned int SampleRate, unsigned int nBuffers, int DefaultDvcId)
{
    ids = dev.getDeviceIds();
    if ( ids.size() == 0 ) {
      std::cout << "No devices found." << std::endl;
    }

    for ( unsigned int n=0; n<ids.size(); n++ ) {
        info = dev.getDeviceInfo( ids[n] );
        if(info.inputChannels>0){
            device_names.push_back(std::string(info.name));
            device_ids.push_back(ids[n]);
            if(info.isDefaultInput)
                dev_id = device_ids.size()-1;
        }
    }

    sampleRate = SampleRate;
    bufferFrames = nBuffers;
    dev_id = DefaultDvcId;

    parameters.deviceId = device_ids[dev_id];
    parameters.nChannels = 1;
    parameters.firstChannel = 0;
    isOpen=false;
}
AudioSource::~AudioSource()
{
    Close();
}

