#include <RtAudio.h>

class AudioSink
{
protected:
    bool isOpen;
    RtAudio dev;
    std::vector< unsigned int > ids;
    std::vector<std::string> device_names;
    std::vector<int> device_ids;
    int dev_id;
    RtAudio::DeviceInfo info;
    RtAudio::StreamParameters parameters;
    unsigned int sampleRate;
    unsigned int bufferFrames;
    int (*callback_fn)( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData );
public:
    AudioSink(unsigned int SampleRate, unsigned int nBuffers, int DefaultDvcId = 0);
    void SetCallback(int (*Callback)(void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData));
    void Start();
    void Stop();
    void SetSampleRate(unsigned int newSampleRate);
    void SetNoFrames(unsigned int newNumBuffers);
    void ChangeDevice(int NewDevId);
    void SetDeviceId(int DeviceId);
    void Open();
    void Close();
    int GetCurrentDeviceId();
    std::string GetCurrentDeviceName();
    const char* GetCurrentDeviceNameC();
    std::string GetDeviceName(int DeviceId);
    const char* GetDeviceNameC(int DeviceId);
    size_t GetNumDevices();
    ~AudioSink();
};

class AudioSource
{
protected:
    bool isOpen;
    RtAudio dev;
    std::vector< unsigned int > ids;
    std::vector<std::string> device_names;
    std::vector<int> device_ids;
    int dev_id;
    RtAudio::DeviceInfo info;
    RtAudio::StreamParameters parameters;
    unsigned int sampleRate;
    unsigned int bufferFrames;
    int (*callback_fn)( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData );
public:
    AudioSource(unsigned int SampleRate, unsigned int nBuffers, int DefaultDvcId = 0);
    void SetCallback(int (*Callback)(void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData));
    void Start();
    void Stop();
    void SetSampleRate(unsigned int newSampleRate);
    void SetNoFrames(unsigned int newNumBuffers);
    void ChangeDevice(int NewDevId);
    void SetDeviceId(int DeviceId);
    void Open();
    void Close();
    int GetCurrentDeviceId();
    std::string GetCurrentDeviceName();
    const char* GetCurrentDeviceNameC();
    std::string GetDeviceName(int DeviceId);
    const char* GetDeviceNameC(int DeviceId);
    size_t GetNumDevices();
    ~AudioSource();
};

