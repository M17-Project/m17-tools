// Copyright 2020 Mobilinkd LLC.

#include "M17Demodulator.h"
#include "M17BitDemodulator.h"
#include "CRC16.h"
#include "ax25_frame.h"
#include "FirFilter.h"
#include "queue.h"

#include <codec2/codec2.h>
#include <boost/crc.hpp>
#include <boost/program_options.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/hex.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>


#define CTR 1
#include "aes.hpp"
#include <chrono>
#include <random>

#ifdef WIN32
#include <io.h>
#include <fcntl.h>
#endif

#include <signal.h>

#include "imgui.h"
#include "imgui_internal.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <RtAudio.h>
#include "AudioWrapper.h"
#include "../external/serialib.h"

#if defined (_WIN32) || defined(_WIN64)
    //for serial ports above "COM9", we must use this extended syntax of "\\.\COMx".
    //also works for COM0 to COM9.
    //https://docs.microsoft.com/en-us/windows/win32/api/fileapi/nf-fileapi-createfilea?redirectedfrom=MSDN#communications-resources
    std::vector<std::string> get_available_ports(serialib &serial){
        std::vector<std::string> port_names;
        for (int i = 0; i < 255; i++)
        {
           std::string name = "\\\\.\\COM" + std::to_string(i);
           char errorOpening = serial.openDevice(name.c_str(), 115200);
           serial.RTS(false);
           serial.DTR(false);
           // If connection fails, return the error code otherwise, display a success message
           if (errorOpening==1){
            port_names.push_back(name);
            serial.closeDevice();
           }
        }
        return port_names;
    }
#endif
#if defined (__linux__) || defined(__APPLE__)
    #include <filesystem>
    #include <fstream>
    std::vector<std::string> get_available_ports(serialib &serial) {
    std::vector<std::string> ports;
    std::filesystem::path kdr_path = "/proc/tty/drivers";
    if (std::filesystem::exists(kdr_path))
    {
        std::ifstream ifile(kdr_path.generic_string());
        std::string line;
        std::vector<std::string> prefixes;
        while (std::getline(ifile, line))
        {
            std::vector<std::string> items;
            auto it = line.find_first_not_of(' ');
            while (it != std::string::npos)
            {

                auto it2 = line.substr(it).find_first_of(' ');
                if (it2 == std::string::npos)
                {
                    items.push_back(line.substr(it));
                    break;
                }
                it2 += it;
                items.push_back(line.substr(it, it2 - it));
                it = it2 + line.substr(it2).find_first_not_of(' ');
            }
            if (items.size() >= 5)
            {
                if (items[4] == "serial" && items[0].find("serial") != std::string::npos)
                {
                    prefixes.emplace_back(items[1]);
                }
            }
        }
        ifile.close();
        for (auto& p: std::filesystem::directory_iterator("/dev"))
        {
            for (const auto& pf : prefixes)
            {
                auto dev_path = p.path().generic_string();
                if (dev_path.size() >= pf.size() && std::equal(dev_path.begin(), dev_path.begin() + pf.size(), pf.begin()))
                {
                    ports.emplace_back(dev_path);
                }
            }
        }
    }else{
        std::cerr << "/proc/tty/drivers: No such file or directory\n";
    }
    return ports;
    }
#endif


std::atomic<bool> running{false};


using queue_t = mobilinkd::queue<int16_t, 1920>;

std::shared_ptr<queue_t>squeue;
std::shared_ptr<queue_t>basebandQueue;

bool doBasebandCout = false;

bool display_lsf = false;
bool invert_input = false;
bool quiet = false;
bool debug = false;
bool noise_blanker = false;

//bool enc_key = false;
uint8_t Key[32];	
uint8_t Iv[16];
struct AES_ctx ctx;
bool frame_has_enc = false;

enum class InputType {SYM, BIN, RRC};
InputType inputType = InputType::RRC;

enum class FrameType {AUDIO, DATA, MIXED, BERT};

using lsf_t = std::array<uint8_t, 30>;

enum class OutputType {SYM, BIN, RRC};

OutputType outputType = OutputType::RRC;

struct CODEC2 *codec2;

// Generated using scikit-commpy
/*const auto rrc_taps = std::array<double, 150>{
    0.0029364388513841593, 0.0031468394550958484, 0.002699564567597445, 0.001661182944400927, 0.00023319405581230247, -0.0012851320781224025, -0.0025577136087664687, -0.0032843366522956313, -0.0032697038088887226, -0.0024733964729590865, -0.0010285696910973807, 0.0007766690889758685, 0.002553421969211845, 0.0038920145144327816, 0.004451886520053017, 0.00404219185231544, 0.002674727068399207, 0.0005756567993179152, -0.0018493784971116507, -0.004092346891623224, -0.005648131453822014, -0.006126925416243605, -0.005349511529163396, -0.003403189203405097, -0.0006430502751187517, 0.002365929161655135, 0.004957956568090113, 0.006506845894531803, 0.006569574194782443, 0.0050017573119839134, 0.002017321931508163, -0.0018256054303579805, -0.00571615173291049, -0.008746639552588416, -0.010105075751866371, -0.009265784007800534, -0.006136551625729697, -0.001125978562075172, 0.004891777252042491, 0.01071805138282269, 0.01505751553351295, 0.01679337935001369, 0.015256245142156299, 0.01042830577908502, 0.003031522725559901, -0.0055333532968188165, -0.013403099825723372, -0.018598682349642525, -0.01944761739590459, -0.015005271935951746, -0.0053887880354343935, 0.008056525910253532, 0.022816244158307273, 0.035513467692208076, 0.04244131815783876, 0.04025481153629372, 0.02671818654865632, 0.0013810216516704976, -0.03394615682795165, -0.07502635967975885, -0.11540977897637611, -0.14703962203941534, -0.16119995609538576, -0.14969512896336504, -0.10610329539459686, -0.026921412469634916, 0.08757875030779196, 0.23293327870303457, 0.4006012210123992, 0.5786324696325503, 0.7528286479934068, 0.908262741447522, 1.0309661131633199, 1.1095611856548013, 1.1366197723675815, 1.1095611856548013, 1.0309661131633199, 0.908262741447522, 0.7528286479934068, 0.5786324696325503, 0.4006012210123992, 0.23293327870303457, 0.08757875030779196, -0.026921412469634916, -0.10610329539459686, -0.14969512896336504, -0.16119995609538576, -0.14703962203941534, -0.11540977897637611, -0.07502635967975885, -0.03394615682795165, 0.0013810216516704976, 0.02671818654865632, 0.04025481153629372, 0.04244131815783876, 0.035513467692208076, 0.022816244158307273, 0.008056525910253532, -0.0053887880354343935, -0.015005271935951746, -0.01944761739590459, -0.018598682349642525, -0.013403099825723372, -0.0055333532968188165, 0.003031522725559901, 0.01042830577908502, 0.015256245142156299, 0.01679337935001369, 0.01505751553351295, 0.01071805138282269, 0.004891777252042491, -0.001125978562075172, -0.006136551625729697, -0.009265784007800534, -0.010105075751866371, -0.008746639552588416, -0.00571615173291049, -0.0018256054303579805, 0.002017321931508163, 0.0050017573119839134, 0.006569574194782443, 0.006506845894531803, 0.004957956568090113, 0.002365929161655135, -0.0006430502751187517, -0.003403189203405097, -0.005349511529163396, -0.006126925416243605, -0.005648131453822014, -0.004092346891623224, -0.0018493784971116507, 0.0005756567993179152, 0.002674727068399207, 0.00404219185231544, 0.004451886520053017, 0.0038920145144327816, 0.002553421969211845, 0.0007766690889758685, -0.0010285696910973807, -0.0024733964729590865, -0.0032697038088887226, -0.0032843366522956313, -0.0025577136087664687, -0.0012851320781224025, 0.00023319405581230247, 0.001661182944400927, 0.002699564567597445, 0.0031468394550958484, 0.0029364388513841593, 0.0
};*/

const auto evm_b = std::array<double, 3>{0.02008337, 0.04016673, 0.02008337};
const auto evm_a = std::array<double, 3>{1.0, -1.56101808, 0.64135154};

std::string ptt_on = "";
std::string ptt_off = "";
bool has_ptt = false;
bool invert = false;
bool enc_key = false;
int8_t can = 10;

std::vector<uint8_t> current_packet;
size_t packet_frame_counter = 0;
mobilinkd::CRC16<0x1021, 0xFFFF> packet_crc;
mobilinkd::CRC16<0x5935, 0xFFFF> stream_crc;

mobilinkd::PRBS9 prbs;

template <typename T, size_t N>
std::vector<uint8_t> to_packet(std::array<T, N> in)
{
    std::vector<uint8_t> result;
    result.reserve(N/8);

    uint8_t out = 0;
    size_t b = 0;

    for (auto c : in)
    {
        out = (out << 1) | c;
        if (++b == 8)
        {
            result.push_back(out);
            out = 0;
            b = 0;
        }
    }

    return result;
}

template <typename T, size_t N>
void append_packet(std::vector<uint8_t>& result, std::array<T, N> in)
{
    uint8_t out = 0;
    size_t b = 0;

    for (auto c : in)
    {
        out = (out << 1) | c;
        if (++b == 8)
        {
            result.push_back(out);
            out = 0;
            b = 0;
        }
    }
}

void dump_type(uint16_t type)
{
    std::cerr << ", ";
    if (type & 1) {
        std::cerr << "STR:";
        switch ((type & 6) >> 1)
        {
            case 0:
                std::cerr << "UNK";
                break;
            case 1:
                std::cerr << "D/D";
                break;
            case 2:
                std::cerr << "V/V";
                break;
            case 3:
                std::cerr << "V/D";
                break;
        }
		switch(3 & (type >> 3)){
			case 0:
				std::cerr << ", ENC:NONE,";
				break;
			case 1:
				std::cerr << ", ENC:SCR,";
				break;
			case 2:
				std::cerr << ", ENC:AES,";
				break;
			default:
				std::cerr << ", ENC:UNK,";
				break;
		}
    }
    else
    {
        std::cerr << "PKT:";
        switch ((type & 6) >> 1)
        {
            case 0:
                std::cerr << "UNK";
                break;
            case 1:
                std::cerr << "RAW";
                break;
            case 2:
                std::cerr << "ENC";
                break;
            case 3:
                std::cerr << "UNK";
                break;
        }
    }

    std::cerr << " CAN:" << std::dec << std::setw(2) << std::setfill('0') << int((type & 0x780) >> 7);
}

uint8_t last_FN[2];

std::mutex coutMutex;

struct Diagnostics{
    std::string source = "";
    std::string dest = "";
    std::string dcd = "OFF";
    float dev = 0.0f;
    std::string lock = "UNLOCKED";
    float evm = 0.0f;
};

void ResetDiag(Diagnostics* p_diag){
    p_diag->source = "";
    p_diag->dest = "";
    p_diag->dcd = "OFF";
    p_diag->dev = 0.0f;
    p_diag->lock = "UNLOCKED";
    p_diag->evm = 0.0f;
}

std::unique_ptr<Diagnostics> diag;

template <typename T, size_t N>
bool dump_lsf(std::array<T, N> const& lsf)
{
    using namespace mobilinkd;
    
    LinkSetupFrame::encoded_call_t encoded_call;
	
	frame_has_enc = false;
	
	uint16_t type = (lsf[12] << 8) | lsf[13];
	
	if((type & 1) && ((3 & (type >> 3)) != 0)) frame_has_enc = true;

    if (display_lsf)
    {
        coutMutex.lock();
        std::copy(lsf.begin() + 6, lsf.begin() + 12, encoded_call.begin());
        auto src = LinkSetupFrame::decode_callsign(encoded_call);
        std::cerr << "\nSRC: ";
        for (auto x : src) if (x) std::cerr << x;

        std::copy(lsf.begin(), lsf.begin() + 6, encoded_call.begin());
        auto dest = LinkSetupFrame::decode_callsign(encoded_call);
        std::cerr << ", DEST: ";
        for (auto x : dest) if (x) std::cerr << x;

        dump_type(type);

        std::cerr << ", NONCE: ";
        for (size_t i = 14; i != 28; ++i) std::cerr << std::hex << std::setw(2) << std::setfill('0') << int(lsf[i]);

        uint16_t crc = (lsf[28] << 8) | lsf[29];
        std::cerr << ", CRC: " << std::hex << std::setw(4) << std::setfill('0') << crc;
        std::cerr << std::dec << std::endl;
        diag->source = std::string(src.begin(),src.end());
        diag->dest = std::string(dest.begin(),dest.end());
        coutMutex.unlock();
    }

    current_packet.clear();
    packet_frame_counter = 0;
	
  //if key and AES ENCRYPTION BITS are set
	if(enc_key && frame_has_enc){
		//copy meta field [NONCE + CTR] and FN
		std::copy(std::begin(lsf)+14,std::begin(lsf)+28,std::begin(Iv));
		Iv[14] = last_FN[0];
		Iv[15] = last_FN[1];
		
		//init IV
		AES_init_ctx_iv(&ctx, Key, Iv);
	}

    if (!(lsf[13] & 1)) // LSF type bit 0
    {
        uint8_t packet_type = (lsf[13] & 6) >> 1;

        switch (packet_type)
        {
        case 1: // RAW -- ignore LSF.
             break;
        case 2: // ENCAPSULATED
            append_packet(current_packet, lsf);
            break;
        default:
            std::cerr << "LSF for reserved packet type" << std::endl;
            append_packet(current_packet, lsf);
        }
    }

    return true;
}

float *spk_gain;
float *rx_gain;

float *tx_mic_gain;
float *tx_out_gain;

int16_t ApplyGain(int16_t input, float* gain){
    float ip = (float) input / (float) INT16_MAX;
    ip *= (*gain);
    int16_t val = std::min(std::max((int16_t)INT16_MIN,(int16_t)(ip*(INT16_MAX-1))),(int16_t)INT16_MAX);
    return val;
}

float ApplyGainF(float input, float* gain){
    return input * (*gain);
}

void AudioOutput(std::array<int16_t, 160> buf){
    if(!doBasebandCout){
        for (int i=0; i<buf.size(); i++){
            int16_t b = ApplyGain(buf[i], spk_gain);
            if(squeue->is_open()){
                if(!squeue->put(b, std::chrono::milliseconds(3000))){
                    std::cerr<<"AudioOutput(): ERROR OUTPUT QUEUE\n";
                }
            }
        }
    }
    else{
        std::cout.write((const char*)buf.data(), 320);
    }
}

bool demodulate_audio(mobilinkd::M17FrameDecoder::audio_buffer_t const& audio, int viterbi_cost)
{
    bool result = true;

    std::array<int16_t, 160> buf;
    // First two bytes are the frame counter + EOS indicator.
    if (viterbi_cost < 70 && (audio[0] & 0x80))
    {
        if (display_lsf) std::cerr << "\nEOS" << std::endl;
        result = false;
    }

    if (noise_blanker && viterbi_cost > 80)
    {
        buf.fill(0);
        AudioOutput(buf);
        AudioOutput(buf);
    }
    else
    {
		if(enc_key && frame_has_enc){ //if -K provided and ENC bits set
			
			mobilinkd::M17FrameDecoder::audio_buffer_t enc_audio;
			
			// get codec2 payload
			std::copy(audio.begin()+2,audio.begin()+18, enc_audio.begin());
			
			uint8_t* p_payload = enc_audio.data();
			
			//update IV
			Iv[14] = audio[0];
			Iv[15] = audio[1];
			AES_init_ctx_iv(&ctx, Key, Iv);
		
			AES_CTR_xcrypt_buffer(&ctx, p_payload, 16);
			
			codec2_decode(codec2, buf.data(), p_payload);
			AudioOutput(buf);
			codec2_decode(codec2, buf.data(), p_payload + 8);
			AudioOutput(buf);
		}
		else if(frame_has_enc && !enc_key){  //if -K isn't provided but ENC bits set - MUTE
			buf.fill(0);
			AudioOutput(buf);
			AudioOutput(buf);
		}			
		else{
			codec2_decode(codec2, buf.data(), audio.data() + 2);
			AudioOutput(buf);
			codec2_decode(codec2, buf.data(), audio.data() + 10);
			AudioOutput(buf);
		}
    }

    return result;
}

bool decode_packet(mobilinkd::M17FrameDecoder::packet_buffer_t const& packet_segment)
{
    if (packet_segment[25] & 0x80) // last frame of packet.
    {
        size_t packet_size = (packet_segment[25] & 0x7F) >> 2;
        packet_size = std::min(packet_size, size_t(25));
        for (size_t i = 0; i != packet_size; ++i)
        {
            current_packet.push_back(packet_segment[i]);
        }
        
        boost::crc_optimal<16, 0x1021, 0xFFFF, 0xFFFF, true, true> crc;
        crc.process_bytes(&current_packet.front(), current_packet.size());
        uint16_t checksum = crc.checksum();

        if (checksum == 0x0f47)
        {
            std::string ax25;
            ax25.reserve(current_packet.size());
            for (auto c : current_packet) ax25.push_back(char(c));
            mobilinkd::ax25_frame frame(ax25);
            std::cerr << '\n';
            mobilinkd::write(std::cerr, frame);
            return true;
        }

        std::cerr << "\nPacket checksum error: " << std::hex << checksum << std::dec << std::endl;

        return false;
    }

    size_t frame_number = (packet_segment[25] & 0x7F) >> 2;
    if (frame_number != packet_frame_counter)
    {
        std::cerr << "\nPacket frame sequence error. Got " << frame_number << ", expected " << packet_frame_counter << "\n";
        return false;
    }

    packet_frame_counter += 1;

    for (size_t i = 0; i != 25; ++i)
    {
        current_packet.push_back(packet_segment[i]);
    }

    return true;
}


bool decode_full_packet(mobilinkd::M17FrameDecoder::packet_buffer_t const& packet_segment)
{
    if (packet_segment[25] & 0x80) // last packet;
    {
        size_t packet_size = (packet_segment[25] & 0x7F) >> 2;
        packet_size = std::min(packet_size, size_t(25));
        for (size_t i = 0; i != packet_size; ++i)
        {
            current_packet.push_back(packet_segment[i]);
        }

        std::cout.write((const char*)&current_packet.front(), current_packet.size());

        return true;
    }

    size_t frame_number = (packet_segment[25] & 0x7F) >> 2;
    if (frame_number != packet_frame_counter++)
    {
        std::cerr << "Packet frame sequence error" << std::endl;
        return false;
    }

    for (size_t i = 0; i != 25; ++i)
    {
        current_packet.push_back(packet_segment[i]);
    }

    return true;
}

bool decode_bert(mobilinkd::M17FrameDecoder::bert_buffer_t const& bert)
{
    for (int j = 0; j != 24; ++j) {
        auto b = bert[j];
        for (int i = 0; i != 8; ++i) {
            prbs.validate(b & 0x80);
            b <<= 1;
        }
    }

    auto b = bert[24];
    for (int i = 0; i != 5; ++i)
    {
        prbs.validate(b & 0x80);
        b <<= 1;
    }

    return true;
}



bool handle_frame(mobilinkd::M17FrameDecoder::output_buffer_t const& frame, int viterbi_cost)
{
    using FrameType = mobilinkd::M17FrameDecoder::FrameType;

    bool result = true;

    switch (frame.type)
    {
        case FrameType::LSF:
            result = dump_lsf(frame.lsf);
            break;
        case FrameType::LICH:
            std::cerr << "LICH" << std::endl;
            break;
        case FrameType::STREAM:
			std::copy(frame.stream.begin(), frame.stream.begin()+2, last_FN);
            result = demodulate_audio(frame.stream, viterbi_cost);
            break;
        case FrameType::BASIC_PACKET:
            result = decode_packet(frame.packet);
            break;
        case FrameType::FULL_PACKET:
            result = decode_packet(frame.packet);
            break;
        case FrameType::BERT:
            result = decode_bert(frame.bert);
            break;
    }

    return result;
}

template <typename FloatType>
void diagnostic_callback(bool dcd, FloatType evm, FloatType deviation, FloatType offset, bool locked,
    FloatType clock, int sample_index, int sync_index, int clock_index, int viterbi_cost)
{
    if (debug) {
        coutMutex.lock();
        std::cerr << "\rdcd: " << std::setw(1) << int(dcd)
            << ", evm: " << std::setfill(' ') << std::setprecision(4) << std::setw(8) << evm * 100 <<"%"
            << ", deviation: " << std::setprecision(4) << std::setw(8) << deviation
            << ", freq offset: " << std::setprecision(4) << std::setw(8) << offset
            << ", locked: " << std::boolalpha << std::setw(6) << locked << std::dec
            << ", clock: " << std::setprecision(7) << std::setw(8) << clock
            << ", sample: " << std::setw(1) << sample_index << ", "  << sync_index << ", " << clock_index
            << ", cost: " << viterbi_cost;
        diag->dcd = dcd ? "ON" : "OFF";
        diag->lock = locked ? "LOCK" : "UNLOCKED";
        diag->dev = deviation;
        diag->evm = evm * 100;
        coutMutex.unlock();
    }
        
    if (!dcd && prbs.sync()) { // Seems like there should be a better way to do this.
        prbs.reset();
    }

    if (prbs.sync() && !quiet) {
        if (!debug) {
            std::cerr << '\r';
        } else {
            std::cerr << ", ";
        }
    
        auto ber = double(prbs.errors()) / double(prbs.bits());
        std::cerr << "BER: " << std::fixed << std::setprecision(6) << ber << " (" << prbs.bits() << ")";
    }
    std::cerr << std::flush;
}
const char VERSION[] = "2.2";

struct Config
{
    std::string source_address;
    std::string destination_address;
    std::string audio_device;
    std::string event_device;
	std::string tx_on_cmd;
	std::string tx_off_cmd;
	bool action_ptt = false;
    uint16_t key;

    bool verbose = false;
    bool debug = true;
    bool quiet = false;
    bool invert = false;
    bool lsf = true;
    bool noise_blanker = false;
    bool bin = false;
    bool sym = false;
    bool rrc = true; // default is rrc

    bool bert = false; // Bit error rate testing.
    int can = 0; // default is 0
	
	bool encrypt = false; //Default is no Encryption
	std::string CKEY; //AES Key
    unsigned int dev_IDs[4] = {0u, 0u, 0u, 0u};
    float dev_Gains[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    bool rig_enabled = false;
    int ser_id = 0;
    int baud_id = 0;
    int ptt_id = 1;

    static std::optional<Config> parse(int argc, char* argv[])
    {
        namespace po = boost::program_options;

        Config result;

        // Declare the supported options.
        po::options_description desc(
            "Program options");
        desc.add_options()
            ("help,h", "Print this help message and exit.")
            ("version,V", "Print the application verion and exit.")
            ("invert,i", po::bool_switch(&result.invert), "invert the received baseband")
            ("noise-blanker,b", po::bool_switch(&result.noise_blanker), "noise blanker -- silence likely corrupt audio")
            ("lsf,l", po::bool_switch(&result.lsf), "display the decoded LSF")
            ("bin,x", po::bool_switch(&result.bin), "input packed dibits (default is rrc).")
            ("rrc,r", po::bool_switch(&result.rrc), "input rrc filtered and scaled symbols (default).")
            ("sym,s", po::bool_switch(&result.sym), "input symbols (default is rrc).")
			("encrypt,K",po::value<std::string>(&result.CKEY), "hexadecimal string for AES 128, 192 or 256 Key (default is no encryption).")
            ("verbose,v", po::bool_switch(&result.verbose), "verbose output")
            ("debug,d", po::bool_switch(&result.debug), "debug-level output")
            ("quiet,q", po::bool_switch(&result.quiet), "silence all output -- no BERT output")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "Read M17 baseband from STDIN and write audio to STDOUT\n"
                << desc << std::endl;

            return std::nullopt;
        }

        if (vm.count("version"))
        {
            std::cout << argv[0] << ": " << VERSION << std::endl;
            return std::nullopt;
        }

        try {
            po::notify(vm);
        } catch (std::exception& ex)
        {
            std::cerr << ex.what() << std::endl;
            std::cout << desc << std::endl;
            return std::nullopt;
        }

        if (result.debug + result.verbose + result.quiet > 1)
        {
            std::cerr << "Only one of quiet, verbose or debug may be chosen." << std::endl;
            return std::nullopt;
        }

        if (result.sym + result.bin + result.rrc > 1)
        {
            std::cerr << "Only one of sym, bin or rrc may be chosen." << std::endl;
            return std::nullopt;
        }
		
		switch(result.CKEY.length()/2){
			default:
				result.encrypt = false;
				std::cerr << "No encryption." << std::endl;
				break;
			case 16:
				result.encrypt = true;
				#define AES128 1
				std::cerr << "AES 128 KEY." << std::endl;
				break;
			case 24:
				result.encrypt = true;
				#define AES192 1
				std::cerr << "AES 192 KEY." << std::endl;
				break;
				
			case 32:
				result.encrypt = true;
				#define AES256 1
				std::cerr << "AES 256 KEY." << std::endl;
				break;
			}

        return result;
    }
};

mobilinkd::M17Demodulator<float> *pdemod = new mobilinkd::M17Demodulator<float>(handle_frame);

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<uint32_t> uint_dist;



void signal_handler(int)
{
    running = false;
    std::cerr << "quitting" << std::endl;
}


template <typename T, size_t N>
std::array<int8_t, N / 2> bits_to_symbols(const std::array<T, N>& bits)
{
    std::array<int8_t, N / 2> result;
    size_t index = 0;
    for (size_t i = 0; i != N; i += 2)
    {
        result[index++] = bits_to_symbol((bits[i] << 1) | bits[i + 1]);
    }
    return result;
}


using bitstream_t = std::array<int8_t, 368>;

// bin
void output_bitstream(std::array<uint8_t, 2> sync_word, const bitstream_t& frame)
{
    for (auto c : sync_word) std::cout << c;
    for (size_t i = 0; i != frame.size(); i += 8)
    {
        uint8_t c = 0;
        for (size_t j = 0; j != 8; ++j)
        {
            c <<= 1;
            c |= frame[i + j];
        }
        std::cout << c;
    }
}

// sym
void output_symbols(std::array<uint8_t, 2> sync_word, const bitstream_t& frame)
{
    auto symbols = bits_to_symbols(frame);
    auto sw = bytes_to_symbols(sync_word);

    std::array<int8_t, 192> temp;
    auto fit = std::copy(sw.begin(), sw.end(), temp.begin());
    std::copy(symbols.begin(), symbols.end(), fit);
    for (auto b : temp) std::cout << b;
}

using lich_segment_t = std::array<uint8_t, 96>;
using lich_t = std::array<lich_segment_t, 6>;
using audio_frame_t = std::array<int16_t, 320>;
using codec_frame_t = std::array<uint8_t, 16>;
using data_frame_t = std::array<int8_t, 272>;

// rrc
void output_baseband(std::array<uint8_t, 2> sync_word, const bitstream_t& frame)
{
    auto symbols = bits_to_symbols(frame);
    auto sw = bytes_to_symbols(sync_word);

    std::array<int8_t, 192> temp;
    auto fit = std::copy(sw.begin(), sw.end(), temp.begin());
    std::copy(symbols.begin(), symbols.end(), fit);
    auto baseband = symbols_to_baseband(temp);
    for (auto b : baseband){
        if(!doBasebandCout && squeue->is_open()){
            b = ApplyGain(b, tx_out_gain);
            if(!squeue->put(b, std::chrono::milliseconds(3000))){
                std::cerr<<"output_baseband(): ERROR OUTPUT QUEUE\n";
            }
        }
        else{
            std::cerr<<"output_baseband(): ERROR CLOSED QUEUE\n";
            std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
        }
    } 
}



void output_frame(std::array<uint8_t, 2> sync_word, const bitstream_t& frame)
{
    switch(outputType) 
    {
        case OutputType::SYM:
            output_symbols(sync_word, frame);
            break;
        case OutputType::BIN:
            output_bitstream(sync_word, frame);
            break;
        default: // OutputType::RRC
            output_baseband(sync_word, frame);
            break;
    }
}

void send_preamble()
{
    // Preamble is simple... bytes -> symbols -> baseband.
    std::cerr << "Sending preamble." << std::endl;
    std::array<uint8_t, 48> preamble_bytes;
    preamble_bytes.fill(0x77);
    switch(outputType) {
        case OutputType::SYM: 
            {
                auto preamble_symbols = bytes_to_symbols(preamble_bytes);
                for (auto b : preamble_symbols) std::cout << b;
            }
            break;
        case OutputType::BIN:
            for (auto c : preamble_bytes) std::cout << c;
            break;
        default: 
            { // OutputType::RRC
                auto preamble_symbols = bytes_to_symbols(preamble_bytes);
                auto preamble_baseband = symbols_to_baseband(preamble_symbols);
                for (auto b : preamble_baseband){
                    if(!doBasebandCout && squeue->is_open()){
                        b = ApplyGain(b, tx_out_gain);
                        if(!squeue->put(b, std::chrono::milliseconds(3000))){
                            std::cerr<<"send_preamble(): ERROR OUTPUT QUEUE\n";
                        }
                    }
                    else{
                        std::cerr<<"send_preamble(): ERROR CLOSED QUEUE\n";
                        std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
                    }
                } 
            }
            break;
    }
}

constexpr std::array<uint8_t, 2> SYNC_WORD = {0x32, 0x43};
constexpr std::array<uint8_t, 2> LSF_SYNC_WORD = {0x55, 0xF7};
constexpr std::array<uint8_t, 2> STREAM_SYNC_WORD = {0xFF, 0x5D};
constexpr std::array<uint8_t, 2> PACKET_SYNC_WORD = {0xFF, 0x5D};
constexpr std::array<uint8_t, 2> BERT_SYNC_WORD = {0xDF, 0x55};
constexpr std::array<uint8_t, 2> EOT_SYNC = { 0x55, 0x5D };

void output_eot()
{
	std::cerr << "Sending EOT." << std::endl;
    switch(outputType) {
        case OutputType::SYM:
            {
                std::array<int8_t, 192> out_symbols;
                auto symbols = bytes_to_symbols(EOT_SYNC);
                auto repeat = out_symbols.size()/symbols.size();
                for (size_t j = 0; j < repeat; j++)
                {
                    auto offset = symbols.size()*j;
                    for (size_t i = 0; i < symbols.size(); i++)
                    {
                        out_symbols[offset+i] = symbols[i];
                    }
                }
                for (auto b : out_symbols) std::cout << b;
            }
            break;
        case OutputType::BIN:
            for (size_t i = 0; i < 24; ++i) for (auto c : EOT_SYNC) std::cout << c;
            for (size_t i = 0; i !=10; ++i) std::cout << '\0'; // Flush RRC FIR Filter.
            break;
        default: 
            { // OutputType::RRC
                std::array<int8_t, 192> out_symbols;
                auto symbols = bytes_to_symbols(EOT_SYNC);
                auto repeat = out_symbols.size()/symbols.size();
                for (size_t j = 0; j < repeat; j++)
                {
                    auto offset = symbols.size()*j;
                    for (size_t i = 0; i < symbols.size(); i++)
                    {
                        out_symbols[offset+i] = symbols[i];
                    }
                }
                auto baseband = symbols_to_baseband(out_symbols);
                for (auto b : baseband){
                    if(!doBasebandCout && squeue->is_open()){
                        b = ApplyGain(b, tx_out_gain);
                        if(!squeue->put(b, std::chrono::seconds(300))){
                            std::cerr<<"send_eot(): ERROR OUTPUT QUEUE\n";
                        }
                    }
                    else{
                        std::cerr<<"send_eot(): ERROR CLOSED QUEUE\n";
                        std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
                    }
                }
				
				std::array<int8_t, 192> flush_symbols;
				flush_symbols.fill(0);
				auto f_baseband = symbols_to_baseband(flush_symbols);
                for (auto b : f_baseband) {
                    if(!doBasebandCout && squeue->is_open()){
                        if(!squeue->put(b, std::chrono::seconds(300))){
                            std::cerr<<"send_eot(): ERROR OUTPUT QUEUE\n";
                        }
                    }
                    else{
                        std::cerr<<"send_eot(): ERROR CLOSED QUEUE\n";
                        std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
                    }
                }
            }
            break;
    }
}

lsf_t send_lsf(const std::string& src, const std::string& dest, const FrameType type = FrameType::AUDIO)
{
    using namespace mobilinkd;

    lsf_t result;
    result.fill(0);
    
    M17Randomizer<368> randomizer;
    PolynomialInterleaver<45, 92, 368> interleaver;
    CRC16<0x5935, 0xFFFF> crc;

    std::cerr << "Sending link setup." << std::endl;

    mobilinkd::LinkSetupFrame::call_t callsign;
    callsign.fill(0);
    std::copy(src.begin(), src.end(), callsign.begin());
    auto encoded_src = mobilinkd::LinkSetupFrame::encode_callsign(callsign);

     mobilinkd::LinkSetupFrame::encoded_call_t encoded_dest = {0xff,0xff,0xff,0xff,0xff,0xff};
     if (!dest.empty())
     {
        callsign.fill(0);
        std::copy(dest.begin(), dest.end(), callsign.begin());
        encoded_dest = mobilinkd::LinkSetupFrame::encode_callsign(callsign);
     }

    auto rit = std::copy(encoded_dest.begin(), encoded_dest.end(), result.begin());
    std::copy(encoded_src.begin(), encoded_src.end(), rit);
    if (type == FrameType::AUDIO) {
        result[12] = can >> 1;
        result[13] = 5 | ((can & 1) << 7);
    } else if (type == FrameType::BERT) {
        result[12] = 0;
        result[13] = 1;
    }
	
	if(enc_key){
		//set enc bits 01 - AES
		result[13] |= (1<<3);
		
		uint32_t timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
		uint32_t random_data[2] = {uint_dist(rng), uint_dist(rng)};
		uint16_t CTR_HIGH = uint_dist(rng);
		
		memcpy(Iv, &timestamp, sizeof(uint32_t));
		memcpy(Iv+4, random_data, 2*sizeof(uint32_t));
		memcpy(Iv+12, &CTR_HIGH, sizeof(uint16_t));
		
		//frame number 
		memset(Iv+14, 0, sizeof(uint16_t));
		
		//copy META field
		std::copy(std::begin(Iv),std::end(Iv)-2,result.begin()+14);
		
		//init IV
		AES_init_ctx_iv(&ctx, Key, Iv);
		
	}

    crc.reset();
    for (size_t i = 0; i != 28; ++i)
    {
        crc(result[i]);
    }
    auto checksum = crc.get_bytes();
    result[28] = checksum[0];
    result[29] = checksum[1];

    std::array<uint8_t, 488> encoded;
    size_t index = 0;
    uint32_t memory = 0;
    for (auto b : result)
    {
        for (size_t i = 0; i != 8; ++i)
        {
            uint32_t x = (b & 0x80) >> 7;
            b <<= 1;
            memory = mobilinkd::update_memory<4>(memory, x);
            encoded[index++] = mobilinkd::convolve_bit(031, memory);
            encoded[index++] = mobilinkd::convolve_bit(027, memory);
        }
    }
    // Flush the encoder.
    for (size_t i = 0; i != 4; ++i)
    {
        memory = mobilinkd::update_memory<4>(memory, 0);
        encoded[index++] = mobilinkd::convolve_bit(031, memory);
        encoded[index++] = mobilinkd::convolve_bit(027, memory);
    }

    std::array<int8_t, 368> punctured;
    auto size = puncture(encoded, punctured, P1);
    assert(size == 368);

    interleaver.interleave(punctured);
    randomizer.randomize(punctured);
    output_frame(LSF_SYNC_WORD, punctured);
    return result;
}

/**
 * Encode 2 frames of data.  Caller must ensure that the audio is
 * padded with 0s if the incoming data is incomplete.
 */
codec_frame_t encode(struct CODEC2* codec2, const audio_frame_t& audio)
{
    codec_frame_t result;
    codec2_encode(codec2, &result[0], const_cast<int16_t*>(&audio[0]));
    codec2_encode(codec2, &result[8], const_cast<int16_t*>(&audio[160]));
    return result;
}

data_frame_t make_data_frame(uint16_t frame_number, const codec_frame_t& payload)
{
    std::array<uint8_t, 18> data;   // FN, Audio = 2 + 16;
    data[0] = uint8_t((frame_number >> 8) & 0xFF);
    data[1] = uint8_t(frame_number & 0xFF);
	
    std::copy(payload.begin(), payload.end(), data.begin() + 2);

	if(enc_key){
		uint8_t* p_payload = data.data()+2;

		//update IV with FN
		Iv[14] = uint8_t((frame_number >> 8) & 0xFF);
		Iv[15] = uint8_t(frame_number & 0xFF);
		AES_init_ctx_iv(&ctx, Key, Iv);
	
		AES_CTR_xcrypt_buffer(&ctx, p_payload, 16);
	}

    std::array<uint8_t, 296> encoded;
    size_t index = 0;
    uint32_t memory = 0;
    for (auto b : data)
    {
        for (size_t i = 0; i != 8; ++i)
        {
            uint32_t x = (b & 0x80) >> 7;
            b <<= 1;
            memory = mobilinkd::update_memory<4>(memory, x);
            encoded[index++] = mobilinkd::convolve_bit(031, memory);
            encoded[index++] = mobilinkd::convolve_bit(027, memory);
        }
    }
    // Flush the encoder.
    for (size_t i = 0; i != 4; ++i)
    {
        memory = mobilinkd::update_memory<4>(memory, 0);
        encoded[index++] = mobilinkd::convolve_bit(031, memory);
        encoded[index++] = mobilinkd::convolve_bit(027, memory);
    }

    data_frame_t punctured;
    auto size = mobilinkd::puncture(encoded, punctured, mobilinkd::P2);
    assert(size == 272);
    return punctured;
}

template <typename PRBS>
bitstream_t make_bert_frame(PRBS& prbs)
{
    std::array<uint8_t, 25> data;   // 24.6125 bytes, 197 bits

    // Generate the data.
    for (size_t i = 0; i != data.size() - 1; ++i) {
        uint8_t byte = 0;
        for (int i = 0; i != 8; ++i) {
            byte <<= 1;
            byte |= prbs.generate();
        }
        data[i] = byte;
    }

    uint8_t byte = 0;
    for (int i = 0; i != 5; ++i) {
        byte <<= 1;
        byte |= prbs.generate();
    }
    byte <<= 3;
    data[24] = byte;


    std::array<uint8_t, 402> encoded;
    size_t index = 0;
    uint32_t memory = 0;
    for (size_t i = 0; i != data.size() - 1; ++i)
    {
        auto b = data[i];
        for (size_t j = 0; j != 8; ++j)
        {
            uint32_t x = (b & 0x80) >> 7;
            b <<= 1;
            memory = mobilinkd::update_memory<4>(memory, x);
            encoded[index++] = mobilinkd::convolve_bit(031, memory);
            encoded[index++] = mobilinkd::convolve_bit(027, memory);
        }
    }

    auto b = data[24];
    for (size_t j = 0; j != 5; ++j)
    {
        uint32_t x = (b & 0x80) >> 7;
        b <<= 1;
        memory = mobilinkd::update_memory<4>(memory, x);
        encoded[index++] = mobilinkd::convolve_bit(031, memory);
        encoded[index++] = mobilinkd::convolve_bit(027, memory);
    }

    // Flush the encoder.
    for (size_t i = 0; i != 4; ++i)
    {
        memory = mobilinkd::update_memory<4>(memory, 0);
        encoded[index++] = mobilinkd::convolve_bit(031, memory);
        encoded[index++] = mobilinkd::convolve_bit(027, memory);
    }

    bitstream_t punctured;
    auto size = mobilinkd::puncture(encoded, punctured, mobilinkd::P2);
    assert(size == 368);
    return punctured;
}

/**
 * Encode each LSF segment into a Golay-encoded LICH segment bitstream.
 */
lich_segment_t make_lich_segment(std::array<uint8_t, 5> segment, uint8_t segment_number)
{
    lich_segment_t result;
    uint16_t tmp;
    uint32_t encoded;

    tmp = segment[0] << 4 | ((segment[1] >> 4) & 0x0F);
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 0; i != 24; ++i)
    {
        result[i] = (encoded & (1 << 23)) != 0;
        encoded <<= 1;
    }

    tmp = ((segment[1] & 0x0F) << 8) | segment[2];
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 24; i != 48; ++i)
    {
        result[i] = (encoded & (1 << 23)) != 0;
        encoded <<= 1;
    }

    tmp = segment[3] << 4 | ((segment[4] >> 4) & 0x0F);
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 48; i != 72; ++i)
    {
        result[i] = (encoded & (1 << 23)) != 0;
        encoded <<= 1;
    }

    tmp = ((segment[4] & 0x0F) << 8) | (segment_number << 5);
    encoded = mobilinkd::Golay24::encode24(tmp);
    for (size_t i = 72; i != 96; ++i)
    {
        result[i] = (encoded & (1 << 23)) != 0;
        encoded <<= 1;
    }

    return result;
}

void send_audio_frame(const lich_segment_t& lich, const data_frame_t& data)
{
    using namespace mobilinkd;

    std::array<int8_t, 368> temp;
    auto it = std::copy(lich.begin(), lich.end(), temp.begin());
    std::copy(data.begin(), data.end(), it);

    M17Randomizer<368> randomizer;
    PolynomialInterleaver<45, 92, 368> interleaver;

    interleaver.interleave(temp);
    randomizer.randomize(temp);
    output_frame(STREAM_SYNC_WORD, temp);
}

void transmit(std::shared_ptr<queue_t>& queue, const lsf_t& lsf)
{
    using namespace mobilinkd;

    assert(running);

    lich_t lich;
    for (size_t i = 0; i != lich.size(); ++i)
    {
        std::array<uint8_t, 5> segment;
        std::copy(lsf.begin() + i * 5, lsf.begin() + (i + 1) * 5, segment.begin());
        auto lich_segment = make_lich_segment(segment, i);
        std::copy(lich_segment.begin(), lich_segment.end(), lich[i].begin());
    }
    
    struct CODEC2* codec2 = ::codec2_create(CODEC2_MODE_3200);

    M17Randomizer<368> randomizer;
    PolynomialInterleaver<45, 92, 368> interleaver;
    CRC16<0x5935, 0xFFFF> crc;

    audio_frame_t audio;
    size_t index = 0;
    uint16_t frame_number = 0;
    uint8_t lich_segment = 0;
    while(!queue->is_closed() && queue->empty()) std::this_thread::yield();
    while (!queue->is_closed())
    {
        int16_t sample;
        if (!queue->get(sample, std::chrono::milliseconds(3000))) break;
        audio[index++] = ApplyGain(sample, tx_mic_gain);
        if (index == audio.size())
        {
            index = 0;
            auto data = make_data_frame(frame_number++, encode(codec2, audio));
            if (frame_number == 0x8000) frame_number = 0;
            send_audio_frame(lich[lich_segment++], data);
            if (lich_segment == lich.size()) lich_segment = 0;
            audio.fill(0);
        } 
    }
	
    if (index > 0)
    {
        // send parial frame;
        auto data = make_data_frame(frame_number++, encode(codec2, audio));
        if (frame_number == 0x8000) frame_number = 0;
        send_audio_frame(lich[lich_segment++], data);
        if (lich_segment == lich.size()) lich_segment = 0;
    }
	
    // Last frame
    audio.fill(0);
    auto data = make_data_frame(frame_number | 0x8000, encode(codec2, audio));
    send_audio_frame(lich[lich_segment], data);
    output_eot();
	
	if(has_ptt){
        std::cerr << "\r\nPTT: OFF \n";	
		system(ptt_off.c_str());
	}
	
    codec2_destroy(codec2);
}



int record( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData )
{
    if ( status )
        std::cerr << "(record) Stream overflow detected!" << std::endl;
    
    for(unsigned int i=0; i<nBufferFrames; i++){
        uint16_t sample = ((int16_t*)inputBuffer)[i];
        if(!basebandQueue->put(sample, std::chrono::seconds(300))){
            std::cerr<<"record(): ERROR INPUT QUEUE";
        }
    }
    return 0;
}

//int last = 0;

int playback( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData )
{
    if ( status )
        std::cerr << "(playback) Stream overflow detected!" << std::endl;

    int16_t *buffer = (int16_t *) outputBuffer;

    for(unsigned int i=0; i<nBufferFrames; i++){
        int16_t sample; 
        if(squeue->is_closed()){
            sample = 0;
        }else if(!squeue->get(sample, std::chrono::milliseconds(3000))){
            break;
        }
        for (unsigned int j=0; j<2; j++ ) {
            *buffer++ = sample;
        }
    }

    return 0;
}

void drawTestDock()
{
	bool open = true;

	ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;
	ImGuiViewport* viewport = ImGui::GetMainViewport();
	ImGui::SetNextWindowPos(viewport->Pos);
	ImGui::SetNextWindowSize(viewport->Size);
	ImGui::SetNextWindowViewport(viewport->ID);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
	window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
	window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;

	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
	ImGui::Begin("DockSpace Demo", &open, window_flags);
	ImGui::PopStyleVar();

	ImGui::PopStyleVar(2);

	if (ImGui::DockBuilderGetNode(ImGui::GetID("MyDockspace")) == NULL)
	{
		ImGuiID dockspace_id = ImGui::GetID("MyDockspace");
		ImGuiViewport* viewport = ImGui::GetMainViewport();

        ImGui::DockBuilderRemoveNode(dockspace_id); // Clear out existing layout
        ImGui::DockBuilderAddNode(dockspace_id, window_flags); // Add empty node
        ImGui::DockBuilderSetNodeSize(dockspace_id, viewport->Size);

		ImGuiID dock_main_id = dockspace_id; // This variable will track the document node, however we are not using it here as we aren't docking anything into it.
		ImGuiID dock_id_left = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Left, 0.20f, NULL, &dock_main_id);
		ImGuiID dock_id_right = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Right, 0.20f, NULL, &dock_main_id);
		ImGuiID dock_id_bottom = ImGui::DockBuilderSplitNode(dock_main_id, ImGuiDir_Down, 0.4f, NULL, &dock_main_id);

		
		ImGui::DockBuilderDockWindow("M17 Modulator", dock_main_id);
        ImGui::DockBuilderDockWindow("M17 Demodulator", dock_main_id);
        ImGui::DockBuilderDockWindow("Rig Control", dock_id_bottom);
        ImGui::DockBuilderDockWindow("Audio", dock_id_bottom);
        ImGui::DockBuilderDockWindow("Gains", dock_id_bottom);
		ImGui::DockBuilderFinish(dockspace_id);
	}
    
	ImGuiID dockspace_id = ImGui::GetID("MyDockspace");
	ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), 0);
	ImGui::End();
}


static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "Glfw Error %d: %s\n", error, description);
}

void func(std::shared_ptr<queue_t>& queue){
    std::cout << "Hi I'm Thready McThreadFace\n";
    while(!queue->is_closed() && queue->empty()){
        std::this_thread::yield(); 
    }
    while (!queue->is_closed())
    {
        int16_t sample;
        if (!queue->get(sample, std::chrono::milliseconds(3000))) break;
        if (invert_input) sample *= -1;
        float s = sample / 44000.0f;
        s = ApplyGainF(s,rx_gain);
        (*pdemod)(s);
    }
}

void saveConfig(const std::string& filename, std::optional<Config>& config) {
    std::ofstream file(filename);
    if (file.is_open()) {
        file << config->source_address << std::endl;
        file << config->destination_address << std::endl;
        file << config->audio_device << std::endl;
        file << config->event_device << std::endl;
        file << config->tx_on_cmd << std::endl;
        file << config->tx_off_cmd << std::endl;
        file << config->action_ptt << std::endl;
        file << config->key << std::endl;
        file << config->verbose << std::endl;
        file << config->debug << std::endl;
        file << config->quiet << std::endl;
        file << config->invert << std::endl;
        file << config->lsf << std::endl;
        file << config->noise_blanker << std::endl;
        file << config->bin << std::endl;
        file << config->sym << std::endl;
        file << config->rrc << std::endl;
        file << config->bert << std::endl;
        file << config->can << std::endl;
        file << config->encrypt << std::endl;
        file << config->CKEY << std::endl;
        file << config->dev_IDs[0] << std::endl;
        file << config->dev_IDs[1] << std::endl;
        file << config->dev_IDs[2] << std::endl;
        file << config->dev_IDs[3] << std::endl;
        file << config->dev_Gains[0] << std::endl;
        file << config->dev_Gains[1] << std::endl;
        file << config->dev_Gains[2] << std::endl;
        file << config->dev_Gains[3] << std::endl;
        file << config->rig_enabled << std::endl;
        file << config->ser_id << std::endl;
        file << config->baud_id << std::endl;
        file << config->ptt_id << std::endl;

        file.close();
    } else {
        std::cerr << "Unable to open file for writing." << std::endl;
    }
}

bool readConfig(const std::string& filename, std::optional<Config>& config) {
    std::ifstream file(filename);
    if (file.is_open()) {
        std::string line;

        // Read each line and assign it to the corresponding member of the Config struct
        std::getline(file, config->source_address);
        std::getline(file, config->destination_address);
        std::getline(file, config->audio_device);
        std::getline(file, config->event_device);
        std::getline(file, config->tx_on_cmd);
        std::getline(file, config->tx_off_cmd);
        file >> config->action_ptt;
        file >> config->key;
        file >> config->verbose;
        file >> config->debug;
        file >> config->quiet;
        file >> config->invert;
        file >> config->lsf;
        file >> config->noise_blanker;
        file >> config->bin;
        file >> config->sym;
        file >> config->rrc;
        file >> config->bert;
        file >> config->can;
        file >> config->encrypt;
        std::getline(file >> std::ws, config->CKEY); // Read the remaining part of the line (including spaces)
        file >>  config->dev_IDs[0];
        file >>  config->dev_IDs[1];
        file >>  config->dev_IDs[2];
        file >>  config->dev_IDs[3];
        file >>  config->dev_Gains[0];
        file >>  config->dev_Gains[1];
        file >>  config->dev_Gains[2];
        file >>  config->dev_Gains[3];
        file >>  config->rig_enabled;
        file >>  config->ser_id;
        file >>  config->baud_id;
        file >>  config->ptt_id;

        file.close();
        return true;
    } else {
        std::cerr << "Unable to open file for reading." << std::endl;
        return false;
    }
}

int main(int argc, char* argv[])
{
    using namespace mobilinkd;

    auto config = Config::parse(argc, argv);
    if (!config) return 0;

    (*pdemod).diagnostics(diagnostic_callback<float>);

    std::string fname("config.ini");
    if(!readConfig(fname,config)){
        std::cerr << "No default cfg loaded.\n";
        config->lsf = true;
        config->debug = true;
    }

    display_lsf = config->lsf;
    invert_input = config->invert;
    invert = config->invert;
    quiet = config->quiet;
    debug = config->debug;
    noise_blanker = config->noise_blanker;
	enc_key = config->encrypt;

    has_ptt = config->action_ptt;
	ptt_on = config->tx_on_cmd;
	ptt_off = config->tx_off_cmd;

    can = config->can;
	
	std::string hash = boost::algorithm::unhex(config->CKEY);
	std::copy(hash.begin(), hash.end(), Key);

    diag = std::make_unique<Diagnostics>();

    if (config->sym) {
        inputType = InputType::SYM;
        outputType = OutputType::SYM;
    }
    else if (config->bin) {
        inputType = InputType::BIN;
        outputType = OutputType::BIN;
    }
    else {
        inputType = InputType::RRC;
        outputType = OutputType::RRC;
    }

    codec2 = ::codec2_create(CODEC2_MODE_3200);

    // Setup window
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(300, 600, "M17-RTX Gui", NULL, NULL);
    if (window == NULL)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;       // Enable Keyboard Controls
    //io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;           // Enable Docking
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;         // Enable Multi-Viewport / Platform Windows
    //io.ConfigViewportsNoAutoMerge = true;
    //io.ConfigViewportsNoTaskBarIcon = true;

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // When viewports are enabled we tweak WindowRounding/WindowBg so platform windows can look identical to regular ones.
    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Load Fonts
    // - If no fonts are loaded, dear imgui will use the default font. You can also load multiple fonts and use ImGui::PushFont()/PopFont() to select them.
    // - AddFontFromFileTTF() will return the ImFont* so you can store it if you need to select the font among multiple.
    // - If the file cannot be loaded, the function will return NULL. Please handle those errors in your application (e.g. use an assertion, or display an error and quit).
    // - The fonts will be rasterized at a given size (w/ oversampling) and stored into a texture when calling ImFontAtlas::Build()/GetTexDataAsXXXX(), which ImGui_ImplXXXX_NewFrame below will call.
    // - Use '#define IMGUI_ENABLE_FREETYPE' in your imconfig file to use Freetype for higher quality font rendering.
    // - Read 'docs/FONTS.md' for more instructions and details.
    // - Remember that in C/C++ if you want to include a backslash \ in a string literal you need to write a double backslash \\ !
    //io.Fonts->AddFontDefault();
    //io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\segoeui.ttf", 18.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/DroidSans.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Roboto-Medium.ttf", 16.0f);
    //io.Fonts->AddFontFromFileTTF("../../misc/fonts/Cousine-Regular.ttf", 15.0f);
    //ImFont* font = io.Fonts->AddFontFromFileTTF("c:\\Windows\\Fonts\\ArialUni.ttf", 18.0f, NULL, io.Fonts->GetGlyphRangesJapanese());
    //IM_ASSERT(font != NULL);

    // Our state
    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    bool rx = false;
    bool tx = false;
    bool rig_enabled = config->rig_enabled;

    static char str1[8] = "";
    static char str2[8] = "ALL";
    static char buf[128] = "";

    std::copy(config->source_address.begin(),config->source_address.end(),&str1[0]);
    std::copy(config->destination_address.begin(),config->destination_address.end(),&str2[0]);
    std::copy(config->CKEY.begin(),config->CKEY.end(),&buf[0]);

    std::string ptt[2] = {"OFF","ON"};

    AudioSource BasebandSrc(48000u, 1920u, config->dev_IDs[0]);
    AudioSource VoiceSrc(8000u, 320u, config->dev_IDs[1]);

    AudioSink VoiceSink(8000u, 320u, config->dev_IDs[2]);
    AudioSink BasebandSink(48000u, 64u, config->dev_IDs[3]);

    BasebandSrc.SetCallback(&record);
    VoiceSrc.SetCallback(&record);

    BasebandSink.SetCallback(&playback);
    VoiceSink.SetCallback(&playback);

    BasebandSrc.Open();
    VoiceSink.Open();

    std::thread thd;

    serialib serial;
    bool hasSerial = false;
    std::vector<std::string> Serialports = get_available_ports(serial);
    int port_id = config->ser_id;

    if(Serialports.size() == 0){
        std::cerr << "No serial ports found.\n";
        Serialports.push_back("NONE");
    }else{
        hasSerial = true;
    }

    std::vector<int> Serialbaud = {115200};
    int baud_id = config->baud_id;

    std::vector<std::string> Serialptt = {"DTR","RTS"};
    int ptt_id = config->ptt_id;

    if(rig_enabled && hasSerial){
        std::cerr << "Trying to open: " << Serialports[port_id] << "\n";
        char errorOpening = serial.openDevice(Serialports[port_id].c_str(), 115200);
        // If connection fails, return the error code otherwise, display a success message
        if (errorOpening!=1){
            std::cerr << "Error: " << errorOpening << "\n";
        }else{
            std::cerr << "Serial: " << Serialports[port_id] << "Is open!\n";
        }
        serial.RTS(false);
        serial.DTR(false);
    }

    bool StartRx=false;
    bool StopRx=false;

    float g0 = config->dev_Gains[0];
    float g1 = config->dev_Gains[1];
    float g2 = config->dev_Gains[2];
    float g3 = config->dev_Gains[3];

    tx_out_gain = &g3;
    tx_mic_gain = &g0;
    rx_gain = &g1;
    spk_gain = &g2;


    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            drawTestDock();
        }

        {
            ImGui::Begin("M17 Modulator");
            ImGui::InputTextWithHint("SRC", "SRC CALLSIGN", str1, 8, ImGuiInputTextFlags_CharsUppercase);
            ImGui::InputTextWithHint("DST", "DST CALLSIGN", str2, 8, ImGuiInputTextFlags_CharsUppercase);
            ImGui::InputInt("CAN", &config->can);
	            config->can = std::min<int>(15,std::max<int>(0,config->can));
            ImGui::Checkbox("Invert Polarity", &config->invert);
            ImGui::Checkbox("Encrypt", &config->encrypt);
            ImGui::InputText("AES Key", buf, 128, ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);

            if(tx){
                ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(0 / 7.0f, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0 / 7.0f, 0.4f, 0.6f));
            }
            else{
                ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(2 / 7.0f, 0.4f, 0.6f));
            }

            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.0f);
            ImVec2 button_sz(ImGui::GetContentRegionAvail().x, 40);

            auto btn = ImGui::Button("PTT",button_sz);

            ImGui::PopStyleColor(2);
            ImGui::PopStyleVar(1);           

            if (btn || (StopRx && !rx) ){
                config->source_address = std::string(str1);
                config->destination_address = std::string(str2);
                if(config->source_address.length()>2){
                    if(StopRx){
                        StopRx=false;
                    }
                    if(!running){
                        invert = config->invert;
                        can = config->can;
                        enc_key = config->encrypt;
                        
                        //has_ptt = config->action_ptt;
                        //ptt_on = config->tx_on_cmd;
                        //ptt_off = config->tx_off_cmd;

                        config->CKEY = std::string(buf);
                        std::string hash = boost::algorithm::unhex(config->CKEY);
                        std::copy(hash.begin(), hash.end(), Key);
                       
                        squeue = std::make_shared<queue_t>();
                        basebandQueue = std::make_shared<queue_t>();

                        VoiceSrc.Start();
                        BasebandSink.Start();

                        if(has_ptt){
                            std::cerr << "\r\nPTT: ON \n";          
                            system(ptt_on.c_str());
                        }

                        if(rig_enabled && hasSerial){
                            switch (ptt_id)
                            {
                            case 0:
                                serial.DTR(true);
                                break;
                            case 1:
                                serial.RTS(true);
                                serial.DTR(true);
                                break;
                            }
                        }

                        send_preamble();
                        send_preamble();
                    
                        auto lsf = send_lsf(config->source_address, config->destination_address);

                        running = true;

                        thd = std::thread(transmit, std::ref(basebandQueue), std::ref(lsf));
                    
                        std::cerr << "m17-mod running. ctrl-D to break." << std::endl;

                        tx = true;

                    }else if(rx){
                        StopRx = true;
                    }else{
                        running = false;

                        VoiceSrc.Stop();
                        basebandQueue.get()->close();
                        
                        thd.join(); 

                        squeue.get()->close();
                        BasebandSink.Stop();              

                        if(rig_enabled && hasSerial){
                            std::this_thread::sleep_for(std::chrono::milliseconds(40)); // Ptt delay
                            switch (ptt_id)
                            {
                            case 0:
                                serial.DTR(false);
                                break;
                            case 1:
                                serial.RTS(false);
                                serial.DTR(false);
                                break;
                            }
                        }
                        
                        if(has_ptt){
                            std::cerr << "\r\nPTT: OFF \n";          
                            system(ptt_off.c_str());
                        }
                        tx = false;
                        StartRx=true;
                    }
                }
            }
        }

        {
            ImGui::Begin("Gains");
            ImGui::Text("Mic Gain:");
            ImGui::SliderFloat("##mic_gain", tx_mic_gain, 0.0f, 1.0f, "%.01f");

            ImGui::Text("Rx Gain:");
            ImGui::SliderFloat("##rx_gain", rx_gain, 0.0f, 1.5f, "%.01f");

            ImGui::Text("Speaker Gain:");
            ImGui::SliderFloat("##spk_gain", spk_gain, 0.0f, 1.0f, "%.01f");

            ImGui::Text("Tx Gain:");
            ImGui::SliderFloat("##tx_gain", tx_out_gain, 0.0f, 1.0f, "%.01f");
            ImGui::End();
        }

        {
            ImGui::Begin("M17 Demodulator");
            ImGui::Text("SOURCE: %s",diag->source.c_str());
            ImGui::Text("DESTINATION: %s",diag->dest.c_str());
            ImGui::Text("DCD: %s",diag->dcd.c_str());
            ImGui::Text("Lock: %s",diag->lock.c_str());
            ImGui::Text("EVM: %.1f %%",diag->evm);
            ImGui::Text("Deviation: %.1f %%",diag->dev*100.0f);

            float val = diag->dev;
            if(val>0.5f && val<1.0f){
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, (ImVec4)ImColor::HSV(2.0f/7.0f, 0.6f, 0.6f));
            }else{
                ImGui::PushStyleColor(ImGuiCol_PlotHistogram, (ImVec4)ImColor::HSV(0.0f/7.0f, 0.6f, 0.6f));
            }
            
            ImGui::ProgressBar(val, ImVec2(0.0f, 0.0f));
            //ImGui::SameLine(0.0f, ImGui::GetStyle().ItemInnerSpacing.x);
            ImGui::PopStyleColor(1);

            ImGui::Checkbox("Invert Polarity", &config->invert);
            ImGui::Checkbox("Decrypt", &config->encrypt);
            ImGui::InputText("AES Key", buf, 128, ImGuiInputTextFlags_CharsHexadecimal | ImGuiInputTextFlags_CharsUppercase);

            if(rx){
                ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(1 / 7.0f, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(0 / 7.0f, 0.4f, 0.6f));
            }
            else{
                ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(2 / 7.0f, 0.6f, 0.6f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(2 / 7.0f, 0.4f, 0.6f));
            }

            ImGui::PushStyleVar(ImGuiStyleVar_FrameRounding, 5.0f);
            ImVec2 button_sz(ImGui::GetContentRegionAvail().x, 40);

            auto btn = ImGui::Button("START",button_sz);

            ImGui::PopStyleColor(2);
            ImGui::PopStyleVar(1);

            if (btn || StartRx || StopRx){
                if(1){
                    ResetDiag(diag.get());
                    if(!running && !tx && !rx){
                        invert = config->invert;
                        enc_key = config->encrypt;
        
                        config->CKEY = std::string(buf);
                        std::string hash = boost::algorithm::unhex(config->CKEY);
                        std::copy(hash.begin(), hash.end(), Key);
                       
                        squeue = std::make_shared<queue_t>();
                        basebandQueue = std::make_shared<queue_t>();

                        BasebandSrc.Start();
                        VoiceSink.Start();

                        if(rig_enabled && hasSerial){
                            std::this_thread::sleep_for(std::chrono::milliseconds(40)); // Ptt delay
                            switch (ptt_id)
                            {
                            case 0:
                                serial.DTR(false);
                                break;
                            case 1:
                                serial.RTS(false);
                                serial.DTR(false);
                                break;
                            }
                        }
                        
                        if(has_ptt){
                            std::cerr << "\r\nPTT: OFF \n";          
                            system(ptt_off.c_str());
                        }

                        running = true;

                        thd = std::thread(func, std::ref(basebandQueue));
                    
                        std::cerr << "m17-demod running. ctrl-D to break." << std::endl;

                        rx = true;
                        StartRx=false;
                    }else{
                        running = false;
                        
                        BasebandSrc.Stop();
                        basebandQueue.get()->close();
                        
                        thd.join(); 

                        squeue.get()->close();
                        VoiceSink.Stop();          
                        rx = false;
                    }
                }
            }
            ImGui::End();
        }

        {
            ImGui::Begin("Audio");
            ImGui::Text("Input Device [MIC]:");
            float menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("input_audio_0",VoiceSrc.GetCurrentDeviceNameC())) {
                for (int i = 0; i < VoiceSrc.GetNumDevices(); ++i) {
                    const bool isSelected = (VoiceSrc.GetCurrentDeviceId() == i);
                    if (ImGui::Selectable(VoiceSrc.GetDeviceNameC(i), isSelected)) {
                        if(running){
                            rx = false;
                            running = false;

                            BasebandSrc.Stop();
                            VoiceSrc.Stop();
                            basebandQueue.get()->close();
                            
                            thd.join(); 

                            squeue.get()->close();
                            BasebandSink.Stop();
                            VoiceSink.Stop();
                        }
                        VoiceSrc.ChangeDevice(i);
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::Text("Input Device [RX]:");
            menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("input_audio_1",BasebandSrc.GetCurrentDeviceNameC())) {
                for (int i = 0; i < BasebandSrc.GetNumDevices(); ++i) {
                    const bool isSelected = (BasebandSrc.GetCurrentDeviceId() == i);
                    if (ImGui::Selectable(BasebandSrc.GetDeviceNameC(i), isSelected)) {
                        if(running){
                            rx = false;
                            running = false;

                            BasebandSrc.Stop();
                            VoiceSrc.Stop();
                            basebandQueue.get()->close();
                            
                            thd.join(); 

                            squeue.get()->close();
                            BasebandSink.Stop();
                            VoiceSink.Stop();
                        }
                        BasebandSrc.ChangeDevice(i);
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::Text("Output Device [SPK]:");
            menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("output_audio_0",VoiceSink.GetCurrentDeviceNameC())) {
                for (int i = 0; i < VoiceSink.GetNumDevices(); ++i) {
                    const bool isSelected = (VoiceSink.GetCurrentDeviceId() == i);
                    if (ImGui::Selectable(VoiceSink.GetDeviceNameC(i), isSelected)) {
                        if(running){
                            rx = false;
                            running = false;

                            BasebandSrc.Stop();
                            VoiceSrc.Stop();
                            basebandQueue.get()->close();
                            
                            thd.join(); 

                            squeue.get()->close();
                            BasebandSink.Stop();
                            VoiceSink.Stop();
                        }
                        VoiceSink.ChangeDevice(i);
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::Text("Output Device [TX]:");
            menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("output_audio_1",BasebandSink.GetCurrentDeviceNameC())) {
                for (int i = 0; i < BasebandSink.GetNumDevices(); ++i) {
                    const bool isSelected = (BasebandSink.GetCurrentDeviceId() == i);
                    if (ImGui::Selectable(BasebandSink.GetDeviceNameC(i), isSelected)) {
                        if(running){
                            rx = false;
                            running = false;

                            BasebandSrc.Stop();
                            VoiceSrc.Stop();
                            basebandQueue.get()->close();
                            
                            thd.join(); 

                            squeue.get()->close();
                            BasebandSink.Stop();
                            VoiceSink.Stop();
                        }
                        BasebandSink.ChangeDevice(i);
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }

            ImGui::End();
        }

        {
            ImGui::Begin("Rig Control");
            if(ImGui::Checkbox("Enable", &rig_enabled)){
                if(hasSerial){
                    if(rig_enabled){
                        std::cerr << "Trying to open: " << Serialports[port_id] << "\n";
                        char errorOpening = serial.openDevice(Serialports[port_id].c_str(), 115200);
                        // If connection fails, return the error code otherwise, display a success message
                        if (errorOpening!=1) std::cerr << "Error: " << errorOpening << "\n";
                        serial.RTS(false);
                        serial.DTR(false);
                    }else{
                        serial.closeDevice();
                    }
                }
            }
            ImGui::Text("Serial Port:");
            float menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("port_menu",Serialports[port_id].c_str())) {
                for (int i = 0; i < Serialports.size(); ++i) {
                    const bool isSelected = (port_id == i);
                    if (ImGui::Selectable(Serialports[i].c_str(), isSelected)) {
                        port_id = i;

                        if(running){
                            rx = false;
                            running = false;
                            
                            BasebandSrc.Stop();
                            VoiceSrc.Stop();
                            basebandQueue.get()->close();
                            
                            thd.join(); 

                            squeue.get()->close();
                            BasebandSink.Stop();
                            VoiceSink.Stop();

                            if(rig_enabled && hasSerial){
                                serial.closeDevice();
                            }
                        }

                        if(rig_enabled && hasSerial){
                            serial.closeDevice();
                            std::cerr << "Trying to open: " << Serialports[port_id] << "\n";
                            char errorOpening = serial.openDevice(Serialports[port_id].c_str(), 115200);
                            // If connection fails, return the error code otherwise, display a success message
                            if (errorOpening!=1){
                                std::cerr << "Error: " << errorOpening << "\n";
                            }else{
                                std::cerr << "Serial: " << Serialports[port_id] << "Is open!\n";
                            }
                            serial.RTS(false);
                            serial.DTR(false);
                        }
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Text("Baud Rate:");
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("baud_menu",std::to_string(Serialbaud[baud_id]).c_str())) {
                for (int i = 0; i < Serialbaud.size(); ++i) {
                    const bool isSelected = (baud_id == i);
                    if (ImGui::Selectable(std::to_string(Serialbaud[i]).c_str(), isSelected)) {
                        baud_id = i;
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Text("PTT Key:");
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("ptt_menu",Serialptt[ptt_id].c_str())) {
                for (int i = 0; i < Serialptt.size(); ++i) {
                    const bool isSelected = (ptt_id == i);
                    if (ImGui::Selectable(Serialptt[i].c_str(), isSelected)) {
                        ptt_id = i;
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::End();
        }

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        // Update and Render additional Platform Windows
        // (Platform functions may change the current OpenGL context, so we save/restore it to make it easier to paste this code elsewhere.
        //  For this specific demo app we could also call glfwMakeContextCurrent(window) directly)
        if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    if(running){
        running = false;
        
        BasebandSrc.Stop();
        VoiceSrc.Stop();
        basebandQueue.get()->close();
        
        thd.join(); 

        squeue.get()->close();
        BasebandSink.Stop();
        VoiceSink.Stop();

        if(rig_enabled){
            switch (ptt_id)
            {
            case 0:
                serial.DTR(false);
                break;
            case 1:
                serial.RTS(false);
                serial.DTR(false);
                break;
            }
        }
        
        if(has_ptt){
            std::cerr << "\r\nPTT: OFF \n";          
            system(ptt_off.c_str());
        }
    }

    config->CKEY = std::string(buf);
    config->source_address = std::string(str1);
    config->destination_address = std::string(str2);
    config->baud_id = baud_id;
    config->ser_id = port_id;
    config->ptt_id = ptt_id;

    config->rig_enabled = rig_enabled;

    config->dev_Gains[0] = g0;
    config->dev_Gains[1] = g1;
    config->dev_Gains[2] = g2;
    config->dev_Gains[3] = g3;

    config->dev_IDs[0] = BasebandSrc.GetCurrentDeviceId();
    config->dev_IDs[1] = VoiceSrc.GetCurrentDeviceId();
    config->dev_IDs[2] = VoiceSink.GetCurrentDeviceId();
    config->dev_IDs[3] = BasebandSink.GetCurrentDeviceId();

    saveConfig(fname,config);

    return EXIT_SUCCESS;
}
