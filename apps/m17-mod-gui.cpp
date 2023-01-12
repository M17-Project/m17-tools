// Copyright 2020 Mobilinkd LLC.

#include "Util.h"
#include "queue.h"
#include "FirFilter.h"
#include "LinkSetupFrame.h"
#include "CRC16.h"
#include "Trellis.h"
#include "Convolution.h"
#include "PolynomialInterleaver.h"
#include "M17Randomizer.h"
#include "Util.h"
#include "Golay24.h"

#include "M17Modulator.h"

#include <codec2/codec2.h>

#include <boost/program_options.hpp>
#include <boost/optional.hpp>
#include <boost/algorithm/hex.hpp>

#include <thread>

#include <array>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <optional>
#include <mutex>

#include <cstdlib>


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
    }
    return ports;
    }
#endif

uint8_t Key[32];	
uint8_t Iv[16];
struct AES_ctx ctx;

// Generated using scikit-commpy
const auto rrc_taps = std::array<double, 150>{
    0.0029364388513841593, 0.0031468394550958484, 0.002699564567597445, 0.001661182944400927, 0.00023319405581230247, -0.0012851320781224025, -0.0025577136087664687, -0.0032843366522956313, -0.0032697038088887226, -0.0024733964729590865, -0.0010285696910973807, 0.0007766690889758685, 0.002553421969211845, 0.0038920145144327816, 0.004451886520053017, 0.00404219185231544, 0.002674727068399207, 0.0005756567993179152, -0.0018493784971116507, -0.004092346891623224, -0.005648131453822014, -0.006126925416243605, -0.005349511529163396, -0.003403189203405097, -0.0006430502751187517, 0.002365929161655135, 0.004957956568090113, 0.006506845894531803, 0.006569574194782443, 0.0050017573119839134, 0.002017321931508163, -0.0018256054303579805, -0.00571615173291049, -0.008746639552588416, -0.010105075751866371, -0.009265784007800534, -0.006136551625729697, -0.001125978562075172, 0.004891777252042491, 0.01071805138282269, 0.01505751553351295, 0.01679337935001369, 0.015256245142156299, 0.01042830577908502, 0.003031522725559901, -0.0055333532968188165, -0.013403099825723372, -0.018598682349642525, -0.01944761739590459, -0.015005271935951746, -0.0053887880354343935, 0.008056525910253532, 0.022816244158307273, 0.035513467692208076, 0.04244131815783876, 0.04025481153629372, 0.02671818654865632, 0.0013810216516704976, -0.03394615682795165, -0.07502635967975885, -0.11540977897637611, -0.14703962203941534, -0.16119995609538576, -0.14969512896336504, -0.10610329539459686, -0.026921412469634916, 0.08757875030779196, 0.23293327870303457, 0.4006012210123992, 0.5786324696325503, 0.7528286479934068, 0.908262741447522, 1.0309661131633199, 1.1095611856548013, 1.1366197723675815, 1.1095611856548013, 1.0309661131633199, 0.908262741447522, 0.7528286479934068, 0.5786324696325503, 0.4006012210123992, 0.23293327870303457, 0.08757875030779196, -0.026921412469634916, -0.10610329539459686, -0.14969512896336504, -0.16119995609538576, -0.14703962203941534, -0.11540977897637611, -0.07502635967975885, -0.03394615682795165, 0.0013810216516704976, 0.02671818654865632, 0.04025481153629372, 0.04244131815783876, 0.035513467692208076, 0.022816244158307273, 0.008056525910253532, -0.0053887880354343935, -0.015005271935951746, -0.01944761739590459, -0.018598682349642525, -0.013403099825723372, -0.0055333532968188165, 0.003031522725559901, 0.01042830577908502, 0.015256245142156299, 0.01679337935001369, 0.01505751553351295, 0.01071805138282269, 0.004891777252042491, -0.001125978562075172, -0.006136551625729697, -0.009265784007800534, -0.010105075751866371, -0.008746639552588416, -0.00571615173291049, -0.0018256054303579805, 0.002017321931508163, 0.0050017573119839134, 0.006569574194782443, 0.006506845894531803, 0.004957956568090113, 0.002365929161655135, -0.0006430502751187517, -0.003403189203405097, -0.005349511529163396, -0.006126925416243605, -0.005648131453822014, -0.004092346891623224, -0.0018493784971116507, 0.0005756567993179152, 0.002674727068399207, 0.00404219185231544, 0.004451886520053017, 0.0038920145144327816, 0.002553421969211845, 0.0007766690889758685, -0.0010285696910973807, -0.0024733964729590865, -0.0032697038088887226, -0.0032843366522956313, -0.0025577136087664687, -0.0012851320781224025, 0.00023319405581230247, 0.001661182944400927, 0.002699564567597445, 0.0031468394550958484, 0.0029364388513841593, 0.0
};

const auto evm_b = std::array<double, 3>{0.02008337, 0.04016673, 0.02008337};
const auto evm_a = std::array<double, 3>{1.0, -1.56101808, 0.64135154};

const char VERSION[] = "2.2";

std::string ptt_on = "";
std::string ptt_off = "";
bool has_ptt = false;

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
    bool debug = false;
    bool quiet = false;

    bool bin = false;
    bool sym = false;
    bool rrc = true; // default is rrc

    bool bert = false; // Bit error rate testing.
    bool invert = false;
    int can = 10;
	
	bool encrypt = false; //Default is no Encryption
	std::string CKEY; //AES Key
	
    /*
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
            ("src,S", po::value<std::string>(&result.source_address)->required(),
                "transmitter identifier (your callsign).")
            ("dest,D", po::value<std::string>(&result.destination_address),
                "destination (default is broadcast).")
			("transmit-on,T", po::value<std::string>(&result.tx_on_cmd)->default_value(""),
                "Transmit/PTT activate action/command eg: toggle gpio on (default is no action).")
			("transmit-off,O", po::value<std::string>(&result.tx_off_cmd)->default_value(""),
                "Transmit/PTT deactivate action/command eg: toggle gpio off (default is no action).")
            ("can,C", po::value<int>(&result.can)->default_value(10),
                "channel access number.")
            ("audio,a", po::value<std::string>(&result.audio_device),
                "audio device (default is STDIN).")
            ("event,e", po::value<std::string>(&result.event_device)->default_value("/dev/input/by-id/usb-C-Media_Electronics_Inc._USB_Audio_Device-event-if03"),
                "event device (default is C-Media Electronics Inc. USB Audio Device).")
			("encrypt,K",po::value<std::string>(&result.CKEY), "hexadecimal string for AES 128, 192 or 256 Key (default is no encryption).")
            ("key,k", po::value<uint16_t>(&result.key)->default_value(385),
                "Linux event code for PTT (default is RADIO).")
            ("bin,x", po::bool_switch(&result.bin), "output packed dibits (default is rrc).")
            ("rrc,r", po::bool_switch(&result.rrc), "output rrc filtered and scaled symbols (default).")
            ("sym,s", po::bool_switch(&result.sym), "output symbols (default is rrc).")
            ("bert,B", po::bool_switch(&result.bert),
                "output a bit error rate test stream (default is read audio from STDIN).")
            ("invert,i", po::bool_switch(&result.invert), "invert the output baseband (only for rrc)")
            ("verbose,v", po::bool_switch(&result.verbose), "verbose output")
            ("debug,d", po::bool_switch(&result.debug), "debug-level output")
            ("quiet,q", po::bool_switch(&result.quiet), "silence all output")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "Read audio from STDIN and write baseband M17 to STDOUT\n"
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

        if (result.source_address.size() > 9)
        {
            std::cerr << "Source identifier too long." << std::endl;
            return std::nullopt;
        }

        if (result.destination_address.size() > 9)
        {
            std::cerr << "Destination identifier too long." << std::endl;
            return std::nullopt;
        }

        if (result.can < 0 || result.can > 15) {
            std::cerr << "invalid channel access number (CAN) " << result.can << ". Must be 0-15." << std::endl;
            return std::nullopt;
        }

        if (result.sym + result.bin + result.rrc > 1)
        {
            std::cerr << "Only one of sym, bin or rrc may be chosen." << std::endl;
            return std::nullopt;
        }
		
		if (result.tx_on_cmd.size() > 0 && result.tx_off_cmd.size() > 0)
        {
			result.action_ptt = true;
            std::cerr << "PTT action provided." << std::endl;
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
    */
};

enum class FrameType {AUDIO, DATA, MIXED, BERT};

using lsf_t = std::array<uint8_t, 30>;

std::atomic<bool> running{false};

enum class OutputType {SYM, BIN, RRC};

OutputType outputType = OutputType::RRC;
bool invert = false;
bool enc_key = false;
int8_t can = 10;

std::random_device dev;
std::mt19937 rng(dev());
std::uniform_int_distribution<uint32_t> uint_dist;



void signal_handler(int)
{
    running = false;
    std::cerr << "quitting" << std::endl;
}


int8_t bits_to_symbol(uint8_t bits)
{
    switch (bits)
    {
    case 0: return 1;
    case 1: return 3;
    case 2: return -1;
    case 3: return -3;
    }
    abort();
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

template <typename T, size_t N>
std::array<int8_t, N * 4> bytes_to_symbols(const std::array<T, N>& bytes)
{
    std::array<int8_t, N * 4> result;
    size_t index = 0;
    for (auto b : bytes)
    {
        for (size_t i = 0; i != 4; ++i)
        {
            result[index++] = bits_to_symbol(b >> 6);
            b <<= 2;
        }
    }
    return result;
}

template <size_t N>
std::array<int16_t, N*10> symbols_to_baseband(std::array<int8_t, N> symbols)
{
    using namespace mobilinkd;

    static BaseFirFilter<double, std::tuple_size<decltype(rrc_taps)>::value> rrc = makeFirFilter(rrc_taps);

    std::array<int16_t, N*10> baseband;
    baseband.fill(0);
    for (size_t i = 0; i != symbols.size(); ++i)
    {
        baseband[i * 10] = symbols[i];
    }

    for (auto& b : baseband)
    {
        b = rrc(b) * 7168.0 * (invert ? -1.0 : 1.0);
    }

    return baseband;
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
using queue_t = mobilinkd::queue<int16_t, 320>;
using audio_frame_t = std::array<int16_t, 320>;
using codec_frame_t = std::array<uint8_t, 16>;
using data_frame_t = std::array<int8_t, 272>;

std::shared_ptr<queue_t>squeue;
std::shared_ptr<queue_t>basebandQueue;

bool doBasebandCout = true;


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
        if(!doBasebandCout && basebandQueue->is_open()){
            if(!basebandQueue->put(b, std::chrono::milliseconds(400))){
                std::cerr<<"output_baseband(): ERROR OUTPUT QUEUE\n";
            }
        }
        else{
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
                    if(!doBasebandCout && basebandQueue->is_open()){
                        if(!basebandQueue->put(b, std::chrono::milliseconds(400))){
                            std::cerr<<"send_preamble(): ERROR OUTPUT QUEUE\n";
                        }
                    }
                    else{
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
                    if(!doBasebandCout && basebandQueue->is_open()){
                        if(!basebandQueue->put(b, std::chrono::milliseconds(400))){
                            std::cerr<<"send_eot(): ERROR OUTPUT QUEUE\n";
                        }
                    }
                    else{
                        std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
                    }
                }
				
				std::array<int8_t, 192> flush_symbols;
				flush_symbols.fill(0);
				auto f_baseband = symbols_to_baseband(flush_symbols);
                for (auto b : f_baseband) {
                    if(!doBasebandCout && basebandQueue->is_open()){
                        if(!basebandQueue->put(b, std::chrono::milliseconds(400))){
                            std::cerr<<"send_eot(): ERROR OUTPUT QUEUE\n";
                        }
                    }
                    else{
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
		//set enc bits 10 - AES
		result[13] |= (1<<4);
		
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
        audio[index++] = sample;
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
        std::cout << "Stream overflow detected!" << std::endl;
    
    for(unsigned int i=0; i<nBufferFrames; i++){
        uint16_t sample = ((int16_t*)inputBuffer)[i];
        if(!squeue->put(sample, std::chrono::seconds(300))){
            std::cerr<<"record(): ERROR INPUT QUEUE";
        }
    }
    return 0;
}

//int last = 0;

int playback( void *outputBuffer, void *inputBuffer, unsigned int nBufferFrames, double streamTime, RtAudioStreamStatus status, void *userData )
{
    if ( status )
        std::cout << "Stream overflow detected!" << std::endl;

    std::vector<int16_t> output(nBufferFrames);

    for(unsigned int i=0; i<nBufferFrames; i++){
        int16_t sample; 
        if(basebandQueue->is_closed()){
            output[i] = 0;
        }else if(!basebandQueue->get(sample, std::chrono::milliseconds(300))){
            break;
        }
        output[i] = sample;
    }

    memcpy(outputBuffer,(void*)(&output[0]),nBufferFrames*sizeof(int16_t));

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
		ImGui::DockBuilderDockWindow("Audio", dock_id_bottom);
        ImGui::DockBuilderDockWindow("Rig Control", dock_id_bottom);
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


int main(int argc, char* argv[])
{
    using namespace mobilinkd;
	
#ifdef WIN32
	// Set "stdin" to have binary mode:
   auto result = _setmode( _fileno( stdin ), _O_BINARY );
   if( result == -1 )
      perror( "Cannot set mode" );
   else
      std::cerr << "'stdin' successfully changed to binary mode\n";
  
  // Set "stdout" to have binary mode:
  result = _setmode( _fileno( stdout ), _O_BINARY );
   if( result == -1 )
      perror( "Cannot set mode" );
   else
      std::cerr << "'stdout' successfully changed to binary mode\n";
#endif

    auto config = new Config;

    if (config->sym) {
        outputType = OutputType::SYM;
    }
    else if (config->bin) {
        outputType = OutputType::BIN;
    }
    else {
        outputType = OutputType::RRC;
    }

    invert = config->invert;
    can = config->can;
	enc_key = config->encrypt;
	
	has_ptt = config->action_ptt;
	ptt_on = config->tx_on_cmd;
	ptt_off = config->tx_off_cmd;
	
	std::string hash = boost::algorithm::unhex(config->CKEY);
	std::copy(hash.begin(), hash.end(), Key);

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
    GLFWwindow* window = glfwCreateWindow(300, 500, "Dear ImGui GLFW+OpenGL3 example", NULL, NULL);
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

    bool tx = false;
    static char str1[7] = "";
    static char str2[7] = "ALL";
    static char buf[128] = "";

    std::string ptt[2] = {"OFF","ON"};

    RtAudio adc;
    RtAudio dac;
    std::vector< unsigned int > ids = adc.getDeviceIds();
    if ( ids.size() == 0 ) {
      std::cout << "No devices found." << std::endl;
      return 0;
    }

    std::vector<std::string> in_device_names;
    std::vector<std::string> out_device_names;

    std::vector<int> in_device_ids;
    std::vector<int> out_device_ids;

    int in_dev_id=0;
    int out_dev_id=0;
    
    out_device_names.push_back("STD::COUT");

    RtAudio::DeviceInfo info;
    for ( unsigned int n=0; n<ids.size(); n++ ) {
        info = adc.getDeviceInfo( ids[n] );
        if(info.inputChannels>0){
            in_device_names.push_back(std::string(info.name));
            in_device_ids.push_back(ids[n]);
            if(info.isDefaultInput)
                in_dev_id = in_device_ids.size()-1;
        }
    }

    for ( unsigned int n=0; n<ids.size(); n++ ) {
        info = dac.getDeviceInfo( ids[n] );
        if(info.outputChannels>0){
            out_device_names.push_back(std::string(info.name));
            out_device_ids.push_back(ids[n]);
            if(info.isDefaultInput)
                out_dev_id = out_device_ids.size()-1;
        }
    }

    RtAudio::StreamParameters in_parameters;
    in_parameters.deviceId = in_device_ids[in_dev_id];
    in_parameters.nChannels = 1;
    in_parameters.firstChannel = 0;
    unsigned int in_sampleRate = 8000;
    unsigned int in_bufferFrames = 320; // 320 sample frames

    RtAudio::StreamParameters out_parameters;
    out_parameters.deviceId = out_device_ids[out_dev_id];
    out_parameters.nChannels = 1;
    out_parameters.firstChannel = 0;
    unsigned int out_sampleRate = 48000;
    unsigned int out_bufferFrames = 320; // 320 sample frames

    adc.openStream( NULL, &in_parameters, RTAUDIO_SINT16, in_sampleRate, &in_bufferFrames, &record );

    dac.openStream( &out_parameters,  NULL, RTAUDIO_SINT16, out_sampleRate, &out_bufferFrames, &playback );

    std::thread thd;

    serialib serial;
    std::vector<std::string> Serialports = get_available_ports(serial);
    int port_id = 0;

    std::vector<int> Serialbaud = {115200};
    int baud_id = 0;

    std::vector<std::string> Serialptt = {"DTR","RTS"};
    int ptt_id = 0;

    bool rig_enabled = 0;

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
            ImGui::InputTextWithHint("SRC", "SRC CALLSIGN", str1, 7, ImGuiInputTextFlags_CharsUppercase);
            ImGui::InputTextWithHint("DST", "DST CALLSIGN", str2, 7, ImGuiInputTextFlags_CharsUppercase);
            ImGui::InputInt("CAN", &config->can);
	    config->can = std::min<int>(15,std::max<int>(0,config->can));
            ImGui::Checkbox("Invert", &config->invert);
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

            if (btn){
                config->source_address = std::string(str1);
                config->destination_address = std::string(str2);
                if(config->source_address.length()>4){
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

                        adc.startStream();
                        if(!doBasebandCout){
                            dac.startStream();
                        }

                        if(has_ptt){
                            std::cerr << "\r\nPTT: ON \n";          
                            system(ptt_on.c_str());
                        }

                        if(rig_enabled){
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
                    
                        auto lsf = send_lsf(config->source_address, config->destination_address);

                        running = true;

                        thd = std::thread(transmit, std::ref(squeue), std::ref(lsf));
                    
                        std::cerr << "m17-mod running. ctrl-D to break." << std::endl;

                        tx = true;

                    }else{
                        running = false;
                        
                        adc.stopStream();
                        squeue.get()->close();
                        
                        thd.join();

                        basebandQueue.get()->close();
                        if(!doBasebandCout){
                            dac.stopStream();
                        }

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
                        tx = false;
                    }
                }
            }
            ImGui::PopStyleColor(2);
            ImGui::PopStyleVar(1);

            ImGui::Text("PTT = %s", ptt[tx]);
            ImGui::End();

        }
        {
            ImGui::Begin("Audio");
            ImGui::Text("Input Device:");
            float menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("input_audio",in_device_names[in_dev_id].c_str())) {
                for (int i = 0; i < in_device_names.size(); ++i) {
                    const bool isSelected = (in_dev_id == i);
                    if (ImGui::Selectable(in_device_names[i].c_str(), isSelected)) {
                        in_dev_id = i;
                        if(running){
                            tx = false;
                            running = false;

                            adc.stopStream();
                            squeue.get()->close();
                            
                            thd.join();

                            basebandQueue.get()->close();
                            if(!doBasebandCout){
                                dac.stopStream();
                            }
                        }
                        adc.closeStream();
                        in_parameters.deviceId = in_device_ids[in_dev_id];
                        adc.openStream( NULL, &in_parameters, RTAUDIO_SINT16, in_sampleRate, &in_bufferFrames, &record );
                    }

                    // Set the initial focus when opening the combo
                    // (scrolling + keyboard navigation focus)
                    if (isSelected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }
                ImGui::EndCombo();
            }
            ImGui::Text("Output Device:");
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("output_audio",out_device_names[out_dev_id].c_str())) {
                for (int i = 0; i < out_device_names.size(); ++i) {
                    const bool isSelected = (out_dev_id == i);
                    if (ImGui::Selectable(out_device_names[i].c_str(), isSelected)) {
                        out_dev_id = i;
                        
                        if(running){
                            tx = false;
                            running = false;
                            
                            adc.stopStream();
                            squeue.get()->close();
                            
                            thd.join();

                            basebandQueue.get()->close();
                            if(!doBasebandCout){
                                dac.stopStream();
                            }
                        }
                        if(i>0){
                            doBasebandCout = false;
                        }else{
                            doBasebandCout = true;
                        }
                        if(!doBasebandCout){
                            dac.closeStream();
                            out_parameters.deviceId = out_device_ids[out_dev_id-1];
                            dac.openStream( &out_parameters,  NULL, RTAUDIO_SINT16, out_sampleRate, &out_bufferFrames, &playback );
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
            ImGui::End();
        }

        {
            ImGui::Begin("Rig Control");
            ImGui::Checkbox("Enable", &rig_enabled);
            ImGui::Text("Serial Port:");
            float menuWidth = ImGui::GetContentRegionAvail().x;
            ImGui::SetNextItemWidth(menuWidth);
            if (ImGui::BeginCombo("port_menu",Serialports[port_id].c_str())) {
                for (int i = 0; i < Serialports.size(); ++i) {
                    const bool isSelected = (port_id == i);
                    if (ImGui::Selectable(Serialports[i].c_str(), isSelected)) {
                        port_id = i;

                        if(running){
                            tx = false;
                            running = false;
                            
                            adc.stopStream();
                            squeue.get()->close();
                            
                            thd.join();

                            basebandQueue.get()->close();
                            if(!doBasebandCout){
                                dac.stopStream();
                            }

                            if(rig_enabled){
                                serial.closeDevice();
                            }
                        }

                        if(rig_enabled){
                            serial.closeDevice();
                            std::cerr << "Trying to open: " << Serialports[port_id] << "\n";
                            char errorOpening = serial.openDevice(Serialports[port_id].c_str(), 115200);
                            // If connection fails, return the error code otherwise, display a success message
                            if (errorOpening!=1) std::cerr << "Error: " << errorOpening << "\n";
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
        adc.stopStream();
        squeue.get()->close();
        thd.join();
    }

    adc.closeStream();

    return EXIT_SUCCESS;
}
