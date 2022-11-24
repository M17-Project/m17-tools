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
			("transmit-off,S", po::value<std::string>(&result.tx_off_cmd)->default_value(""),
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


// rrc
void output_baseband(std::array<uint8_t, 2> sync_word, const bitstream_t& frame)
{
    auto symbols = bits_to_symbols(frame);
    auto sw = bytes_to_symbols(sync_word);

    std::array<int8_t, 192> temp;
    auto fit = std::copy(sw.begin(), sw.end(), temp.begin());
    std::copy(symbols.begin(), symbols.end(), fit);
    auto baseband = symbols_to_baseband(temp);
    for (auto b : baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
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
                for (auto b : preamble_baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
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
                for (auto b : baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
				
				std::array<int8_t, 192> flush_symbols;
				flush_symbols.fill(0);
				auto f_baseband = symbols_to_baseband(flush_symbols);
                for (auto b : f_baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
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

using lich_segment_t = std::array<uint8_t, 96>;
using lich_t = std::array<lich_segment_t, 6>;
using queue_t = mobilinkd::queue<int16_t, 320>;
using audio_frame_t = std::array<int16_t, 320>;
using codec_frame_t = std::array<uint8_t, 16>;
using data_frame_t = std::array<int8_t, 272>;

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

void transmit(queue_t& queue, const lsf_t& lsf)
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
    while(!queue.is_closed() && queue.empty()) std::this_thread::yield();
    while (!queue.is_closed())
    {
        int16_t sample;
        if (!queue.get(sample, std::chrono::milliseconds(3000))) break;
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

#define USE_OLD_MODULATOR
#ifdef USE_OLD_MODULATOR


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

    auto config = Config::parse(argc, argv);
    if (!config) return 0;

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
	
    signal(SIGINT, &signal_handler);
	
	if(has_ptt){
        std::cerr << "\r\nPTT: ON \n";			
		system(ptt_on.c_str());
	}

    send_preamble();

    if (!config->bert) {
        auto lsf = send_lsf(config->source_address, config->destination_address);

        running = true;
        queue_t queue;
        std::thread thd([&queue, &lsf](){transmit(queue, lsf);});

        std::cerr << "m17-mod running. ctrl-D to break." << std::endl;

        // Input must be 8000 SPS, 16-bit LE, 1 channel raw audio.
        while (running)
        {
            int16_t sample;
            if (!std::cin.read(reinterpret_cast<char*>(&sample), 2)) break;
            if (!queue.put(sample, std::chrono::seconds(300))) break;
        }

        running = false;

        queue.close();
        thd.join();
    } else {
        PRBS9 prbs;

        send_preamble();
        running = true;
        M17Randomizer<368> randomizer;
        PolynomialInterleaver<45, 92, 368> interleaver;

        while (running) {
            auto frame = make_bert_frame(prbs);
            interleaver.interleave(frame);
            randomizer.randomize(frame);
            output_frame(BERT_SYNC_WORD, frame);    
        }
    }


    return EXIT_SUCCESS;
}

#else // use new modulator

int main(int argc, char* argv[])
{
    using namespace mobilinkd;
    using namespace std::chrono_literals;
	
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

    auto config = Config::parse(argc, argv);
    if (!config) return 0;

    signal(SIGINT, &signal_handler);

    auto audio_queue = std::make_shared<M17Modulator::audio_queue_t>();
    auto bitstream_queue = std::make_shared<M17Modulator::bitstream_queue_t>();
    
    M17Modulator modulator(config->source_address, config->destination_address);
    auto future = modulator.run(audio_queue, bitstream_queue);
    modulator.ptt_on();

    std::cerr << "m17-mod running. ctrl-D to break." << std::endl;

    M17Modulator::bitstream_t bitstream;
    uint8_t bits;
    size_t index = 0;

    std::thread thd([audio_queue](){
        int16_t sample = 0;
        running = true;
        while (running && std::cin)
        {
            std::cin.read(reinterpret_cast<char*>(&sample), 2);
            audio_queue->put(sample, 5s);
        }
        running = false;
    });

    while (!running) std::this_thread::yield();

    // Input must be 8000 SPS, 16-bit LE, 1 channel raw audio.
    while (running)
    {
        if (!(bitstream_queue->get(bits, 1s)))
        {
            assert(bitstream_queue->is_closed());
            std::clog << "bitstream queue is closed; done transmitting." << std::endl;
            running = false;
            break;
        }

        if (config->bitstream)
        {
            std::cout << bits;
            index += 1;
            if (index == bitstream.size())
            {
                index == 0;
                std::cout.flush();
            }
        }
        else
        {
            bitstream[index++] = bits;
            if (index == bitstream.size())
            {
                auto baseband = M17Modulator::symbols_to_baseband(M17Modulator::bytes_to_symbols(bitstream));
                for (auto b : baseband) std::cout << uint8_t((b & 0xFF00) >> 8) << uint8_t(b & 0xFF);
                std::cout.flush();
            }
        }
    }

    std::clog << "No longer running" << std::endl;

    running = false;
    modulator.ptt_off();
    modulator.wait_until_idle();
    thd.join();
    audio_queue->close();
    future.get();
    bitstream_queue->close();

    return EXIT_SUCCESS;
}
#endif
