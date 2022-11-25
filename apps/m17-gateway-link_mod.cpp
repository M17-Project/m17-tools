// Copyright 2020 Mobilinkd LLC.
// Many parts of this code are from Mobilinkd's m17-mod.cpp app
// Gateway Link made by Paulo Dutra aKa DutraCGI ( calssign PU4THZ )

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

#include "../external/UDPSocket.h"
#include "../external/Log.h"

#include <thread>

#include <array>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include <atomic>
#include <optional>
#include <mutex>
#include <chrono>

#include <cstdlib>

#ifdef WIN32
#include <io.h>
#include <fcntl.h>
#endif

#include <signal.h>

// Generated using scikit-commpy
const auto rrc_taps = std::array<double, 150>{
    0.0029364388513841593, 0.0031468394550958484, 0.002699564567597445, 0.001661182944400927, 0.00023319405581230247, -0.0012851320781224025, -0.0025577136087664687, -0.0032843366522956313, -0.0032697038088887226, -0.0024733964729590865, -0.0010285696910973807, 0.0007766690889758685, 0.002553421969211845, 0.0038920145144327816, 0.004451886520053017, 0.00404219185231544, 0.002674727068399207, 0.0005756567993179152, -0.0018493784971116507, -0.004092346891623224, -0.005648131453822014, -0.006126925416243605, -0.005349511529163396, -0.003403189203405097, -0.0006430502751187517, 0.002365929161655135, 0.004957956568090113, 0.006506845894531803, 0.006569574194782443, 0.0050017573119839134, 0.002017321931508163, -0.0018256054303579805, -0.00571615173291049, -0.008746639552588416, -0.010105075751866371, -0.009265784007800534, -0.006136551625729697, -0.001125978562075172, 0.004891777252042491, 0.01071805138282269, 0.01505751553351295, 0.01679337935001369, 0.015256245142156299, 0.01042830577908502, 0.003031522725559901, -0.0055333532968188165, -0.013403099825723372, -0.018598682349642525, -0.01944761739590459, -0.015005271935951746, -0.0053887880354343935, 0.008056525910253532, 0.022816244158307273, 0.035513467692208076, 0.04244131815783876, 0.04025481153629372, 0.02671818654865632, 0.0013810216516704976, -0.03394615682795165, -0.07502635967975885, -0.11540977897637611, -0.14703962203941534, -0.16119995609538576, -0.14969512896336504, -0.10610329539459686, -0.026921412469634916, 0.08757875030779196, 0.23293327870303457, 0.4006012210123992, 0.5786324696325503, 0.7528286479934068, 0.908262741447522, 1.0309661131633199, 1.1095611856548013, 1.1366197723675815, 1.1095611856548013, 1.0309661131633199, 0.908262741447522, 0.7528286479934068, 0.5786324696325503, 0.4006012210123992, 0.23293327870303457, 0.08757875030779196, -0.026921412469634916, -0.10610329539459686, -0.14969512896336504, -0.16119995609538576, -0.14703962203941534, -0.11540977897637611, -0.07502635967975885, -0.03394615682795165, 0.0013810216516704976, 0.02671818654865632, 0.04025481153629372, 0.04244131815783876, 0.035513467692208076, 0.022816244158307273, 0.008056525910253532, -0.0053887880354343935, -0.015005271935951746, -0.01944761739590459, -0.018598682349642525, -0.013403099825723372, -0.0055333532968188165, 0.003031522725559901, 0.01042830577908502, 0.015256245142156299, 0.01679337935001369, 0.01505751553351295, 0.01071805138282269, 0.004891777252042491, -0.001125978562075172, -0.006136551625729697, -0.009265784007800534, -0.010105075751866371, -0.008746639552588416, -0.00571615173291049, -0.0018256054303579805, 0.002017321931508163, 0.0050017573119839134, 0.006569574194782443, 0.006506845894531803, 0.004957956568090113, 0.002365929161655135, -0.0006430502751187517, -0.003403189203405097, -0.005349511529163396, -0.006126925416243605, -0.005648131453822014, -0.004092346891623224, -0.0018493784971116507, 0.0005756567993179152, 0.002674727068399207, 0.00404219185231544, 0.004451886520053017, 0.0038920145144327816, 0.002553421969211845, 0.0007766690889758685, -0.0010285696910973807, -0.0024733964729590865, -0.0032697038088887226, -0.0032843366522956313, -0.0025577136087664687, -0.0012851320781224025, 0.00023319405581230247, 0.001661182944400927, 0.002699564567597445, 0.0031468394550958484, 0.0029364388513841593, 0.0
};

const auto evm_b = std::array<double, 3>{0.02008337, 0.04016673, 0.02008337};
const auto evm_a = std::array<double, 3>{1.0, -1.56101808, 0.64135154};

const char VERSION[] = "1.0";

std::string ptt_on = "";
std::string ptt_off = "";
bool has_ptt = false;

struct Config
{
    std::string audio_device;
    std::string event_device;
	std::string tx_on_cmd;
	std::string tx_off_cmd;
	bool action_ptt = false;
    uint16_t key;
    bool verbose = false;
    bool debug = false;
    bool quiet = false;
    bool bitstream = false; // default is baseband audio
    bool bert = false; // Bit error rate testing.
    bool invert = false;
    int can = 10;
    std::string addr;
    unsigned short port;

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

            ("addr,A", po::value<std::string>(&result.audio_device)->default_value("127.0.0.1"),
                "Gateway repeater address (default is 127.0.0.1).")
            ("port,P", po::value<unsigned short>(&result.port)->default_value(17011),
                "Gateway repeater port (default is 17011).")
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
            ("key,k", po::value<uint16_t>(&result.key)->default_value(385),
                "Linux event code for PTT (default is RADIO).")
            ("bitstream,b", po::bool_switch(&result.bitstream),
                "output bitstream (default is baseband).")
            ("bert,B", po::bool_switch(&result.bert),
                "output a bit error rate test stream (default is read audio from STDIN).")
            ("invert,i", po::bool_switch(&result.invert), "invert the output baseband (ignored for bitstream)")
            ("verbose,v", po::bool_switch(&result.verbose), "verbose output")
            ("debug,d", po::bool_switch(&result.debug), "debug-level output")
            ("quiet,q", po::bool_switch(&result.quiet), "silence all output")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "Read IP packets from M17-Gateway and write M17 baseband to STDOUT\n"
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
            std::cerr << "Only one of quiet, verbos or debug may be chosen." << std::endl;
            return std::nullopt;
        }

        if (result.addr.size() > 15)
        {
            std::cerr << "Repeater IP Address too long." << std::endl;
            return std::nullopt;
        }
		
		if (result.tx_on_cmd.size() > 0 && result.tx_off_cmd.size() > 0)
        {
			result.action_ptt = true;
            std::cerr << "PTT action provided." << std::endl;
        }

        if (result.can < 0 || result.can > 15) {
            std::cerr << "invalid channel access number (CAN) " << result.can << ". Must be 0-15." << std::endl;
            return std::nullopt;
        }

        return result;
    }
};

enum class FrameType {AUDIO, DATA, MIXED, BERT};

using lsf_t = std::array<uint8_t, 30>;

std::atomic<bool> running{false};

bool bitstream = false;
bool invert = false;
bool mdebug = false;
int8_t can = 10;

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
    if (bitstream) output_bitstream(sync_word, frame);
    else output_baseband(sync_word, frame);
}

void send_preamble()
{
    // Preamble is simple... bytes -> symbols -> baseband.
    std::cerr << "Sending preamble." << std::endl;
    std::array<uint8_t, 48> preamble_bytes;
    preamble_bytes.fill(0x77);
    if (bitstream)
    {
        for (auto c : preamble_bytes) std::cout << c;
    }
    else // baseband
    {
        auto preamble_symbols = bytes_to_symbols(preamble_bytes);
        auto preamble_baseband = symbols_to_baseband(preamble_symbols);
        for (auto b : preamble_baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
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
    if (bitstream)
    {
        for (auto c : EOT_SYNC) std::cout << c;
        for (size_t i = 0; i !=10; ++i) std::cout << '\0'; // Flush RRC FIR Filter.
    }
    else
    {
        std::array<int8_t, 48> out_symbols; // EOT symbols + FIR flush.
        out_symbols.fill(0);
        auto symbols = bytes_to_symbols(EOT_SYNC);
        for (size_t i = 0; i != symbols.size(); ++i)
        {
            out_symbols[i] = symbols[i];
        }
        auto baseband = symbols_to_baseband(out_symbols);
        for (auto b : baseband) std::cout << uint8_t(b & 0xFF) << uint8_t(b >> 8);
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
using codec_frame_t = std::array<uint8_t, 16>;
using queue_t = mobilinkd::queue<codec_frame_t, 10>;
using data_frame_t = std::array<int8_t, 272>;

data_frame_t make_data_frame(uint16_t frame_number, const codec_frame_t& payload)
{
    std::array<uint8_t, 18> data;   // FN, Audio = 2 + 16;
    data[0] = uint8_t((frame_number >> 8) & 0xFF);
    data[1] = uint8_t(frame_number & 0xFF);
    std::copy(payload.begin(), payload.end(), data.begin() + 2);

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

/*
    M17 Standard IP Frame
*/
struct M17_IP{
    std::array<char, 4> MAGIC;
    uint16_t SID;
    lsf_t LICH;
    uint16_t FN;
    std::array<uint8_t, 16> Payload;
    uint16_t CRC;
};

/*
    Main M17RTXLink Class
*/
class M17RTXLink
{
public:
    // Default Address 127.0.0.1 Port 12711
    M17RTXLink(std::string LocalAddress, unsigned short LocalPort);
    void run();
    int start();

private:
    void transmit();

    unsigned short port;
    std::string addr;

    CUDPSocket*      m_socket;

    sockaddr_storage m_sockaddr;
    unsigned int m_sockaddrLen;

    M17_IP packet;

    queue_t qPayload;
    lsf_t lsf;
    lich_t lich;
    uint8_t lich_segment;
    std::array<unsigned char, 64> command;
    bool timeout;
    std::chrono::steady_clock::time_point begin;
    std::chrono::steady_clock::time_point now;
    bool pttOn = false;
};

// setup
M17RTXLink::M17RTXLink(std::string LocalAddress, unsigned short LocalPort){
    addr = LocalAddress;
    port = LocalPort;
}

// helper func
uint16_t fromchar(unsigned char c1, unsigned char c2){
    return (((uint16_t)c1) << 8) | c2;
}

void M17RTXLink::run(){
    sockaddr_storage sockaddr;
    unsigned int sockaddrLen = 0U;

	running = true;
    std::thread thd(&M17RTXLink::transmit,this);
	
	std::cerr << "M17RTXLink running. ctrl-D to break." << std::endl;

    // Start LICH Construction
    lich_segment = 0;
    for (size_t i = 0; i != lich.size(); ++i)
    {
        std::array<uint8_t, 5> segment;
        std::copy(lsf.begin() + i * 5, lsf.begin() + (i + 1) * 5, segment.begin());
        auto lich_segment = make_lich_segment(segment, i);
        std::copy(lich_segment.begin(), lich_segment.end(), lich[i].begin());
    }

    // Main Loop
    while(1){
        std::fill(command.begin(),command.end(),'\0');
        // Read M17 IP Packets from M17 Gateway
        int ret = m_socket->read(&command[0], 64U, sockaddr, sockaddrLen);
		if (ret > 0) {

            // Get MAGIC 
            std::copy(command.begin(),command.begin()+4,packet.MAGIC.begin());
            int magic = (uint32_t)packet.MAGIC[0] << 24 | (uint32_t)packet.MAGIC[1] << 16 | (uint32_t)packet.MAGIC[2] << 8  | (uint32_t)packet.MAGIC[3];

            if(mdebug){
			    std::cerr << "[DEBUG] MAGIC: " << std::string(packet.MAGIC.begin(),packet.MAGIC.end()) << "\n";
            }
            
            // Check Magic "M17 "
            if(magic == 0x4d313720){

                packet.FN =  fromchar(command[34],command[35]); // FRAME NUMBER

                std::copy(command.begin()+36,command.begin()+36+16,packet.Payload.begin()); // CODEC2 PAYLOAD
				
                if((packet.FN & 0x8000)){
					if(has_ptt){
                        std::cerr << "\r\nPTT: OFF \n";	
						system(ptt_off.c_str());
						pttOn = false;
					}
                }

                if(packet.FN == 0x00){

                    packet.SID = fromchar(command[4],command[5]);   // SID (not used...)
                    
                    // Make LSF & Swap DST->SRC
                    std::copy(command.begin()+6,command.begin()+6+28,packet.LICH.begin());
                    for(int8_t i=0; i<6; i++){
                        std::swap(packet.LICH[i],packet.LICH[6+i]);
                    }
                    
                    // Decode SRC Callsign
                    mobilinkd::LinkSetupFrame::encoded_call_t encoded_src = {0x00,0x00,0x00,0x00,0x00,0x00};
                    std::copy(packet.LICH.begin(), packet.LICH.begin()+6, encoded_src.begin());
                    auto src = mobilinkd::LinkSetupFrame::decode_callsign(encoded_src);

                    // Encode SRC Callsign
                    mobilinkd::LinkSetupFrame::encoded_call_t encoded_dst {0x00,0x00,0x00,0x00,0x00,0x00};
                    std::copy(packet.LICH.begin()+6, packet.LICH.begin()+12, encoded_dst.begin());
                    auto dst = mobilinkd::LinkSetupFrame::decode_callsign(encoded_dst);
                
                    if(mdebug){
                        std::cerr << "[DEBUG] FN: " << packet.FN << " SRC: " << std::string(src.begin(), src.end()) << " DST: " << std::string(dst.begin(), dst.end()) << "\n";
                    }
					
					if(has_ptt){
                        std::cerr << "\r\nPTT: ON \n";			
						system(ptt_on.c_str());
						pttOn = true;
					}
                    
                    send_preamble();
                    lsf = send_lsf(std::string(src.begin(), src.end()),std::string(dst.begin(), dst.end()));

                    lich_segment = 0;
                    for (size_t i = 0; i != lich.size(); ++i)
                    {
                        std::array<uint8_t, 5> segment;
                        std::copy(lsf.begin() + i * 5, lsf.begin() + (i + 1) * 5, segment.begin());
                        auto lich_segment = make_lich_segment(segment, i);
                        std::copy(lich_segment.begin(), lich_segment.end(), lich[i].begin());
                    }
                }
                begin = std::chrono::steady_clock::now();
                codec_frame_t payload;
                std::copy(packet.Payload.begin(),packet.Payload.end(),payload.begin());
                if (!qPayload.put(payload, std::chrono::seconds(300))){ std::cerr << "QUEUE FULL BREAK\n"; break;} 
            }
        }else{
            if(pttOn && has_ptt){
                now = std::chrono::steady_clock::now();
                auto time = std::chrono::duration_cast<std::chrono::milliseconds>(now - begin).count();
                if(time>1000){
                    std::cerr << "\r\n[timeout] PTT: OFF \n";			
                    system(ptt_off.c_str());
                    pttOn = false;
                    begin = std::chrono::steady_clock::now();
                    std::cerr << "Elapsed(ms)=" << time << std::endl;
                }
            }
        }
    }

    thd.join();
    qPayload.close();
}

int M17RTXLink::start(){
    int ret;
    CUDPSocket::startup();

    if (CUDPSocket::lookup(addr, port, m_sockaddr, m_sockaddrLen) != 0) {
		LogError("Could not lookup the address");
		::LogFinalise();
		return 1;
	}

	m_socket = new CUDPSocket(addr, port);
	ret = m_socket->open();
	if (!ret) {
		LogError("Unable to open the link socket");
		::LogFinalise();
		return 1;
	}

    std::fill(command.begin(),command.end(),'\0');

    return ret;
}

void M17RTXLink::transmit()
{
    using namespace mobilinkd;

    assert(running);

    uint16_t frame_number = 0;

    codec_frame_t payload;

    while(!qPayload.is_closed() && qPayload.empty()) std::this_thread::yield();
    while (!qPayload.is_closed())
    {
        if(pttOn){
            qPayload.get(payload, std::chrono::milliseconds(300));
            auto data = make_data_frame(frame_number++, payload);
            if (frame_number == 0x8000){
                frame_number = 0;
                // Last frame
                data = make_data_frame(frame_number | 0x8000, payload);
                send_audio_frame(lich[lich_segment], data);
                output_eot();
            } 
            send_audio_frame(lich[lich_segment++], data);
            if (lich_segment == lich.size()) lich_segment = 0;
        }
    }

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

    auto config = Config::parse(argc, argv);
    if (!config) return 0;

    bitstream = config->bitstream;
    invert = config->invert;
    can = config->can;
    mdebug = config ->debug;

    std::string addr = config->addr;
    unsigned short port = config->port;
	
	has_ptt = config->action_ptt;
	ptt_on = config->tx_on_cmd;
	ptt_off = config->tx_off_cmd;

    int res;
    M17RTXLink glink(std::string("127.0.0.1"),(unsigned short)17011);
    res = glink.start();
    glink.run();
    return res;
}
