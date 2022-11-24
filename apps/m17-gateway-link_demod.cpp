// Copyright 2020 Mobilinkd LLC.
// Many parts of this code are from Mobilinkd's m17-mod.cpp app
// Gateway Link made by Paulo Dutra aKa DutraCGI ( calssign PU4THZ )

#include "M17Demodulator.h"
#include "CRC16.h"
#include "ax25_frame.h"
#include "FirFilter.h"

#include <codec2/codec2.h>
#include <boost/crc.hpp>
#include <boost/program_options.hpp>
#include <boost/optional.hpp>

#include "../external/UDPSocket.h"
#include "../external/Log.h"

#include <random>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

#ifdef WIN32
#include <io.h>
#include <fcntl.h>
#endif

const char VERSION[] = "2.2";

bool display_lsf = false;
bool invert_input = false;
bool quiet = false;
bool debug = false;
bool noise_blanker = false;

struct CODEC2 *codec2;

std::vector<uint8_t> current_packet;
size_t packet_frame_counter = 0;
mobilinkd::CRC16<0x1021, 0xFFFF> packet_crc;
mobilinkd::CRC16<0x5935, 0xFFFF> stream_crc;

mobilinkd::PRBS9 prbs;

/*
    M17 Standard IP Frame
*/
using lsf_t = std::array<uint8_t, 28>;
struct M17_IP{
    std::array<char, 4> MAGIC;
    uint16_t SID;
    lsf_t LICH;
    uint16_t FN;
    std::array<uint8_t, 16> Payload;
    uint16_t CRC;
};

M17_IP M17packet;
CUDPSocket* m_socket;
sockaddr_storage m_sockaddr;
unsigned int m_sockaddrLen;
std::array<unsigned char, 64> command;

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

template <typename T, size_t N>
bool dump_lsf(std::array<T, N> const& lsf)
{
    using namespace mobilinkd;
    
    LinkSetupFrame::encoded_call_t encoded_call;

    if (display_lsf)
    {
        std::copy(lsf.begin() + 6, lsf.begin() + 12, encoded_call.begin());
        auto src = LinkSetupFrame::decode_callsign(encoded_call);
        std::cerr << "\nSRC: ";
        for (auto x : src) if (x) std::cerr << x;

        std::copy(lsf.begin(), lsf.begin() + 6, encoded_call.begin());
        auto dest = LinkSetupFrame::decode_callsign(encoded_call);
        std::cerr << ", DEST: ";
        for (auto x : dest) if (x) std::cerr << x;

        uint16_t type = (lsf[12] << 8) | lsf[13];
        dump_type(type);

        std::cerr << ", NONCE: ";
        for (size_t i = 14; i != 28; ++i) std::cerr << std::hex << std::setw(2) << std::setfill('0') << int(lsf[i]);

        uint16_t crc = (lsf[28] << 8) | lsf[29];
        std::cerr << ", CRC: " << std::hex << std::setw(4) << std::setfill('0') << crc;
        std::cerr << std::dec << std::endl;
    }

    current_packet.clear();
    packet_frame_counter = 0;

    if (!lsf[111]) // LSF type bit 0
    {
        uint8_t packet_type = (lsf[109] << 1) | lsf[110];

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
        std::cout.write((const char*)buf.data(), 320);
        std::cout.write((const char*)buf.data(), 320);
    }
    else
    {
        codec2_decode(codec2, buf.data(), audio.data() + 2);
        std::cout.write((const char*)buf.data(), 320);
        codec2_decode(codec2, buf.data(), audio.data() + 10);
        std::cout.write((const char*)buf.data(), 320);
    }

    return result;
}

bool decode_packet(mobilinkd::M17FrameDecoder::packet_buffer_t const& packet_segment)
{
    if (packet_segment[25] & 0x80) // last frame of packet.
    {
        size_t packet_size = (packet_segment[25] & 0x7F) >> 2;
#ifdef WIN32
        packet_size = min(packet_size, size_t(25));
#else
        packet_size = std::min(packet_size, size_t(25));
#endif
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
#ifdef WIN32:
        packet_size = min(packet_size, size_t(25));
#else
        packet_size = std::min(packet_size, size_t(25));
#endif
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

// helper func
uint16_t fromchar(unsigned char c1, unsigned char c2){
    return (((uint16_t)c1) << 8) | c2;
}

 sockaddr_storage sockaddr;
 unsigned int sockaddrLen = 0U;

int sock_start(std::string LocalAddress, unsigned short LocalPort){
    int ret;
    CUDPSocket::startup();

    if (CUDPSocket::lookup(LocalAddress, LocalPort, m_sockaddr, m_sockaddrLen) != 0) {
		LogError("Could not lookup the address");
		::LogFinalise();
		return 1;
	}

	m_socket = new CUDPSocket(LocalAddress, LocalPort);
	ret = m_socket->open();
	if (!ret) {
		LogError("Unable to open the link socket");
		::LogFinalise();
		return 1;
	}

    std::fill(command.begin(),command.end(),'\0');
	
	std::this_thread::sleep_for (std::chrono::seconds(3));
	
	m_socket->read(&command[0], 64U, sockaddr, sockaddrLen);
	
	std::cerr << "Running at " << LocalAddress << ":" << LocalPort << std::endl;

    return ret;
}

bool handle_frame(mobilinkd::M17FrameDecoder::output_buffer_t const& frame, int viterbi_cost)
{
    using FrameType = mobilinkd::M17FrameDecoder::FrameType;

    bool result = true;

    switch (frame.type)
    {
        case FrameType::LSF:{
		    std::copy(frame.lsf.begin(),frame.lsf.begin()+28,M17packet.LICH.begin());
            result = dump_lsf(frame.lsf);
        } break;
        case FrameType::LICH:{
            std::cerr << "LICH" << std::endl;
        } break;
        case FrameType::STREAM:{
            result = true;
            // First two bytes are the frame counter + EOS indicator.
            M17packet.FN = fromchar(frame.stream[1], frame.stream[0]);
            if (viterbi_cost < 70 && (fromchar(frame.stream[0], frame.stream[1]) & 0x8000)){
                M17packet.SID = rand()%UINT16_MAX;
                if (display_lsf) std::cerr << "\nEOS" << std::endl;
                result = false;
            }
            if (viterbi_cost < 80){
                std::copy(frame.stream.begin()+2,frame.stream.begin()+18,M17packet.Payload.begin());
                stream_crc.reset();
                for (size_t i = 0; i != sizeof(M17_IP); ++i){
                    stream_crc(((char*)&M17packet)[i]);
                }
                auto crc_bytes = stream_crc.get_bytes();
                std::copy((char*)&crc_bytes,(char*)&crc_bytes+2,(char*)&M17packet.CRC);
                if(!m_socket->write((uint8_t*)std::string((char*)&M17packet, (char*)&M17packet+sizeof(M17_IP)).c_str(),sizeof(M17_IP), sockaddr, sockaddrLen)){
                    std::cerr << "ERROR SENDING DATA\n";
                }
            }
        } break;
        case FrameType::BASIC_PACKET:{
            result = decode_packet(frame.packet);
        } break;
        case FrameType::FULL_PACKET:{
            result = decode_packet(frame.packet);
        } break;
        case FrameType::BERT:{
            result = decode_bert(frame.bert);
        } break;
    }

    return result;
}

template <typename FloatType>
void diagnostic_callback(bool dcd, FloatType evm, FloatType deviation, FloatType offset, bool locked,
    FloatType clock, int sample_index, int sync_index, int clock_index, int viterbi_cost)
{
    if (debug) {
        std::cerr << "\rdcd: " << std::setw(1) << int(dcd)
            << ", evm: " << std::setfill(' ') << std::setprecision(4) << std::setw(8) << evm * 100 <<"%"
            << ", deviation: " << std::setprecision(4) << std::setw(8) << deviation
            << ", freq offset: " << std::setprecision(4) << std::setw(8) << offset
            << ", locked: " << std::boolalpha << std::setw(6) << locked << std::dec
            << ", clock: " << std::setprecision(7) << std::setw(8) << clock
            << ", sample: " << std::setw(1) << sample_index << ", "  << sync_index << ", " << clock_index
            << ", cost: " << viterbi_cost;
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
        char buffer[40];
        snprintf(buffer, 40, "BER: %-1.6lf (%lu bits)", ber, prbs.bits());
        std::cerr << buffer;
    }
    std::cerr << std::flush;
}

struct Config
{
    bool verbose = false;
    bool debug = false;
    bool quiet = false;
    bool invert = false;
    bool lsf = false;
    bool noise_blanker = false;
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
             ("addr,A", po::value<std::string>(&result.addr)->default_value("127.0.0.1"),
                "Gateway repeater address (default is 127.0.0.1).")
            ("port,P", po::value<unsigned short>(&result.port)->default_value(17011),
                "Gateway repeater port (default is 17011).")
            ("invert,i", po::bool_switch(&result.invert), "invert the received baseband")
            ("noise-blanker,b", po::bool_switch(&result.noise_blanker), "noise blanker -- silence likely corrupt audio")
            ("lsf,l", po::bool_switch(&result.lsf), "display the decoded LSF")
            ("verbose,v", po::bool_switch(&result.verbose), "verbose output")
            ("debug,d", po::bool_switch(&result.debug), "debug-level output")
            ("quiet,q", po::bool_switch(&result.quiet), "silence all output -- no BERT output")
            ;

        po::variables_map vm;
        po::store(po::parse_command_line(argc, argv, desc), vm);

        if (vm.count("help"))
        {
            std::cout << "Read M17 baseband from STDIN and write M17 IP packets to M17-Gateway\n"
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

        if (result.addr.size() > 15)
        {
            std::cerr << "Repeater IP Address too long." << std::endl;
            return std::nullopt;
        }

        if (result.debug + result.verbose + result.quiet > 1)
        {
            std::cerr << "Only one of quiet, verbos or debug may be chosen." << std::endl;
            return std::nullopt;
        }

        return result;
    }
};

int main(int argc, char* argv[])
{
    using namespace mobilinkd;
    using namespace std::string_literals;
	
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

    display_lsf = config->lsf;
    invert_input = config->invert;
    quiet = config->quiet;
    debug = config->debug;
    noise_blanker = config->noise_blanker;

    std::string addr = config->addr;
    unsigned short port = config->port;
    sock_start(addr,port);

    codec2 = ::codec2_create(CODEC2_MODE_3200);

    using FloatType = float;

    uint32_t magic = 0x2037314d;
    std::copy((char*) &magic,(char*) &magic+4,M17packet.MAGIC.data());
    M17packet.SID = rand()%UINT16_MAX;

    M17Demodulator<FloatType> demod(handle_frame);

#if 0
    if (display_diags)
    {
        std::cerr << "Size of M17Demodulator: " << sizeof(demod) << std::endl;
        std::cerr << "    Size of M17FrameDecoder: " << sizeof(M17FrameDecoder) << std::endl;
        std::cerr << "        Size of M17Randomizer<368>: " << sizeof(M17Randomizer<368>) << std::endl;
        std::cerr << "        Size of PolynomialInterleaver<45, 92, 368>: " << sizeof(PolynomialInterleaver<45, 92, 368>) << std::endl;
        std::cerr << "        Size of Trellis<4,2>: " << sizeof(Trellis<4,2>) << std::endl;
        std::cerr << "        Size of Viterbi<Trellis<4,2>, 4>: " << sizeof(Viterbi<Trellis<4,2>, 4>) << std::endl;
        std::cerr << "        Size of output_buffer_t: " << sizeof(M17FrameDecoder::output_buffer_t) << std::endl;
        std::cerr << "        Size of depunctured_buffer_t: " << sizeof(M17FrameDecoder::depunctured_buffer_t) << std::endl;
        std::cerr << "        Size of decode_buffer_t: " << sizeof(M17FrameDecoder::decode_buffer_t) << std::endl;
        std::cerr << "    Size of M17 Matched Filter: " << sizeof(BaseFirFilter<FloatType, detail::Taps<double>::rrc_taps.size()>) << std::endl;
        std::cerr << "    Size of M17 Correlator: " << sizeof(Correlator<FloatType>) << std::endl;
        std::cerr << "    Size of M17 SyncWord: " << sizeof(SyncWord<Correlator<FloatType>>) << std::endl;
        std::cerr << "    Size of M17 DataCarrierDetect: " << sizeof(DataCarrierDetect<FloatType, 48000, 500>) << std::endl;
        std::cerr << "    Size of M17 ClockRecovery: " << sizeof(ClockRecovery<FloatType, 48000, 4800>) << std::endl;
        std::cerr << "    Size of M17 M17Framer: " << sizeof(M17Framer<368>) << std::endl;
    }
#endif

    demod.diagnostics(diagnostic_callback<FloatType>);

    // std::ofstream out("stream.out");
    // auto old_rdbuf = std::clog.rdbuf();
    // std::clog.rdbuf(out.rdbuf());

    while (std::cin)
    {
        int16_t sample;
        std::cin.read(reinterpret_cast<char*>(&sample), 2);
        if (invert_input) sample *= -1;
        demod(sample / 44000.0);
    }

    std::cerr << std::endl;

    codec2_destroy(codec2);

    // std::clog.rdbuf(old_rdbuf);

    return EXIT_SUCCESS;
}
