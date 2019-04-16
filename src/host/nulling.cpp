#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/types/time_spec.hpp>

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <complex>
#include <cmath>

#include <csignal>
#include <stdio.h>
#include <stdlib.h>
#include "nulling.h"
#include <fftw3.h>

namespace po = boost::program_options;

// System parameters 
double freq, gain, thres;
double inter, rate;

// USRP cmd
uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS);
uhd::time_spec_t time_start_recv;

// USRP
uhd::usrp::multi_usrp::sptr usrp_tx1;
uhd::usrp::multi_usrp::sptr usrp_tx2;
uhd::usrp::multi_usrp::sptr usrp_rx;
string usrp_tx1_ip;
string usrp_tx2_ip;
string usrp_rx_ip;

// TX/RX metadata
uhd::rx_metadata_t rx_md;
uhd::tx_metadata_t tx_md, tx_md2;

// Buffer
gr_complex pkt_tx_time[SYM_LEN], pkt_tx[MAX_PKT_LEN], zeros[SYM_LEN];
gr_complex *pkt_rx, *pkt_rx_tmp;

/* Files
 * in_file: 		Tx freq signals
 * out_file: 		Rx time signal after preambles
 * raw_file: 		all Rx raw signal
 * tx_time_file: 	Tx time signals
 */ 
FILE* in_file;
FILE* out_file;
FILE* raw_file;
FILE* tx_time_file;
string in_name, out_name, raw_name; 

// Evaluation
size_t r_cnt, s_cnt;
double r_sec;
static bool stop_signal = false;


void init_usrp() {
	// David: Configure multiple usrp
	usrp_tx1 = uhd::usrp::multi_usrp::make(usrp_tx1_ip);
	usrp_tx2 = uhd::usrp::multi_usrp::make(usrp_tx2_ip);
	usrp_rx = uhd::usrp::multi_usrp::make(usrp_rx_ip);
	usrp_tx1->set_rx_rate(rate);
	usrp_tx2->set_rx_rate(rate);
	usrp_rx->set_rx_rate(rate);
	usrp_tx1->set_tx_rate(rate);
	usrp_tx2->set_tx_rate(rate);
	usrp_rx->set_tx_rate(rate);

	usrp_tx1->set_rx_freq(freq);
	usrp_tx2->set_rx_freq(freq);
	usrp_rx->set_rx_freq(freq);
	usrp_tx1->set_tx_freq(freq);
	usrp_tx2->set_tx_freq(freq);
	usrp_rx->set_tx_freq(freq);

	usrp_tx1->set_rx_gain(gain);
	usrp_tx2->set_rx_gain(gain);
	usrp_rx->set_rx_gain(gain);
}

void sync_clock() {
	cout << "SYNC Clock" << endl;
	usrp_tx1->set_clock_config(uhd::clock_config_t::external());
	usrp_tx2->set_clock_config(uhd::clock_config_t::external());
	usrp_rx->set_clock_config(uhd::clock_config_t::external());
	usrp_tx1->set_time_next_pps(uhd::time_spec_t(0.0));
	usrp_tx2->set_time_next_pps(uhd::time_spec_t(0.0));
	usrp_rx->set_time_next_pps(uhd::time_spec_t(0.0));
}

void init_stream() {
    boost::this_thread::sleep(boost::posix_time::milliseconds(WARM_UP_TIME));
    stream_cmd.time_spec = time_start_recv  = uhd::time_spec_t(1.0) + usrp_rx->get_time_now();
	cout << "Time to start receiving: " << time_start_recv.get_real_secs() << endl;
    stream_cmd.stream_now = false;
    usrp_rx->issue_stream_cmd(stream_cmd);
}

void init_sys() {
	if (DEBUG) {
		raw_file = fopen("../../out/rx_raw_signals.bin", "wb");
		tx_time_file = fopen("../../out/tx_time_signals.bin", "wb");
	}

	// Tx buffer initialize	
	memset(pkt_tx, 0, sizeof(pkt_tx));
	memset(pkt_tx_time, 0, sizeof(pkt_tx_time));
	
	// Put generated signal in pkt_tx
	FILE* tx_freq_file = fopen(in_name.c_str(), "rb");

	// Read freq_data file to freq_data
	fread(pkt_tx, sizeof(gr_complex), MAX_PKT_LEN, tx_freq_file);
	fclose(tx_freq_file);

	// Rx buffer initialize
	printf("New receiving signal buffer\n");
	s_cnt = (size_t)(1e8 / SYM_LEN * r_sec);
	pkt_rx = new gr_complex[s_cnt];
	pkt_rx_tmp = new gr_complex[s_cnt];

	if (pkt_rx != NULL) {
		memset(pkt_rx, 0, sizeof(gr_complex) * s_cnt);
	}
	if (pkt_rx_tmp != NULL) {
		memset(pkt_rx_tmp, 0, sizeof(gr_complex) * s_cnt);
	}

	// USRP setting
	rate = 1e8 / inter;
	freq = 1e9 * freq;

	init_usrp();

	// David: Use external clock for synchronziation
	sync_clock();

	init_stream();
}

void ifft(gr_complex *input) {
	fftw_complex *in, *out;
	fftw_plan p;

	in = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * SC_LEN);
	out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * SC_LEN);
	for (size_t i = 0; i < SC_LEN; ++i) {
		in[i][0] = input[i].real();
		in[i][1] = input[i].imag(); 
	}
	
	p = fftw_plan_dft_1d(SC_LEN, in, out, FFTW_BACKWARD, FFTW_ESTIMATE);
	fftw_execute(p);
	for (size_t i = 0; i < SC_LEN; ++i) {
		out[i][0] = out[i][0] / SC_LEN;
		out[i][1] = out[i][1] / SC_LEN;
	}

	size_t CP_IX = SC_LEN - CP_LEN;
	for (size_t i = 0; i < CP_LEN; ++i) {
		pkt_tx_time[i].real() = out[CP_IX + i][0];
		pkt_tx_time[i].imag() = out[CP_IX + i][1];
	} 

	for (size_t i = 0; i < SC_LEN; ++i) {
		pkt_tx_time[CP_LEN + i].real() = out[i][0];
		pkt_tx_time[CP_LEN + i].imag() = out[i][1];
	}
}


void dump_signals() {
	cout << endl << "Dump signals" << endl;

	FILE* out_file;
	char tmp_n[1000];
	sprintf(tmp_n, "%s", out_name.c_str());

	out_file = fopen(tmp_n, "wb");
	fwrite(pkt_rx, sizeof(gr_complex), s_cnt, out_file);
	fclose(out_file);

	if (DEBUG) fclose(raw_file);
}

void end_sys() {
	printf("Delete receiving signal buffer\n");
	if (pkt_rx != NULL) {
		delete [] pkt_rx;
	}
	if (pkt_rx_tmp != NULL) {
		delete [] pkt_rx_tmp;
	}
}

void sig_int_handler(int){
	stop_signal = true;
}

int UHD_SAFE_MAIN(int argc, char *argv[]) {
	uhd::set_thread_priority_safe();
	uhd::time_spec_t refer;

	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "help message")
		("tx1ip", po::value<string>(&usrp_tx1_ip)->default_value("addr=192.168.10.2"), "tx1 usrp's IP")
		("tx2ip", po::value<string>(&usrp_tx2_ip)->default_value("addr=192.168.10.3"), "tx2 usrp's IP")
		("rxip", po::value<string>(&usrp_rx_ip)->default_value("addr=192.168.20.2"), "rx usrp's IP")
		("in", po::value<string>(&in_name)->default_value("../../out/tx_freq_vec.bin"), "binary samples file")
		("out", po::value<string>(&out_name)->default_value("../../out/rx_signals.bin"), "signal file")
		("i", po::value<double>(&inter)->default_value(128), "interval of two sampling")
		("f", po::value<double>(&freq)->default_value(2.43), "RF center frequency in Hz")
		("g", po::value<double>(&gain)->default_value(30.0), "gain for the RF chain")
		("s", po::value<double>(&r_sec)->default_value(RECV_SEC), "recording seconds")
		("c", po::value<size_t>(&r_cnt)->default_value(90), "round count");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << boost::format("UHD TX samples from file %s") % desc << endl;
		return ~0;
	}

	// Initialize
	init_sys();

	std::signal(SIGINT, &sig_int_handler);
	std::cout << "Press Ctrl + C to stop streaming..." << std::endl;

	// Tx cleaning
	tx_md.start_of_burst   = true;
	tx_md.end_of_burst     = false;
	tx_md.has_time_spec    = false;
	tx_md2.start_of_burst  = true;
	tx_md2.end_of_burst    = false;
	tx_md2.has_time_spec   = false;

	usrp_tx1->get_device()->send(zeros, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
	usrp_tx2->get_device()->send(zeros, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);

	tx_md.start_of_burst   = false;
	tx_md.end_of_burst	   = true; 
	tx_md2.start_of_burst  = false;
	tx_md2.end_of_burst	   = true; 
	usrp_tx1->get_device()->send(zeros, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
	usrp_tx2->get_device()->send(zeros, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);

	// Rx cleaning
	size_t done_cleaning;
	done_cleaning = 0;
	while (!done_cleaning) {
		usrp_rx->get_device()->recv(pkt_rx_tmp, SYM_LEN, rx_md, C_FLOAT32, R_ONE_PKT);
		if (rx_md.time_spec.get_real_secs() >= time_start_recv.get_real_secs()) {
			done_cleaning = 1;
		}
	}

	// Send Signals until press ^C
	// HINT: You have to send signals here
	// How many symbols you have to send? Ans: sym_cnt
	// pkt: records the samples which we want to send
	
	// David: Setup time
	boost::this_thread::sleep(boost::posix_time::milliseconds(WARM_UP_TIME));
	cout << "Current clock time: " << usrp_tx1->get_time_now().get_real_secs() << endl;
	tx_md.time_spec = usrp_tx1->get_time_now() + uhd::time_spec_t(1, 0, 1e8 / inter); 
	tx_md2.time_spec = tx_md.time_spec; 

    stream_cmd.time_spec = time_start_recv = tx_md.time_spec;
	cout << "Configured Tx time: " << time_start_recv.get_real_secs() << endl;
	cout << "Configured Tx tick: " << time_start_recv.get_tick_count(rate) << endl;
    stream_cmd.stream_now = false;
    usrp_rx->issue_stream_cmd(stream_cmd);

	// David: Let the first sample use the configured timer
	tx_md.start_of_burst  = true;
	tx_md.end_of_burst    = false;
	tx_md.has_time_spec   = true;
	tx_md2.start_of_burst = true;
	tx_md2.end_of_burst   = false;
	tx_md2.has_time_spec  = true;

	size_t rx_cnt     = 0;
	size_t rx_cnt_tmp = 0;
	size_t rx_samples = 0;
	size_t read_cnt   = 0;
	size_t tx_round   = 0;
	double sym_time   = 1 / rate * SYM_LEN;

	while (!stop_signal) {
		size_t sym_cnt 		= PREAMBLE_SYMS * 2 + NUM_SYMS * 4 ;
		size_t offset  		= 0;
		size_t preamble_cnt = 0;

		for (size_t s = 0; s < sym_cnt; ++s) {
			// Send time-domain preamble from tx1
			if (s < PREAMBLE_SYMS)  {
				preamble_cnt += usrp_tx1->get_device()->send(pkt_tx+preamble_cnt, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
				usrp_tx2->get_device()->send(zeros, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);
			} 
			// Send time-domain preamble from tx2
			else if (s < 2 * PREAMBLE_SYMS)  {
				usrp_tx1->get_device()->send(zeros, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
				preamble_cnt += usrp_tx2->get_device()->send(pkt_tx + preamble_cnt, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);
			} 

			// Convert freq-domain to time-domain and send time-domain symbols
			else {
				// Perform precoding here for symbol 150~199
				// HINT: input pkt_tx * precoding_coeff to ifft!
				// tx1: freq to time, send time signals
				ifft(pkt_tx + PREAMBLE_LEN + offset); // freq to time
				usrp_tx1->get_device()->send(pkt_tx_time, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
				//usrp_tx1->get_device()->send(zeros, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT); // DEBUG

				// tx2: freq to time, send time signals
				ifft(pkt_tx + PREAMBLE_LEN + offset);
				usrp_tx2->get_device()->send(pkt_tx_time, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);
				//usrp_tx2->get_device()->send(zeros, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT); // DEBUG

				offset += SC_LEN;

				// DEBUG: write the time-domain symbol to file if you wanna debug
				if (DEBUG && tx_round == 0) { 
					fwrite(pkt_tx_time, sizeof(gr_complex), SYM_LEN, tx_time_file);
					if (s == 2 * PREAMBLE_SYMS) { 
						for (size_t i = 0; i < SYM_LEN; ++i)
							cout << i << ": (" << pkt_tx_time[i].real() << ", " << pkt_tx_time[i].imag() <<")"<<endl; 
					} 
				}
			}

			// David: Reset has_time_spec to false
			tx_md.has_time_spec = false;
			tx_md2.has_time_spec = false;

			// David: Receive samples 
			if (rx_cnt < s_cnt) {
				read_cnt = SYM_LEN;

				// At last recv(), modify read_cnt to receive the remaining samples
				if (s_cnt - rx_cnt < read_cnt) {
					read_cnt = s_cnt - rx_cnt;
				}
				rx_samples = usrp_rx->get_device()->recv(pkt_rx_tmp + rx_cnt_tmp, read_cnt, rx_md, C_FLOAT32, R_ONE_PKT);

				// Actually recv signals when the timestamp matches tx time
				if(rx_md.time_spec.get_real_secs() + sym_time >= time_start_recv.get_real_secs()) {
					memcpy(pkt_rx + rx_cnt, pkt_rx_tmp + rx_cnt_tmp, sizeof(gr_complex) * rx_samples);
					rx_cnt += rx_samples;

					// Print timestamp when recving the first sample
					if (rx_cnt == rx_samples) {
						cout << "start recving tick: " << rx_md.time_spec.get_tick_count(rate) << endl;
						cout << "start recving " << rx_cnt << " samples at " << rx_md.time_spec.get_real_secs() << endl;
						// Perform channel estimation here
						// Learn H1 from preamble 1 and H2 from preamble 2
					}

					// Print timestamp when recving the last sample
					if (rx_cnt == s_cnt) {
						cout << "finish recving " << rx_cnt << " samples at " << rx_md.time_spec.get_real_secs() << endl;
						dump_signals();
						end_sys();	
					}
				}

				// Update rx_cnt_tmp
				rx_cnt_tmp += rx_samples;
				// Write raw signals to raw_file and reset cnt to 0
				if (rx_cnt_tmp == s_cnt) {
					// Write the samples to file if you wanna debug
					if (DEBUG) fwrite(pkt_rx_tmp, sizeof(gr_complex), s_cnt, raw_file);
					rx_cnt_tmp = 0;
				}
			}
		}
		// Clean the buffer of USRP
		for (size_t j = 0; j < 20; ++j) {
			usrp_tx1->get_device()->send(zeros, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
			usrp_tx2->get_device()->send(zeros, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);
		}
		++tx_round;
	}

	tx_md.start_of_burst    = false;
	tx_md.end_of_burst		= true; 
	tx_md2.start_of_burst  	= false;
	tx_md2.end_of_burst		= true; 
	
	usrp_tx1->get_device()->send(zeros, SYM_LEN, tx_md, C_FLOAT32, S_ONE_PKT);
	usrp_tx2->get_device()->send(zeros, SYM_LEN, tx_md2, C_FLOAT32, S_ONE_PKT);
	
	if (DEBUG) fclose(tx_time_file);

    boost::this_thread::sleep(boost::posix_time::seconds(1));
	cout << "Terminate systems ... " << endl;
	return 0;
}
