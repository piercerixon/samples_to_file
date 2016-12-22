//
//
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <Python.h>
#include <numpy/arrayobject.h>
//#include <boost/python.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>
#include <cmath>
#include <vector>
#include <fftw3/fftw3.h>
#include <Windows.h>

namespace po = boost::program_options;

//Normal UHD code
static bool stop_signal_called = false;
void sig_int_handler(int){ stop_signal_called = true; }

//Global Python objects for outputting plots
PyObject *pFunc, *pArg;
PyArrayObject* pArray;
PyObject *pTup = PyTuple_New(4);

//Globals
int frame_count = 0;

const std::string currentDateTime() {

	SYSTEMTIME st;
	GetSystemTime(&st);

	char currentTime[85] = "";
	sprintf_s(currentTime, "%d/%d/%d  %d:%d:%d %d ", st.wDay, st.wMonth, st.wYear, st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);

	return std::string(currentTime);
}

//Measurement globals
//hack
const int WIN_SAMPS = 131072;
const int FFT_DEPTH = 2048;
//const int WIN_SAMPS = 2048;
int win_samps;
double centre_freq, samp_rate, bandwidth, rx_gain; //WBX daughterboard only handles 40MHz BW, GbE limits useable BW to 20MHz (with 25MS/s)
bool write_to_file;
char* sub_dev = "A:0";
double GB_4 = 4294967296 / (sizeof(short) * 2);

//Program conditions for different modes
static bool DISPLAY = false;
static bool FFT = true;
static bool TIMING = false;
const int NUM_THREADS = 1;
const bool VERBOSE = false;
static bool LOG = true;

//Logging
std::ofstream logfile;

//mutexes for updating display
boost::mutex display_mtx, stream_mtx, fft_mtx;
boost::condition_variable_any disp_cond;//, fft_cond, stream_cond;

//mutexes and cond vars for FFT threads and receiver
boost::mutex fft_mutex[NUM_THREADS], recv_mutex, super_mutex;
boost::condition_variable_any fft_cond, recv_cond, super_cond;

static const double pi = double(std::acos(-1.0));

//Container for Plot
bool frame_ready = false;
const int num_disp_pts = 2048;

/**************** Prototypes *****************/
int receiver(int, char**, std::complex<short>*, size_t*, bool);
//void fft_proc(short*, size_t*);
void fft_proc(std::complex<double>*, size_t*, int);
void fftf_proc(std::complex<float>*, int);
void fftf_proc_s(std::complex<short>*, int, size_t*);
void recv_to_file(
	uhd::usrp::multi_usrp::sptr,
	const std::string&,
	const std::string&,
	const std::string&,
	size_t,
	std::complex<short>*,
	size_t*,
	unsigned long long,
	double,
	bool,
	bool,
	bool,
	bool,
	bool);

// Reciever
void recv_to_file(
	uhd::usrp::multi_usrp::sptr usrp,
	const std::string &cpu_format,
	const std::string &wire_format,
	const std::string &file,
	size_t samps_per_buff,
	std::complex<short>* buff,
	size_t* num_rx_samps,
	unsigned long long num_requested_samples,
	double time_requested = 0.0,
	bool bw_summary = false,
	bool stats = false,
	bool null = false,
	bool enable_size_map = false,
	bool continue_on_bad_packet = false
	){
	unsigned long long num_total_samps = 0;
	//create a receive streamer
	uhd::stream_args_t stream_args(cpu_format, wire_format);
	uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

	uhd::rx_metadata_t md;
	std::ofstream outfile;
	std::string filename;
	int fileItr = 1;

	filename = file + "_0.dat";

	if (LOG) logfile.open("logfile.log", std::ofstream::out);
	if (not null && write_to_file)
		outfile.open(filename.c_str(), std::ofstream::binary);
	bool overflow_message = true;

	//setup streaming
	uhd::stream_cmd_t stream_cmd((num_requested_samples == 0) ?
		uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS :
		uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
		);
	stream_cmd.num_samps = size_t(num_requested_samples);
	stream_cmd.stream_now = true;
	stream_cmd.time_spec = uhd::time_spec_t();
	rx_stream->issue_stream_cmd(stream_cmd);

	boost::system_time start = boost::get_system_time();
	unsigned long long ticks_requested = (long)(time_requested * (double)boost::posix_time::time_duration::ticks_per_second());
	boost::posix_time::time_duration ticks_diff;
	boost::system_time last_update = start;
	unsigned long long last_update_samps = 0;

	typedef std::map<size_t, size_t> SizeMap;
	SizeMap mapSizes;

	boost::unique_lock<boost::mutex> lock(recv_mutex);

	if (FFT) {
		super_cond.notify_one(); //wake the supervisor up after USRP connected
		std::cout << "wake up supervisor - love, reciever" << std::endl;
		recv_cond.wait(lock);
		std::cout << "reciever awake! Time for them samples" << std::endl;
	}
	//Rx Streamer here

	while (not stop_signal_called and(num_requested_samples != num_total_samps or num_requested_samples == 0)){

		boost::system_time now = boost::get_system_time();

		//*num_rx_samps = rx_stream->recv(buff, win_samps*NUM_THREADS, md, 3.0, enable_size_map);	
		*num_rx_samps = rx_stream->recv(buff, win_samps, md, 3.0, enable_size_map);

		//Update total number of samples
		num_total_samps += *num_rx_samps;

		if (num_total_samps > GB_4*fileItr && write_to_file){
			outfile.close();
			filename = file + "_";
			filename += boost::lexical_cast<std::string>(fileItr);
			filename += ".dat";
			fileItr++;
			outfile.open(filename.c_str(), std::ofstream::binary);
			if (LOG) logfile << filename << " Created" << std::endl;
		}


		if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) {
			std::cout << boost::format("Timeout while streaming") << std::endl;
			if (LOG) logfile << currentDateTime() << boost::format("Timeout while streaming") << std::endl;
			break;
		}
		if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_OVERFLOW){
			if (overflow_message){
				overflow_message = false;
				std::cerr << boost::format(
					"Got an overflow indication. Please consider the following:\n"
					"  Your write medium must sustain a rate of %fMB/s.\n"
					"  Dropped samples will not be written to the file.\n"
					"  Please modify this example for your purposes.\n"
					"  This message will not appear again.\n"
					) % (usrp->get_rx_rate()*sizeof(std::complex<double>) / 1e6);
				if (LOG) logfile << currentDateTime() << boost::format(
					"Got an overflow indication. Please consider the following:\n"
					"  Your write medium must sustain a rate of %fMB/s.\n"
					"  Dropped samples will not be written to the file.\n"
					"  Please modify this example for your purposes.\n"
					"  This message will not appear again.\n"
					) % (usrp->get_rx_rate()*sizeof(std::complex<double>) / 1e6);
			}
			continue;
		}
		if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
			std::string error = str(boost::format("Receiver error: %s") % md.strerror());
			if (LOG) logfile << currentDateTime() << boost::format("Receiver error: %s") % md.strerror();
			if (continue_on_bad_packet){
				std::cerr << error << std::endl;
				continue;
			}
			else
				throw std::runtime_error(error);
		}

		if (enable_size_map){
			SizeMap::iterator it = mapSizes.find(*num_rx_samps);
			if (it == mapSizes.end())
				mapSizes[*num_rx_samps] = 0;
			mapSizes[*num_rx_samps] += 1;
		}

		//for(int i = 0; i < 8; i++) {
		//	std::cout << hackbuff[i] << " ";
		//}

		//Write samples to outfile
		if (outfile.is_open())
			//outfile.write((const char*)buff, *num_rx_samps*sizeof(std::complex<double>));
			outfile.write((const char*)buff, *num_rx_samps*sizeof(std::complex<short>));

		//std::cout << hackbuff[0] << " " << hackbuff[1] << " " << hackbuff[2] << " " << hackbuff[3] << " " << hackbuff[4] << std::endl;

		if (*num_rx_samps != NUM_THREADS*samps_per_buff) {
			std::cout << boost::format("Number of samples written: %f") % (*num_rx_samps) << std::endl;
			if (LOG) logfile << currentDateTime() << boost::format("Number of samples written: %f") % (*num_rx_samps) << std::endl;
		}

		last_update_samps += *num_rx_samps;
		boost::posix_time::time_duration update_diff = now - last_update;
		if (update_diff.ticks() > boost::posix_time::time_duration::ticks_per_second()) {
			double t = (double)update_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
			double r = (double)last_update_samps / t;
			std::cout << boost::format("\t%f Msps at %fMB/s") % (r / 1e6) % (r*sizeof(std::complex<short>) / 1e6) << std::endl;
			if (LOG) logfile << currentDateTime() << boost::format("\t%f Msps at %fMB/s") % (r / 1e6) % (r*sizeof(std::complex<short>) / 1e6) << std::endl;
			//std::cout << "FPS: " << frame_count << std::endl;
			frame_count = 0;
			last_update_samps = 0;
			last_update = now;
		}

		ticks_diff = now - start;
		if (ticks_requested > 0){
			if ((unsigned long long)ticks_diff.ticks() > ticks_requested)
				break;
		}


		if (VERBOSE) std::cout << "Wake up supervisor, I have recieved " << *num_rx_samps << " :D" << std::endl;
		if (FFT) {
			super_cond.notify_one();
			recv_cond.wait(lock);
		}
	}

	//After loop ends, exit gracefully
	stream_cmd.stream_mode = uhd::stream_cmd_t::STREAM_MODE_STOP_CONTINUOUS;
	rx_stream->issue_stream_cmd(stream_cmd);

	if (outfile.is_open())
		outfile.close();

	std::cout << std::endl;

	double t = (double)ticks_diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
	std::cout << boost::format("Received %d samples (%d MB) in %f seconds") % num_total_samps % (num_total_samps*sizeof(std::complex<short>) / 1e6) % t << std::endl;
	if (LOG) logfile << boost::format("Received %d samples (%d MB) in %f seconds") % num_total_samps % (num_total_samps*sizeof(std::complex<short>) / 1e6) % t << std::endl;

	if (logfile.is_open()) logfile.close();


	if (enable_size_map) {
		std::cout << std::endl;
		std::cout << "Packet size map (bytes: count)" << std::endl;
		for (SizeMap::iterator it = mapSizes.begin(); it != mapSizes.end(); it++)
			std::cout << it->first << ":\t" << it->second << std::endl;

	}
}

typedef boost::function<uhd::sensor_value_t(const std::string&)> get_sensor_fn_t;

bool check_locked_sensor(std::vector<std::string> sensor_names, const char* sensor_name, get_sensor_fn_t get_sensor_fn, double setup_time){
	if (std::find(sensor_names.begin(), sensor_names.end(), sensor_name) == sensor_names.end())
		return false;

	boost::system_time start = boost::get_system_time();
	boost::system_time first_lock_time;

	//std::cout << boost::format("Waiting for \"%s\": ") % sensor_name;
	std::cout << "Waiting for " << sensor_name << ": ";
	std::cout.flush();

	while (true){
		if ((not first_lock_time.is_not_a_date_time()) and
			(boost::get_system_time() > (first_lock_time + boost::posix_time::seconds(setup_time))))
		{
			std::cout << " locked." << std::endl;
			break;
		}
		if (get_sensor_fn(sensor_name).to_bool()){
			if (first_lock_time.is_not_a_date_time())
				first_lock_time = boost::get_system_time();
			std::cout << "+";
			std::cout.flush();
		}
		else{
			first_lock_time = boost::system_time();	//reset to 'not a date time'

			if (boost::get_system_time() > (start + boost::posix_time::seconds(setup_time))){
				std::cout << std::endl;
				throw std::runtime_error(str(boost::format("timed out waiting for consecutive locks on sensor \"%s\"") % sensor_name));
			}
			std::cout << "_";
			std::cout.flush();
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}
	std::cout << std::endl;
	return true;
}

/**************** Main *****************/

int main(int argc, char*argv[]) {

	_putenv_s("PYTHONPATH", ".");
	Py_Initialize();
	import_array();

	/*
	std::cout << "To the Python mobile!" << std::endl << std::endl;

	_putenv_s("PYTHONPATH", ".");

	Py_Initialize();

	PyObject *pName, *pModule, *pDict, *pFunc, *pArg;

	pName = PyUnicode_FromString((char*)"python_test");
	pModule = PyImport_Import(pName);
	pDict = PyModule_GetDict(pModule);
	pFunc = PyObject_GetAttrString(pModule, (char*)"pyitup");

	if (PyCallable_Check(pFunc)) {
		std::cout << "It worked?\n";
		pArg = PyTuple_New(1);
		PyTuple_SetItem(pArg,0,PyUnicode_FromString((char*)"FIRE ZE CANNON"));
		PyObject_CallObject(pFunc, pArg);
	}
	else {
		PyErr_Print();
	}

	Py_DECREF(pModule);
	Py_DECREF(pName);

	system("PAUSE");
	return 0;
	*/
	//int pythonvictory = result;

	//pyplot("attack!");
	//std::cout << "Python number is: " << pyout << std::endl;

	//Welcome text here
	std::cout << "How to use: \n/f centre freq \n/b bandwidth \n/r sample rate \n/s transport size (2^n) \n/g rx gain (0 - 25) \n/w output to file (0,1)\n" << std::endl;

	char *charout;
	int optionmask = 0; //freq,bw,rate,sampsize,gain actually using a bitmask, but C++ seems to use ints instead of bits
	write_to_file = false;

	for (int i = 1; i < argc - 1; i++) { //starts at 1 as argv[0] is the program name. 

		charout = argv[i];

		if (charout[0] == '/' && charout[1] != NULL) {

			switch (charout[1]) {
			case 'f': std::cout << "Centre Frequency = " << atof(argv[++i]) / 1e6 << "MHz" << std::endl;
				centre_freq = atof(argv[i]);
				optionmask += 16;
				break;
			case 'b': std::cout << "Bandwidth = " << atof(argv[++i]) / 1e6 << "MHz" << std::endl;
				bandwidth = atof(argv[i]);
				optionmask += 8;
				break;
			case 'r': std::cout << "Sample Rate = " << atof(argv[++i]) / 1e6 << "MS/s" << std::endl;
				samp_rate = atof(argv[i]);
				optionmask += 4;
				break;
			case 's': std::cout << "Transport Size = " << pow(2.0, atof(argv[++i])) << std::endl;
				win_samps = pow(2.0, atof(argv[i]));
				optionmask += 2;
				break;
			case 'g': std::cout << "RX Gain = " << atoi(argv[++i]) << "dB" << std::endl;
				rx_gain = atoi(argv[i]);
				optionmask += 1;
				break;
			case 'w': std::cout << "Output to file is ";
				if (atoi(argv[++i]) == 0) { std::cout << "DISABLED" << std::endl; write_to_file = false; }
				else { std::cout << "ENABLED" << std::endl; write_to_file = true; }
				break;
			}
		}
	}

	if (~optionmask & 31){
		std::cout << std::endl << "Setting Program Defaults:" << std::endl;
	}
	if (~optionmask & 16) {
		std::cout << "Default Centre Frequency = " << 104e6 / 1e6 << "MHz" << std::endl;
		centre_freq = 104e6;
	}
	if (~optionmask & 8) {
		std::cout << "Default Bandwidth = " << 40e6 / 1e6 << "MHz" << std::endl;
		bandwidth = 40e6;
	}
	if (~optionmask & 4) {
		std::cout << "Default Sample Rate = " << 6.25e6 / 1e6 << "MHz" << std::endl;
		samp_rate = 6.25e6;
	}
	if (~optionmask & 2) {
		std::cout << "Default Transport Size = " << pow(2.0, 10) << " Transport Size should be set to n=17 for optimal result" << std::endl;
		win_samps = pow(2.0, 10.0); //1024
	}
	if (~optionmask & 1) {
		std::cout << "Default RX Gain = " << 0 << "dB" << std::endl;
		rx_gain = 25;
	}

	std::cout << std::endl;

	boost::this_thread::sleep(boost::posix_time::seconds(3)); //let the user ponder the gravity of their decisions

	/************** This is now the manager thread *******************/

	std::vector<std::complex<short>> buff(NUM_THREADS*win_samps);
	std::complex<short>* rcv_buff_ptr = &buff[0];
	//std::vector<std::complex<float>> buff(NUM_THREADS*win_samps);
	//std::complex<float>* rcv_buff_ptr = &buff[0];

	size_t num_rx_samps = 0;
	size_t* write_samp_ptr = &num_rx_samps;
	size_t* read_samp_ptr = &num_rx_samps;

	if (!FFT) {
		std::cout << "Only Recording!" << std::endl;
		receiver(argc, argv, rcv_buff_ptr, write_samp_ptr, true); //THIS IS A HACK TO JUST RECORD
		return 0;
	}

	//std::cout << "num_rx_samps = " << num_rx_samps << " samp_ptr = " << samp_ptr << " &samp_ptr = " << &samp_ptr << " *samp_ptr = " << *samp_ptr << " &samp_ptr[0] = " << &samp_ptr[0] << std::endl;
	//Fire up the FFTs :D

	if (FFT) {
		boost::thread_group fft_thread_swarm;
		std::complex<short>* buff_ptr[NUM_THREADS];

		for (int i = 0; i < NUM_THREADS; i++) {
			buff_ptr[i] = &buff[i*win_samps];

			//fft_thread_swarm.create_thread((boost::bind(fftf_proc_s, boost::ref(buff_ptr[i]), boost::ref(read_samp_ptr), i)));
			fft_thread_swarm.create_thread((boost::bind(fftf_proc_s, boost::ref(buff_ptr[i]), i, boost::ref(read_samp_ptr))));
			boost::this_thread::sleep(boost::posix_time::milliseconds(100));
		}
	}

	boost::thread receiver_thread(boost::bind(receiver, argc, boost::ref(argv), boost::ref(rcv_buff_ptr), boost::ref(write_samp_ptr), false));
	//receiver(argc, argv, rcv_buff_ptr, write_samp_ptr, false);

	boost::unique_lock<boost::mutex> lock(super_mutex);

	int fft_inactive = 0;
	bool recvtex = false;

	super_cond.wait(lock);
	//std::cout << "supervisor sleep 1" << std::endl;
	//recv_cond.notify_one();
	std::cout << "Commence supervision" << std::endl;
	//Keeps track of the receiver state and each FFT threads state
	//TODO: keep track of buffers and handle writing out/displaying. Processing of FFT'd information is also to be taken care of
	boost::system_time time1, time2, now1, now2;
	while (true) {

		for (int i = 0; i < NUM_THREADS; i++)
			if (fft_mutex[i].try_lock()) fft_inactive++;

		if (fft_inactive == NUM_THREADS && recv_mutex.try_lock()) {
			//Spin the ffts!
			if (TIMING) now1 = boost::get_system_time();

			recvtex = true;
			for (int i = 0; i < NUM_THREADS; i++) fft_mutex[i].unlock();
			fft_inactive = 0;
			//std::cout << "waiting for fft to finish" << std::endl;
			fft_cond.notify_all();

			//This is to avoid the FFT threads 'wake up' getting lost due to race conditions
			boost::chrono::system_clock::time_point time_limit = boost::chrono::system_clock::now() + boost::chrono::milliseconds(2000);
			if (boost::cv_status::timeout == super_cond.wait_until(lock, time_limit)) {
				if (LOG) logfile << "SuperV: Timed out! ";
				std::cout << "A Supervisor Timeout has occured on FFT" << std::endl;
			}

			if (TIMING)  {
				boost::system_time time = boost::get_system_time();
				boost::posix_time::time_duration diff = time - now1;
				double timeu = (double)diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
				std::cout << boost::format("FFT time: %d") % timeu << std::endl;
			}
		}

		if (fft_inactive == NUM_THREADS && recvtex) {
			if (TIMING) now2 = boost::get_system_time();

			recv_mutex.unlock();
			recvtex = false;
			//std::cout << "waiting for receiver to finish" << std::endl;
			recv_cond.notify_one();
			boost::chrono::system_clock::time_point time_limit = boost::chrono::system_clock::now() + boost::chrono::milliseconds(200);
			if (boost::cv_status::timeout == super_cond.wait_until(lock, time_limit)) {
				if (LOG) logfile << "SuperV: Timed out! ";
				std::cout << "A Supervisor Timeout has occured on Receive" << std::endl;
			}
			//boost::this_thread::sleep(boost::posix_time::milliseconds(.1));

			if (TIMING) {
				boost::system_time time = boost::get_system_time();
				boost::posix_time::time_duration diff = time - now2;
				double timeu = (double)diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
				std::cout << boost::format("Recv time: %d") % timeu;

				time1 = boost::get_system_time();
				boost::posix_time::time_duration diffv = time1 - time2;
				double timev = (double)diffv.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
				std::cout << boost::format("\tCycle time: %d") % timev << std::endl;
				time2 = boost::get_system_time();
			}
		}

		//if (frame_ready) {
		//	frame_ready = false;
		//	disp_cond.notify_one();
		//}
	}
	/*
	if(false) {}
	if(fft_inactive == NUM_THREADS) {
	std::cout << "FFTs stopped" << std::endl;
	for(int i = 0; i < NUM_THREADS; i++)
	fft_mutex[i].unlock();
	fft_cond.notify_all();
	fft_inactive = 0;
	super_cond.wait(super_mutex);
	} else recv_mutex.unlock();
	*/

	return 0;
}

int receiver(int argc, char *argv[], std::complex<short>* buff_ptr, size_t* samp_ptr, bool sample){

	uhd::set_thread_priority_safe();

	//variables to be set by po
	std::string args, file, type, ant, subdev, ref, wirefmt, cpufmt;
	size_t total_num_samps, spb;
	double rate, freq, gain, bw, total_time, setup_time;

	//setup the program options
	po::options_description desc("Allowed options");
	desc.add_options()
		("help", "help message")
		("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
		("file", po::value<std::string>(&file)->default_value("samples"), "name of the file to write binary samples to")
		("type", po::value<std::string>(&type)->default_value("short"), "sample type: double, float, or short") //format to store SUPERSCEDED by cpufmt
		("nsamps", po::value<size_t>(&total_num_samps)->default_value(0), "total number of samples to receive") //set to 0 for unlimited
		("duration", po::value<double>(&total_time)->default_value(0), "total number of seconds to receive") //set to 0 for continuous 
		("time", po::value<double>(&total_time), "(DEPRECATED) will go away soon! Use --duration instead")
		("spb", po::value<size_t>(&spb)->default_value(win_samps), "samples per buffer")
		("rate", po::value<double>(&rate)->default_value(samp_rate), "rate of incoming samples")
		("freq", po::value<double>(&freq)->default_value(centre_freq), "RF center frequency in Hz")
		("gain", po::value<double>(&gain)->default_value(rx_gain), "gain for the RF chain")
		("ant", po::value<std::string>(&ant), "antenna selection")
		("subdev", po::value<std::string>(&subdev)->default_value(sub_dev), "subdevice specification") //Set up for X300 with UBX160 in slot B. http://www.ettus.com/content/files/kb/application_note_frontends_subdevices_antenna_ports.pdf
		("bw", po::value<double>(&bw)->default_value(bandwidth), "analog frontend filter bandwidth in Hz")
		("ref", po::value<std::string>(&ref)->default_value("internal"), "reference source (internal, external, mimo)")
		("wirefmt", po::value<std::string>(&wirefmt)->default_value("sc16"), "wire format (sc8 or sc16)") //format to receive from USRP
		("cpufmt", po::value<std::string>(&cpufmt)->default_value("sc16"), "cpu format (sc16, fc32, fc64)") //format USRP samples are stored (does not impact ethernet throughput)
		("setup", po::value<double>(&setup_time)->default_value(1.0), "seconds of setup time")
		("progress", "periodically display short-term bandwidth")
		("stats", "show average bandwidth on exit")
		("sizemap", "track packet size and display breakdown on exit")
		("null", "run without writing to file")
		("continue", "don't abort on a bad packet")
		("skip-lo", "skip checking LO lock status")
		("int-n", "tune USRP with integer-N tuning")
		;
	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	boost::this_thread::sleep(boost::posix_time::milliseconds(2000));

	//print the help message
	if (vm.count("help")){
		std::cout << boost::format("UHD RX samples to file %s") % desc << std::endl;
		return ~0;
	}

	bool bw_summary = vm.count("progress") > 0;
	bool stats = vm.count("stats") > 0;
	bool null = vm.count("null") > 0;
	bool enable_size_map = vm.count("sizemap") > 0;
	bool continue_on_bad_packet = vm.count("continue") > 0;

	if (enable_size_map)
		std::cout << "Packet size tracking enabled - will only recv one packet at a time!" << std::endl;

	//create a usrp device
	std::cout << std::endl;
	std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
	uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

	//Lock mboard clocks
	usrp->set_clock_source(ref);

	//always select the subdevice first, the channel mapping affects the other settings
	if (vm.count("subdev")) usrp->set_rx_subdev_spec(subdev);

	//std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;
	//std::cout << "Using Device: " << usrp->get_pp_string() << std::endl;

	//Write configuration data to file
	// Config is as follows: name; centre freq, bandwidth, sample rate, rx gain (number of samples per window irrelevant here)
	//Yes its ugly, but, eh, it works. n.n
	if (false) {
		std::ofstream config;
		config.open("sample_config.cfg");
		config << "config filename or something" << std::endl;
		config << usrp->get_mboard_name() << std::endl;
		config << freq << std::endl;
		config << bw << std::endl;
		config << rate << std::endl;
		config << gain;
		config.close();

		std::cout << "Configuration file created\n" << std::endl;
	}

	//set the sample rate
	if (rate <= 0.0){
		std::cerr << "Please specify a valid sample rate" << std::endl;
		return ~0;
	}
	std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate / 1e6) << std::endl;
	usrp->set_rx_rate(rate);
	std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate() / 1e6) << std::endl << std::endl;

	//set the center frequency
	if (vm.count("freq")){	//with default of 0.0 this will always be true
		std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq / 1e6) << std::endl;
		uhd::tune_request_t tune_request(freq);//, 10e6);
		if (vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
		usrp->set_rx_freq(tune_request);
		std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq() / 1e6) << std::endl << std::endl;
	}

	//set the rf gain
	if (vm.count("gain")){
		std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
		usrp->set_rx_gain(gain);
		std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
	}

	//set the analog frontend filter bandwidth
	if (vm.count("bw")){
		std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw / 1e6) << std::endl;
		usrp->set_rx_bandwidth(bw);
		std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % (usrp->get_rx_bandwidth() / 1e6) << std::endl << std::endl;
	}

	//set the antenna
	if (vm.count("ant")) usrp->set_rx_antenna(ant);

	boost::this_thread::sleep(boost::posix_time::seconds(setup_time)); //allow for some setup time
	boost::this_thread::sleep(boost::posix_time::seconds(setup_time));
	/*
	//check Ref and LO Lock detect
	if (not vm.count("skip-lo")){
		check_locked_sensor(usrp->get_rx_sensor_names(0), "lo_locked", boost::bind(&uhd::usrp::multi_usrp::get_rx_sensor, usrp, _1, 0), setup_time);
		if (ref == "mimo")
			check_locked_sensor(usrp->get_mboard_sensor_names(0), "mimo_locked", boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, 0), setup_time);
		if (ref == "external")
			check_locked_sensor(usrp->get_mboard_sensor_names(0), "ref_locked", boost::bind(&uhd::usrp::multi_usrp::get_mboard_sensor, usrp, _1, 0), setup_time);
	}
	*/
	if (total_num_samps == 0){
		std::signal(SIGINT, &sig_int_handler);
		std::cout << "Press Ctrl + C to stop streaming..." << std::endl << std::endl;
	}

	recv_to_file(usrp, cpufmt, wirefmt, file, spb, buff_ptr, samp_ptr, total_num_samps, total_time, bw_summary, stats, null, enable_size_map, continue_on_bad_packet);

	//finished
	std::cout << std::endl << "Done!" << std::endl << std::endl;
	return 0;
}

/********* HERE BE THE FUNCTIONS ********/


void fftf_proc_s(std::complex<short>* re, int idx, size_t* fsz) {

	//boost::unique_lock<boost::mutex> lock(fft_mutex[idx]);
	//Prepare your body for the FFT
	//CONVENTION fftwf = floats, instead of doubles used before. AS we only have 14 bits of precision it doesnt make sense to use double point precision
	// but im using doubles anyway :D
	//LARGE_INTEGER wind;
	//QueryPerformanceCounter(&wind);
	std::cout << "launching fft thread" << std::endl;
	boost::unique_lock<boost::mutex> lock(fft_mutex[idx]);
	std::complex<float>* buff;
	buff = (std::complex<float>*) malloc(sizeof(std::complex<float>) * WIN_SAMPS); //Buffer for floated samples n.n

	std::complex<short>* s_ptr = NULL;

	std::complex<float>* f_ptr = buff;

	fftwf_complex *fft_buff;
	fftwf_plan plan;
	float win_power = 0;
	float *w_array, *avg_buff, *pow_buff;
	int avg_count = 0;
	const char* filename = "fftwf_plan";

	int frame_count = 0;
	int error_counter = 0;
	const int NUM_WINS = 1;

	int ws_array[WIN_SAMPS];
	unsigned char ws_frame[WIN_SAMPS];
	int overlap[WIN_SAMPS];

	//const int for number of running averages to take
	const int AVG = 10;

	//long long ws_count = 0;
	//std::vector<window> window_vec;
	//std::vector<window>* w_vec_ptr = &window_vec;
	int frame_number = 0;
	bool init = false;

	fft_buff = (fftwf_complex*)fftwf_malloc(sizeof(fftwf_complex) * WIN_SAMPS); //buffer allocated for Output
	w_array = (float*)malloc(sizeof(float) * WIN_SAMPS); //allocate memory for window coefficients
	avg_buff = (float*)malloc(sizeof(float) * WIN_SAMPS * AVG); //buffer allocated for averaging FFT results
	pow_buff = (float*)malloc(sizeof(float) * WIN_SAMPS); //buffer for current power levels - added with moving average code

	for (int i = 0; i < WIN_SAMPS; i++){
		ws_array[i] = 0;
		ws_frame[i] = 0;
		overlap[i] = 0;
		avg_buff[i] = 0;
	}


	//added for peak detector
	int avg_row = 0;
	int fft_idx = 0;

	for (int i = 0; i < AVG; i++) {
		for (int j = 0; j < WIN_SAMPS; j++){
			avg_buff[i*WIN_SAMPS + j] = 0;
		}
	}

	//go here fam: http://www.fftw.org/fftw3.pdf
	int res[] = { WIN_SAMPS };
	int stride = 1;
	int dist = WIN_SAMPS;
	int batch = 10; //FIXME

	//Cast the std::complex<double> as a fftw_complex
	if (!fftwf_import_wisdom_from_filename(filename)) {
		std::cout << "should use wisdom next time :)" << std::endl;
		
		//plan = fftwf_plan_dft_1d(WIN_SAMPS, reinterpret_cast<fftwf_complex*>(&buff[0]), fft_buff, FFTW_FORWARD, FFTW_MEASURE);


		plan = fftwf_plan_many_dft(1, res, batch, reinterpret_cast<fftwf_complex*>(&buff[0]), res, stride, dist, fft_buff, res, stride, dist, FFTW_FORWARD, FFTW_MEASURE);
		std::cout << "FFTW PLAN: " << fftwf_export_wisdom_to_filename(filename) << std::endl;
	}
	//plan = fftwf_plan_dft_1d(WIN_SAMPS, reinterpret_cast<fftwf_complex*>(&buff[0]), fft_buff, FFTW_FORWARD, FFTW_PATIENT);
	else plan = fftwf_plan_dft_1d(WIN_SAMPS, reinterpret_cast<fftwf_complex*>(&buff[0]), fft_buff, FFTW_FORWARD, FFTW_MEASURE);

	//Create coefficient array and x axis index for plotting
	for (int i = 0; i < WIN_SAMPS; i++) {
		w_array[i] = 0.35875 - 0.48829*cos(2 * pi*i / (WIN_SAMPS - 1)) + 0.14128*cos(4 * pi*i / (WIN_SAMPS - 1)) - 0.01168*cos(6 * pi*i / (WIN_SAMPS - 1)); //blackmann harris window		
		win_power += (w_array[i] * w_array[i]); //this computes the total window power and normalises it to account for DC gain due to the window.
	}
	win_power /= WIN_SAMPS; //normalise the total window power across each sample.


	//std::cout << 10*std::log10(win_power) << std::endl;

	//double offset = (- 10*std::log10(win_power) //DC gain of window
	//				 - (174 - 10*std::log10(bandwidth/samp_rate))); //Noise floor
	float offset = -10 - rx_gain + 10 * std::log10(win_power); //-10 is the MAX power detected by the ADC and take into account the gain of the frontend.

	std::cout << "FFT[" << idx << "]: Spawned" << std::endl;

	std::ofstream fftout;
	fftout.open("fftout.csv");

	long long samp = 0;
	double chars_read = 0.0;
	int reduce = 0;
	double last_count = 0;
	double rate = 0;
	double progress = 0;
	boost::posix_time::time_duration diff, read;
	boost::posix_time::ptime now1;
	double time_diff;
	//std::cout << std::fixed << std::setprecision(2);

	LARGE_INTEGER ftpl, wind, avge, perffreq;
	double wf, fa, ae, pf;
	QueryPerformanceFrequency(&perffreq);
	pf = perffreq.QuadPart;
	int testGPU = 0;

	std::vector<float> plot(num_disp_pts, 0);

	while (true) {
		s_ptr = re;
		//QueryPerformanceCounter(&wind);
		//for (int i = 0, j = 0; i < WIN_SAMPS*2; i+=2, j++) { //WIN_SAMPS*2 BECAUSE THEY ARE COMPLEX
		for (int i = 0; i < WIN_SAMPS; i++) {
			//Looks janky, but is actually for good reason. The modulo, is to ensure the f_ptr does not overrun buff, as buff is complex, there is 2 floats per 'float'
			//the s_ptr points to the entire dataset cached in memory, thus it can run untill the end of the file

			f_ptr[i].real(((*s_ptr).real()*1.0f / 32767.0f) * w_array[i]);
			f_ptr[i].imag(((*s_ptr).imag()*1.0f / 32767.0f) * w_array[i]);
			s_ptr++;
			samp += 4;
		}

		//QueryPerformanceCounter(&ftpl);

		//Spin the FFT yoooooooooo
		fftwf_execute(plan);

		//QueryPerformanceCounter(&avge);

		/******* OLD AVERAGING CODE
		//Keep a block average of every FFT that has happened until the averaging says so :D
		for (int i = 0; i < WIN_SAMPS; i++) {
			avg_buff[i] += (
				10 * std::log10(std::abs(fft_buff[i][0] * fft_buff[i][0] + fft_buff[i][1] * fft_buff[i][1]) / WIN_SAMPS) //DFT bin magnitude
				);
		}
		*******/

		//New spicy moving average code, with peak detection. Correctly rotate entries into avg_buff (could just multiply the spectrum by +1,-1,+1,-1... ), and then compute power
		
		for (int i = 0; i < WIN_SAMPS; i++) {

			pow_buff[i] - avg_buff[avg_row*WIN_SAMPS + i];

			fft_idx = ((WIN_SAMPS / 2) + i) % WIN_SAMPS;
			avg_buff[avg_row*WIN_SAMPS + i] = (
				10 * std::log10(std::abs(fft_buff[fft_idx][0] * fft_buff[fft_idx][0] + fft_buff[fft_idx][1] * fft_buff[fft_idx][1]) / (WIN_SAMPS*AVG)) //DFT bin magnitude
				);

			pow_buff[i] + avg_buff[avg_row*WIN_SAMPS + i];
			
			//now detect peaks :D 
		//	if (i > 0 && i < WIN_SAMPS - 1){
		//		if (pow_buff[i - 1] < pow_buff[i] && pow_buff[i + 1] < pow_buff[i]) { 
		//			std::cout << "Peak Detected at: " << i * bandwidth/WIN_SAMPS + (centre_freq - bandwidth/2) << "Hz, Pow: " << pow_buff[i] + offset << "dbm" << std::endl; 
		//		}
		//	} 
		}

		avg_row++ % AVG;

		//end spice

		//wf = (ftpl.QuadPart - wind.QuadPart)/pf;
		//QueryPerformanceCounter(&wind);	
		//fa = (avge.QuadPart - ftpl.QuadPart)/pf;
		//ae = (wind.QuadPart - avge.QuadPart)/pf;
		//std::cout << "Windowing: " << wf << "s FFT: " << fa << "s Avg: " << ae << "s" << std::endl; 
		//Count the number of FFT frames that have been processed thus far
		avg_count++;


		//Perform ws detection - Legacy code
		/***
		if (avg_count > 0 && avg_count%10 == 0 && false) {
			//frame_count++;


			//testGPU++;
			//QueryPerformanceCounter(&ftpl);
			//wf = (ftpl.QuadPart - wind.QuadPart) / pf;
			//std::cout << "CPU Time : " << wf << std::endl;
			//std::cout << "CPU OFFSET : " << offset << std::endl;

			std::cout << "Outputting samples";
			float temp;
			for (int i = 0; i < WIN_SAMPS; i++) {
				temp = (avg_buff[((WIN_SAMPS / 2) + i) % WIN_SAMPS] / 10 + offset);// <= threshold ? 1 : 0;
				std::cout << temp << ", ";
				//std::cout << avg_buff[((WIN_SAMPS / 2) + i) % WIN_SAMPS] << ", ";
			}
			std::cout << std::endl;

			frame_number++;

			// DISPLAY CODE, got to flip the buffer around correctly !!!!

			//display the averaged samples
			if (idx == 0 && display_mtx.try_lock() && DISPLAY) { //ENTER CRITICAL REGION, will not block the FFT thread.
			if(LOG) logfile << "FFT[" << idx << "]: Displaying ";

			std::cout << "Updating frame with averages: " << avg_count << std::endl;

			int step = WIN_SAMPS/num_disp_pts;
			float val_max, val_min = 0;
			float temp = 0;
			int index = 0;

			for (int i = 0; i < num_disp_pts; i++) {
				index = i*step;
				val_max = -1000;
				val_min = 0;
				temp = -1000; //If you actually get a level lower than this, you have issues :)

				for (int j = 0; j < step; j++) {//Grab the max and min values from each step set
					temp = (avg_buff[index + j]/avg_count);

					if(temp > val_max) {val_max = temp;} //Take the MAX value
					else if(temp < val_min) {val_min = temp;} //Else Take the MIN value

					//avg_buff[index + j] = 0;

					//std::cout << "point: " << ((WIN_SAMPS/2)+i)%WIN_SAMPS << " val: " << (avg_buff[i] / avg_count) << std::endl;
				}

				val_max += offset;
				plot[((num_disp_pts / 2) + i) % num_disp_pts] = val_max;
			}

				display_mtx.unlock();
				disp_cond.notify_one();
			}
			//Clear the average once it has been processed, to prepare it for the next set.
			for(int i = 0; i < WIN_SAMPS; i++) avg_buff[i] = 0;
			avg_count = 0;
		}
		***/

		super_cond.notify_one();
		fft_cond.wait(lock);
	}

	std::cout << "FFT[" << idx << "]: Terminating" << std::endl;
	//destroy fft plan when all the fun is over
	fftwf_destroy_plan(plan);
	fftwf_free(fft_buff);
	fftwf_cleanup_threads();
	free(w_array);
}

//deprecated
void fft_proc(std::complex<double>* buff, size_t* num_rx_samps, int idx) {

	boost::unique_lock<boost::mutex> lock(fft_mutex[idx]);
	//Prepare your body for the FFT
	//CONVENTION fftwf = floats, instead of doubles used before. AS we only have 14 bits of precision it doesnt make sense to use double point precision
	// but im using doubles anyway :D

	fftw_complex *fft_buff;
	fftw_plan plan;
	double win_power = 0;
	double *w_array, *avg_buff;
	int avg_count = 0;
	const char* filename = "fftw_plan";

	fft_buff = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * win_samps); //buffer allocated for Output
	w_array = (double*)malloc(sizeof(double) * win_samps); //allocate memory for window coefficients

	avg_buff = (double*)malloc(sizeof(double) * win_samps); //buffer allocated for averaging FFT results

	//std::cout << "Optimising FFT Routines" << std::endl;
	//int temp = fftw_init_threads();
	//fftw_plan_with_nthreads(4);

	//Cast the std::complex<double> as a fftw_complex
	if (!fftw_import_wisdom_from_filename(filename)) std::cout << "should use wisdom next time :)" << std::endl;
	//plan = fftw_plan_dft_1d(win_samps, reinterpret_cast<fftw_complex*>(&buff[0]), fft_buff, FFTW_FORWARD, FFTW_PATIENT);
	plan = fftw_plan_dft_1d(win_samps, reinterpret_cast<fftw_complex*>(&buff[0]), fft_buff, FFTW_FORWARD, FFTW_MEASURE);
	//std::cout << "FFTW PLAN: " << fftw_export_wisdom_to_filename(filename) << std::endl;
	//std::cout << "Optimisation Complete" << std::endl;

	//Create coefficient array and x axis index for plotting
	for (int i = 0; i < win_samps; i++) {
		w_array[i] = 0.35875 - 0.48829*cos(2 * pi*i / (win_samps - 1)) + 0.14128*cos(4 * pi*i / (win_samps - 1)) - 0.01168*cos(6 * pi*i / (win_samps - 1)); //blackmann harris window		
		win_power += (w_array[i] * w_array[i]); //this computes the total window power and normalises it to account for DC gain due to the window.
	}
	win_power /= win_samps; //normalise the total window power across each sample.

	//double offset = (- 10*std::log10(win_power) //DC gain of window
	//				 - (174 - 10*std::log10(bandwidth/samp_rate))); //Noise floor
	double offset = -10 - rx_gain; //-10 is the MAX power detected by the ADC and take into account the gain of the frontend.

	std::cout << "FFT Thread " << idx << " spawned" << std::endl;
	fft_cond.wait(lock);

	while (true) {

		boost::system_time now = boost::get_system_time();
		//Apply window to samples
		if ((int)(*num_rx_samps) == win_samps*NUM_THREADS) {
			for (int i = 0; i < win_samps; i++) {
				buff[i] *= w_array[i];
			}

			//Spin the FFT yoooooooooo
			fftw_execute(plan);

			//Keep a moving average of every FFT that has happened until the averaging says so :D
			//Probably need to optimise this better. Pointer arithmetic is probably the go. 
			//#pragma omp parallel for

			for (int i = 0; i < win_samps; i++) {
				avg_buff[i] += (
					10 * std::log10((std::abs(fft_buff[i][0])*std::abs(fft_buff[i][0]) + std::abs(fft_buff[i][1])*std::abs(fft_buff[i][1])) / WIN_SAMPS) //DFT bin magnitude
					);
			}

			//Count the number of FFT frames that have been processed thus far
			avg_count++;

			// DISPLAY CODE, got to flip the buffer around correctly !!!!
			if (avg_count % 10 == 0) {
				//display the averaged samples
				if (idx == 0 && display_mtx.try_lock() && DISPLAY) { //ENTER CRITICAL REGION, will not block the FFT thread.
					if (LOG) logfile << "FFT[" << idx << "]: Displaying ";

					std::cout << "Updating frame with averages: " << avg_count << std::endl;

					int step = WIN_SAMPS / num_disp_pts;
					float val_max, val_min = 0;
					float temp = 0;
					int index = 0;

					for (int i = 0; i < num_disp_pts; i++) {
						index = i*step;
						val_max = -1000;
						val_min = 0;
						temp = -1000; //If you actually get a level lower than this, you have issues :)

						for (int j = 0; j < step; j++) {//Grab the max and min values from each step set
							temp = (avg_buff[index + j] / avg_count);

							if (temp > val_max) { val_max = temp; } //Take the MAX value
							else if (temp < val_min) { val_min = temp; } //Else Take the MIN value
						}

						val_max += offset;
						//OUTPUT THE DISPLAY HERE
					}

					display_mtx.unlock();
					disp_cond.notify_one();
				}
				for (int i = 0; i < WIN_SAMPS; i++) avg_buff[i] = 0;
				avg_count = 0;
			}
			//std::cout << "Thread " << num << " FFT'd" << std::endl;

		}
		else {
			std::cout << "FFT Skipped" << std::endl;
		}

		//boost::system_time time = boost::get_system_time();
		//boost::posix_time::time_duration diff = time - now;
		//double timeu = (double)diff.ticks() / (double)boost::posix_time::time_duration::ticks_per_second();
		//std::cout << boost::format("FFT: %d") % timeu << std::endl;

		super_cond.notify_one();
		fft_cond.wait(lock);
	}

	std::cout << "ARRRGGGHHHH you killed FFT" << idx << std::endl;
	//destroy fft plan when all the fun is over
	fftw_destroy_plan(plan);
	fftw_free(fft_buff);
	fftw_cleanup_threads();
	free(w_array);
}

/****************** EDIT THESE FIRST PLS ******************************/
void call_py_plot(double *inArr, int scale, int id, double total) {

	if (PyCallable_Check(pFunc)) {

		const int arry_H = 1;
		npy_intp arry_W[arry_H] = { 2048 }; //The way to read this is: array[array_height][array_width]

		//std::cout << "Cast array\n";
		pArg = PyArray_SimpleNewFromData(arry_H, arry_W, NPY_DOUBLE, reinterpret_cast<void*>(inArr));
		pArray = reinterpret_cast<PyArrayObject*>(pArg);
		//std::cout << "Call Python\n";

		PyTuple_SET_ITEM(pTup, 0, pArg);
		PyTuple_SET_ITEM(pTup, 1, PyLong_FromLong(scale));
		PyTuple_SET_ITEM(pTup, 2, PyLong_FromLong(id));
		PyTuple_SET_ITEM(pTup, 3, PyLong_FromDouble(total));

		PyGILState_STATE gstate = PyGILState_Ensure();

		PyObject_CallObject(pFunc, pTup);

		//Just incase ;)
		PyErr_Print();
	}
	else {
		std::cout << "Plotting function unavailable\n\nTerminating\n";
		PyErr_Print();
		system("pause");
		exit(0);
	}
}

void init_py_plot() {

	PyObject *pName, *pModule, *pDict;

	char* py_file = "plot_helper";

	pName = PyUnicode_FromString(py_file);
	pModule = PyImport_Import(pName);
	if (pModule) {
		std::cout << "Module Loaded\n";
		pDict = PyModule_GetDict(pModule);
		pFunc = PyObject_GetAttrString(pModule, (char*)"plot_array");

	}
	else {
		PyErr_Print();
	}

	Py_DECREF(pModule);
	Py_DECREF(pName);

	//Test Routine
	/*
	double x[131072] = { 50, 50, 403, 20, 1 };
	call_py_plot(x,5,0,297);
	x[5] = 10000;
	call_py_plot(x, 5, 1, 497);

	exit(0);
	*/
}
