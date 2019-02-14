#include "stf_head.h"

const int WIN_SAMPS = 131072;
const int FFT_DEPTH = 131072;
int win_samps;
double centre_freq, samp_rate, bandwidth, rx_gain; //WBX daughterboard only handles 40MHz BW, GbE limits useable BW to 20MHz (with 25MS/s)
bool write_to_file;
char* sub_dev = "A:0"; //Daughterboard A, port 0 for RX. 
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

int main(int argc, char*argv[]) {

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




	return 0;
}