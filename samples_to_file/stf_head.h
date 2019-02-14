#ifndef STF_HEAD_H
#define STF_HEAD_H
//header for samples to file

#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>
#include <cmath>
#include <vector>
#include <fftw3/fftw3.h>
#include <Windows.h>


/**************** Prototypes *****************/
int receiver(int, char**, std::complex<short>*, size_t*, bool);
//void fft_proc(short*, size_t*);
void fft_proc(std::complex<double>*, size_t*, int);
void fftf_proc(std::complex<float>*, int);
void fftf_proc_s(std::complex<short>*, int, size_t*);
void recv_to_file(uhd::usrp::multi_usrp::sptr, const std::string&, const std::string&, const std::string&,
	size_t, std::complex<short>*, size_t*, unsigned long long, double, bool, bool, bool, bool, bool);



//Globals


#endif