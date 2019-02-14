//File for spawning a new USRP instance (return ip address string?)

#include "stf_head.h"


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

	if (FFT && false) {
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