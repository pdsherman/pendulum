
#include <libs/util/util.hpp>

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>
#include <deque>

namespace util {

double interpolate(const std::vector<double> &x, const std::vector<double> &y, const double x_n)
{
  // If x_n is outside, return extreme
  if(x_n <= *x.cbegin())  { return *y.cbegin(); }
  if(x_n >= *x.crbegin()) { return *y.crbegin(); }

  for(size_t i = 0; i < x.size()-1; ++i) {
    if(x_n >= x[i] && x_n < x[i+1]) {
      double m = (y[i+1] - y[i])/(x[i+1] - x[i]);
      return y[i] + m*(x_n - x[i]);
    }
  }

  return 0.0;
}

std::map<std::string, std::vector<double>> read_data_from_csv(const std::string& csv_file)
{
  std::ifstream ifs;
  ifs.open(csv_file, std::ifstream::in);

  // Parse header to get column names
  std::string header;
  std::getline(ifs, header);

  std::vector<std::string> columns;
  boost::split(columns, header, boost::is_any_of(","));

  // Parse each line of file to get data
  std::vector<std::vector<double>> raw_data;
  for(size_t i = 0; i < columns.size(); ++i)
    raw_data.push_back(std::vector<double>());

  std::string line;
  while(std::getline(ifs, line)) {
    std::vector<std::string> data_str;
    boost::algorithm::trim(line);
    boost::split(data_str, line, boost::is_any_of(","));

    for(size_t i = 0; i < data_str.size(); ++i) {
      raw_data[i].push_back(std::stod(data_str[i]));
    }
  }

  ifs.close();

  // Collect header and data in single container
  std::map<std::string, std::vector<double>> data;
  for(size_t i = 0; i < columns.size(); ++i) {
    boost::algorithm::trim(columns[i]);
    data[columns[i]] = raw_data[i];
  }
  return data;
}

void write_data_to_csv(const std::string &csv_file, std::map<std::string, std::vector<double>> data)
{
  std::ofstream ofs;
  ofs.open(csv_file, std::ios::out | std::ios::trunc);

  // Parse column names and write first line (header)
  std::vector<std::string> headers;
  for(auto const &x : data) {
    headers.push_back(x.first);
    ofs << x.first;
    if(x.first == data.rbegin()->first) {
      ofs << "\n";
    } else {
      ofs << ",";
    }
  }

  // Use smallest vector of data if they aren't the same size.
  size_t n = (*std::min_element(data.begin(), data.end(), [](
    const std::pair<std::string, std::vector<double>> &i,
    const std::pair<std::string, std::vector<double>> &j)
    { return i.second.size() < j.second.size(); })).second.size();

  // Write data to file
  for(size_t i = 0; i < n ; ++i) {
    for(const auto &name : headers) {
      ofs << data[name][i];
      if(name == (*headers.rbegin())) {
        ofs << "\n";
      } else {
        ofs << ",";
      }
    }
  }

  ofs.close();
}

// Credit: Tatu Ylonen <ylo@cs.hut.di>
// See: https://opensource.apple.com/source/OpenSSH/OpenSSH-7.1/openssh/ttymodes.c?txt
int speed_to_baud(speed_t speed)
{
	switch (speed) {
	case B0:
		return 0;
	case B50:
		return 50;
	case B75:
		return 75;
	case B110:
		return 110;
	case B134:
		return 134;
	case B150:
		return 150;
	case B200:
		return 200;
	case B300:
		return 300;
	case B600:
		return 600;
	case B1200:
		return 1200;
	case B1800:
		return 1800;
	case B2400:
		return 2400;
	case B4800:
		return 4800;
	case B9600:
		return 9600;

#ifdef B19200
	case B19200:
		return 19200;
#else // B19200
#ifdef EXTA
	case EXTA:
		return 19200;
#endif // EXTA
#endif // B19200

#ifdef B38400
	case B38400:
		return 38400;
#else // B38400
#ifdef EXTB
	case EXTB:
		return 38400;
#endif // EXTB
#endif // B38400

#ifdef B7200
	case B7200:
		return 7200;
#endif // B7200
#ifdef B14400
	case B14400:
		return 14400;
#endif // B14400
#ifdef B28800
	case B28800:
		return 28800;
#endif // B28800
#ifdef B57600
	case B57600:
		return 57600;
#endif // B57600
#ifdef B76800
	case B76800:
		return 76800;
#endif // B76800
#ifdef B115200
	case B115200:
		return 115200;
#endif // B115200
#ifdef B230400
	case B230400:
		return 230400;
#endif // B230400
	default:
		return 9600;
	}
}

} // namespace util
