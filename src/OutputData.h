/*****************************************************************************
*                                                                            *
*  //Copyright (c) 2016, Vibekananda Dutta, WUT
  // Faculty of Power and Aeronautical Engineering (MEiL)/ZTMiR Laboratory
  // Warsaw University of Technology
 //  All rights reserved.
*                                                                            *
****************************************************************************/

#ifndef _OUTPUT_DATA_H_
#define _OUTPUT_DATA_H_

#include <fstream>
#include <time.h>
#include <string>

class OutputData
{
public:
	static void Init()
	{
	    time_t t = time(0);   // get time now
	    struct tm * now = localtime( & t );

	    std::stringstream ss;
	    ss << (now->tm_year + 1900) << '-' 
		 << (now->tm_mon + 1) << '-'
		 <<  now->tm_mday << "_"
		 << now->tm_hour << ":"
		 << now->tm_min << ":"
		 << now->tm_sec;
	    PostFix = ss.str();

	    CsvExtension = ".csv";
	}

	static std::string CreateCSVFilename(const std::string &basename)
	{
		return std::string("/home/vibek/Human_intention/OutputData/") + basename + "_" + PostFix + CsvExtension;
	}

	
	class ScopedFileStreamForAppend
	{
	public:
		ScopedFileStreamForAppend(const std::string &basename)
		{
			x_file.open(OutputData::CreateCSVFilename(basename).c_str(), std::ios::app);
		}
		~ScopedFileStreamForAppend()
		{
			x_file.close();
		}

		ofstream &GetStream() { return x_file; }

	private:
		ofstream x_file;
	};


private:
	static std::string PostFix;
	static std::string CsvExtension; 
};

#endif

