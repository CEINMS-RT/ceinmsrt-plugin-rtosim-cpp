// Copyright (c) 2015, Guillaume Durandau and Massimo Sartori
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// - Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// - Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef RTOSIMPlugin_H_
#define RTOSIMPlugin_H_

#include "rtosim/rtosim.h"
using namespace rtosim;

#include <iostream>
#include <string>
#include "execution.hxx"
#include "XMLInterpreter.h"
#include "ProducersPluginVirtual.h"
//#include "rtosim/Laboratory.hxx"
#include "rtosim/LaboratoryExtended.hxx"
#include "boost/filesystem.hpp"
/**
 * Plugin for CEINMS-RT for real-time use of the IK using Qualisys markers
 */

class RTOSIMPlugin : public ProducersPluginVirtual
{
public:
	/**
	 * Constructor
	 */
	RTOSIMPlugin();

	/**
	 * Destructor
	 */
	virtual ~RTOSIMPlugin();

	/**
	 * Initialization method
	 * @param xmlName Subject specific XMl filename, not use here (we use the osim files)
	 * @param executionName Execution xml filename for CEINMS-RT
	 */
	void init(std::string xmlName, std::string executionName);

	void reset()
	{
	}

	void stop();

	/**
	 * Get the data with the name of the joint mapping the angle
	 */
	const std::map<std::string, double>& GetDataMap();

	const std::map<std::string, double>& GetDataMapTorque();

	const double& getTime()
	{
		return timeStamp_;
	}

	void setDirectories(std::string outDirectory, std::string inDirectory = std::string())
	{
		outDirectory_ = outDirectory;
		outDirectory_ = rtosim::FileSystem::getAbsolutePath(outDirectory_);
		rtosim::FileSystem::createDirectory(outDirectory_);
		inDirectory_ = inDirectory;
	}

	void setVerbose(int verbose)
	{

	}

	void setRecord(bool record)
	{
		record_ = record;
	}


protected:

	void threadLancher(std::string executionName);
	std::thread* mainThread;

	using GrfFilter = QueueAdapter < MultipleExternalForcesQueue, MultipleExternalForcesQueue, MultipleExternalForcesDataFilterStateSpace >;

	GeneralisedCoordinatesQueue  filteredGeneralisedCoordinatesQueue_;
	ExternalTorquesQueue jointMomentsQueue_;
	FlowControl runCondition_;

	std::vector<std::string> coordNames_, modelMarkerNames_;

	bool record_;
	std::string outDirectory_, inDirectory_;
	double timeStamp_;
	std::map<std::string, double> positionMap_, torqueMap_;

	bool useID_;

	//define the barriers
	rtb::Concurrency::Latch doneWithSubscriptions_;
	XMLInterpreter* xmlInterpret_;
	int latchCount_;
};

#endif /* RTOSIMPlugin_H_ */
