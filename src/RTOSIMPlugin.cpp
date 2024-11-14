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

#include "RTOSIMPlugin.h"

RTOSIMPlugin::RTOSIMPlugin()
{
	runCondition_.setRunCondition(true);
	record_ = false;
	useID_ = true;
}

RTOSIMPlugin::~RTOSIMPlugin()
{
	filteredGeneralisedCoordinatesQueue_.unsubscribe();
	jointMomentsQueue_.unsubscribe();
	runCondition_.setRunCondition(false);
	if (mainThread->joinable())
		mainThread->join();
	delete mainThread;
}

void RTOSIMPlugin::stop()
{
	filteredGeneralisedCoordinatesQueue_.unsubscribe();
	jointMomentsQueue_.unsubscribe();
	runCondition_.setRunCondition(false);
	if (mainThread->joinable())
		mainThread->join();
	delete mainThread;
}

void RTOSIMPlugin::init(std::string xmlName, std::string executionName)
{
	filteredGeneralisedCoordinatesQueue_.subscribe();
	jointMomentsQueue_.subscribe();

	// Get the execution XML files
	std::auto_ptr<ExecutionType> executionPointer;
	std::string AngleFile;

	try
	{
		std::auto_ptr<ExecutionType> executionPointer(execution(executionName, xml_schema::flags::dont_initialize));
		executionPointer = executionPointer;

		// Get the XML filename for the IK
		AngleFile = executionPointer->ConsumerPlugin().AngleDeviceFile().get();
	}
	catch (const xml_schema::exception& e)
	{
		std::cout << e << std::endl;
		exit(EXIT_FAILURE);
	}

	std::cout << AngleFile << std::endl;

	xmlInterpret_ = new XMLInterpreter(AngleFile);
	xmlInterpret_->readXML();

	latchCount_ = 3; // marker, queue adaptator and IK thread

	if (xmlInterpret_->getUseID())
		latchCount_ += 3;//  AdaptiveCop, GrfFilter and ID thread
	if (record_)
		latchCount_ += 2; // marker + ik logger
	if (xmlInterpret_->getUseID() && record_)
		latchCount_ += 2; // ID + GRF logger
	std::cout << "latchCount_: " << latchCount_ << std::endl;
	doneWithSubscriptions_.setCount(latchCount_ + 1); // +1 for CEINMS

	mainThread = new std::thread(&RTOSIMPlugin::threadLancher, this, executionName);
	doneWithSubscriptions_.wait();
}

void RTOSIMPlugin::threadLancher(std::string executionName)
{
	std::string hostname = xmlInterpret_->getIP() + ":" + std::to_string(xmlInterpret_->getPort());
	std::vector<std::string> coordNames = coordNames_ = getCoordinateNamesFromModel(xmlInterpret_->getOsimFile());
	std::vector<std::string> modelMarkerNames = modelMarkerNames_ = getMarkerNamesFromModel(xmlInterpret_->getOsimFile());
	GeneralisedCoordinatesStateSpace gcFilt(xmlInterpret_->getFc(), coordNames.size());

	//define the shared buffers
	MarkerSetQueue markerSetQueue;
	GeneralisedCoordinatesQueue generalisedCoordinatesQueue;
	MultipleExternalForcesQueue grfQueue, adaptedGrfQueue, filteredGrfQueue;

	rtb::Concurrency::Latch doneWithExecution;
	//FlowControl runCondition(true);

	std::auto_ptr<LaboratoryType> laboratory;
	try
	{
		std::auto_ptr<LaboratoryType> laboratoryPointer(Laboratory(xmlInterpret_->getLabFile()));
		laboratory = laboratoryPointer;
	}
	catch (const xml_schema::exception& e)
	{
		std::cout << e << std::endl;
		exit(EXIT_FAILURE);
	}

	// Read amount of external forces (and their ID) from lab XML
	typedef LaboratoryType::ExternalForcesList_sequence ExternalForceSeq;
	typedef LaboratoryType::ExternalForcesList_type::ExternalForce_sequence ExternalForceSeqSeq;
	const ExternalForceSeq& externalForceSeq = laboratory->ExternalForcesList();
	std::vector<std::string> extForceID;
	for (ExternalForceSeq::const_iterator it = externalForceSeq.begin(); it != externalForceSeq.end(); it++)
	{
		for (ExternalForceSeqSeq::const_iterator it2 = it->ExternalForce().begin(); it2 != it->ExternalForce().end(); it2++)
		{
			extForceID.push_back(it2->ID());
		}
	}

	// Add threshold forceplate in XML
	MultipleExternalForcesDataFilterStateSpace multipleExternalForcesDataFilterStateSpace(xmlInterpret_->getFc(), 40, laboratory->NumberOfForcePlatforms()+extForceID.size());
	std::vector<std::string> GRFCol;
	for (int cpt = 0; cpt < laboratory->NumberOfForcePlatforms(); cpt++)
		GRFCol.push_back(std::to_string(cpt)); // In qualisys the forceplate does not have a name so we use number as name here;

	for (int i = 0; i < extForceID.size(); i++)
	{
		GRFCol.push_back(extForceID.at(i));
	}

	doneWithExecution.setCount(latchCount_);

	DataFromQualisys dataFromQualisys(
		markerSetQueue,
		grfQueue,
		doneWithSubscriptions_,
		doneWithExecution,
		runCondition_,
		modelMarkerNames,
		xmlInterpret_->getLabFile(),
		hostname);

	QueueToInverseKinematics inverseKinematics(
		markerSetQueue,
		generalisedCoordinatesQueue,
		doneWithSubscriptions_,
		doneWithExecution,
		xmlInterpret_->getOsimFile(),
		xmlInterpret_->getNumberofThreadForIK(), xmlInterpret_->getIkTaskFilename(), xmlInterpret_->getMaxMarkerError());

	QueueAdapter <GeneralisedCoordinatesQueue, GeneralisedCoordinatesQueue, GeneralisedCoordinatesStateSpace>
		gcQueueAdaptor(generalisedCoordinatesQueue,
			filteredGeneralisedCoordinatesQueue_,
			doneWithSubscriptions_,
			doneWithExecution,
			gcFilt);

	if (!xmlInterpret_->getUseID() && !record_)
	{
		useID_ = false;
		QueuesSync::launchThreads(
			dataFromQualisys,
			inverseKinematics,
			gcQueueAdaptor
		);
	}
	else if (record_ && !xmlInterpret_->getUseID())
	{
		QueueToFileLogger<MarkerSetData> markerLogger(
			markerSetQueue,
			doneWithSubscriptions_,
			doneWithExecution,
			modelMarkerNames,
			outDirectory_, "marker", "trc");

		QueueToFileLogger<GeneralisedCoordinatesData> filteredIkLogger(
			filteredGeneralisedCoordinatesQueue_,
			doneWithSubscriptions_,
			doneWithExecution,
			getCoordinateNamesFromModel(xmlInterpret_->getOsimFile()),
			outDirectory_, "ik", "sto");
		filteredIkLogger.setIsInDegrees(false);

		//read the frames from generalisedCoordinatesQueue and calculates some stats
		rtosim::FrameCounter<GeneralisedCoordinatesQueue> ikFrameCounter(
			generalisedCoordinatesQueue,
			"time-ik-throughput");

		useID_ = false;

		QueuesSync::launchThreads(
			dataFromQualisys,
			inverseKinematics,
			filteredIkLogger,
			markerLogger,
			gcQueueAdaptor,
			ikFrameCounter
		);


		auto stopWatches = inverseKinematics.getProcessingTimes();

		rtosim::StopWatch combinedSW("time-ikparallel-processing");
		for (auto& s : stopWatches)
			combinedSW += s;

		combinedSW.print(outDirectory_);
		ikFrameCounter.getProcessingTimes().print(outDirectory_);
	}
	else if (xmlInterpret_->getUseID())
	{
		AdaptiveCop adaptiveCop(
			markerSetQueue,
			grfQueue,
			adaptedGrfQueue,
			doneWithSubscriptions_,
			doneWithExecution,
			xmlInterpret_->getOsimFile(),
			xmlInterpret_->getExternalLoadsXml()
		);

		GrfFilter grfFilter(
			adaptedGrfQueue,
			filteredGrfQueue,
			doneWithSubscriptions_,
			doneWithExecution,
			multipleExternalForcesDataFilterStateSpace);

		QueuesToInverseDynamics queueToInverseDynamics(
			filteredGeneralisedCoordinatesQueue_,
			filteredGrfQueue,
			jointMomentsQueue_,
			doneWithSubscriptions_,
			doneWithExecution,
			xmlInterpret_->getOsimFile(),
			xmlInterpret_->getExternalLoadsXml());
		if (!record_)
		{
			QueuesSync::launchThreads(
				dataFromQualisys,
				inverseKinematics,
				gcQueueAdaptor,
				adaptiveCop,
				grfFilter,
				queueToInverseDynamics
			);
		}
		else
		{
			QueueToFileLogger<MarkerSetData> markerLogger(
				markerSetQueue,
				doneWithSubscriptions_,
				doneWithExecution,
				modelMarkerNames,
				outDirectory_, "marker", "trc");

			QueueToFileLogger<GeneralisedCoordinatesData> filteredIkLogger(
				filteredGeneralisedCoordinatesQueue_,
				doneWithSubscriptions_,
				doneWithExecution,
				getCoordinateNamesFromModel(xmlInterpret_->getOsimFile()),
				outDirectory_, "ik", "sto");
			filteredIkLogger.setIsInDegrees(false);

			QueueToFileLogger<ExternalTorquesData> idLogger(
				jointMomentsQueue_,
				doneWithSubscriptions_,
				doneWithExecution,
				getCoordinateNamesFromModel(xmlInterpret_->getOsimFile()),
				outDirectory_, "id", "sto");
			idLogger.setIsInDegrees(false);

#ifdef UNFILT
				QueueToFileLogger<MultipleExternalForcesData> filtGRFLogger(
					grfQueue, // Before: filteredGrfQueue / Now: grfQueue
					doneWithSubscriptions_,
					doneWithExecution,
					GRFCol,
					outDirectory_, "GRF", "mot");

#else
				QueueToFileLogger<MultipleExternalForcesData> filtGRFLogger(
					filteredGrfQueue, // Before: filteredGrfQueue / Now: grfQueue
					doneWithSubscriptions_,
					doneWithExecution,
					GRFCol,
					outDirectory_, "GRF", "mot");

#endif // UNFILT



			//read the frames from generalisedCoordinatesQueue and calculates some stats
			rtosim::FrameCounter<GeneralisedCoordinatesQueue> ikFrameCounter(
				generalisedCoordinatesQueue,
				"time-ik-throughput");

			//read the frames from generalisedCoordinatesQueue and calculates some stats
			rtosim::FrameCounter<ExternalTorquesQueue> idFrameCounter(
				jointMomentsQueue_,
				"time-id-throughput");

			QueuesSync::launchThreads(
				dataFromQualisys,
				inverseKinematics,
				gcQueueAdaptor,
				adaptiveCop,
				grfFilter,
				queueToInverseDynamics,
				filteredIkLogger,
				markerLogger,
				idLogger,
				filtGRFLogger,
				ikFrameCounter,
				idFrameCounter
			);

			auto stopWatches = inverseKinematics.getProcessingTimes();
			queueToInverseDynamics.getProcessingTimes().print(outDirectory_);

			rtosim::StopWatch combinedSW("time-ikparallel-processing");
			for (auto& s : stopWatches)
				combinedSW += s;

			combinedSW.print(outDirectory_);
			ikFrameCounter.getProcessingTimes().print(outDirectory_);
			idFrameCounter.getProcessingTimes().print(outDirectory_);
		}

	}

	std::cout << "end thread" << std::endl;


}

const std::map<std::string, double>& RTOSIMPlugin::GetDataMap()
{
	//std::cout << filteredGeneralisedCoordinatesQueue_ << std::endl;
	GeneralisedCoordinatesFrame lastIK = filteredGeneralisedCoordinatesQueue_.pop(); // check to see if it's too fast
	timeStamp_ = lastIK.time;
	//std::cout << std::setprecision(15)<< timeStamp_ << std::endl;
	std::vector<double>  lastData = lastIK.data.getQ();
	for (std::vector<std::string>::const_iterator it = coordNames_.begin(); it != coordNames_.end(); it++)
		positionMap_[*it] = lastData[std::distance<std::vector<std::string>::const_iterator>(coordNames_.begin(), it)];//*/
	return positionMap_;
}

const std::map<std::string, double>& RTOSIMPlugin::GetDataMapTorque()
{
	if (useID_)
	{
		ExternalTorquesFrame lastID = jointMomentsQueue_.pop(); // check to see if it's too fast
		std::vector<double>  lastData = lastID.data;
		for (std::vector<std::string>::const_iterator it = coordNames_.begin(); it != coordNames_.end(); it++)
			torqueMap_[*it] = lastData[std::distance<std::vector<std::string>::const_iterator>(coordNames_.begin(), it)];//*/
	}
	return torqueMap_;
}

// For the run-time loading of the library

#ifdef UNIX
extern "C" ProducersPluginVirtual * create() {
	return new RTOSIMPlugin;
}

extern "C" void destroy(ProducersPluginVirtual * p) {
	delete p;
}
#endif

#ifdef WIN32 // __declspec (dllexport) id important for dynamic loading
extern "C" __declspec (dllexport) ProducersPluginVirtual * __cdecl create() {
	return new RTOSIMPlugin;
}

extern "C" __declspec (dllexport) void __cdecl destroy(ProducersPluginVirtual * p) {
	delete p;
}
#endif
