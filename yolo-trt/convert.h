#ifndef _CONVERT_H_
#define _CONVERT_H_

#include <stdio.h>
#include <string.h>
#include "NvInfer.h"
#include "NvInferPlugin.h"
#include "NvOnnxParser.h"
using namespace nvinfer1;
using namespace nvonnxparser;
class Logger: public ILogger
{
public:
	Logger(ILogger::Severity severity=ILogger::Severity::kWARNING):severity_(severity){}
	void log(ILogger::Severity severity, const char *msg) noexcept override
	{
		if (severity <= severity_)
			fprintf(stderr,"%s\n",msg);
	}
	ILogger::Severity severity_;
};
inline bool convert_onnx(const char* filename, const char* savename, uint32_t flag, int32_t threads=1, bool force=false, int64_t imgwidth=0, int64_t imgheight=0, int64_t batchsize=4, const char* inputname="images")
{
	Logger logger;
	static IBuilder* builder = createInferBuilder(logger);
	INetworkDefinition* network = builder->createNetworkV2(1U << static_cast<uint32_t>(NetworkDefinitionCreationFlag::kEXPLICIT_BATCH));
	IParser* parser = createParser(*network, logger);
	fprintf(stderr,"Parsing onnx model %s\n",filename);
	if (!initLibNvInferPlugins(&logger,"")||!parser->parseFromFile(filename, static_cast<int32_t>(ILogger::Severity::kWARNING)))
	{
		for (int32_t i = 0; i < parser->getNbErrors(); ++i)
			logger.log(ILogger::Severity::kINTERNAL_ERROR,parser->getError(i)->desc());
		delete parser;
		delete network;
		return false;
	}
	static IBuilderConfig* config = builder->createBuilderConfig();
	config->reset();
	config->setFlags(flag),builder->setMaxThreads(threads);
	if(imgwidth>0&&imgheight>0&&batchsize>0)
	{
		fprintf(stderr,"Creating optimization profile for dynamic tensors: Dims4{%ld, 3, %ld, %ld})\n",batchsize,imgwidth,imgheight);
		IOptimizationProfile* profile=builder->createOptimizationProfile();
		profile->setDimensions(inputname, OptProfileSelector::kMIN, Dims4(1, 3, imgwidth, imgheight));
		profile->setDimensions(inputname, OptProfileSelector::kOPT, Dims4(batchsize, 3, imgwidth, imgheight));
		profile->setDimensions(inputname, OptProfileSelector::kMAX, Dims4(batchsize*2, 3, imgwidth, imgheight));
		if(profile->isValid())
			config->addOptimizationProfile(profile);
		else
			fprintf(stderr,"Profile is invalid\n");
	}
	bool written=true; //OK
	fprintf(stderr,"Opening onnx model %s for writing\n",savename);
	FILE* file=fopen(savename,force?"wb":"wbx"); //Open file and wait to receive data, do this before building so the output file is sure to be opened (prevent time wasting)
	if(file)
	{
		fprintf(stderr,"Converting model, please wait...\n");
		IHostMemory* serializedModel = builder->buildSerializedNetwork(*network, *config);
		if(serializedModel)
		{
			if(written=static_cast<bool>(fwrite(serializedModel->data(),serializedModel->size(),1,file)))
				fprintf(stderr,"Saved engine to %s\n",savename);
			else
				perror("Failed to save engine");
			delete serializedModel;
		}
		else
			logger.log(ILogger::Severity::kINTERNAL_ERROR,"Engine is not generated");
		fclose(file);
	}
	else
		perror("Failed to open model");
	//Cleanup
	delete parser;
	delete network;
	return file&&written;
}

#endif
