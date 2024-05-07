#pragma once

#include <JuceHeader.h>
#include "SOFA.h"

class SpatialiserController
{
public:
	SpatialiserController();
	~SpatialiserController();

	void prepareToPlay(int samplesPerBlockExpected, double sampleRate);

	void openSOFAFile();
	void spatialise(const juce::AudioSourceChannelInfo& bufferToFill, float azi, float ele);

private:
	enum State
	{
		UNPREPARED,
		PREPARED
	};

	struct IRMapping
	{
		double azi;
		double ele;
		float* leftIR;
		float* rightIR;
	};

	void convolve(float* leftSignal, float* rightSignal, float* leftIR, float* rightIR);

	State state;

	// Expected audio buffer values
	int numSamplesPerBlock;
	double inputSampleRate;

	// Sofa file
	std::unique_ptr<juce::FileChooser> fileChooser;
	std::unique_ptr<sofa::File> file;

	// IRs
	std::unique_ptr<float[]> rawIRs;
	size_t IRNumSamples; // number of samples contained in one IR measurement
	std::vector< IRMapping> IRMappingCollection;

	// Convolver output history
	std::unique_ptr<float[]> leftConvolveOutput;
	std::unique_ptr<float[]> rightConvolveOutput;
};