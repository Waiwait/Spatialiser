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
		double m_azi;
		double m_ele;
		float* m_leftIR;
		float* m_rightIR;
	};

	void convolve(float* leftSignal, float* rightSignal, float* leftIR, float* rightIR);

	State m_state;

	// Expected audio buffer values
	int m_numSamplesPerBlock;
	double m_inputSampleRate;

	// Sofa file
	std::unique_ptr<juce::FileChooser> m_fileChooser;

	// IRs
	std::unique_ptr<float[]> m_rawIRs;
	size_t m_IRNumSamples; // number of samples contained in one IR measurement
	std::vector< IRMapping> m_IRMappingCollection;

	// Convolver output history
	std::unique_ptr<float[]> m_leftConvolveOutput;
	std::unique_ptr<float[]> m_rightConvolveOutput;
};