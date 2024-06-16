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
	void loadHRTFData(sofa::GeneralFIR& file);

	void setPosition(double azi, double ele);
	void spatialise(const juce::AudioSourceChannelInfo& bufferToFill, float azi, float ele);

private:
	enum State
	{
		UNPREPARED,
		PREPARED
	};

	struct HRTFMapping
	{
		double m_azi;
		double m_ele;
		int m_leftITD;
		int m_rightITD;
		std::unique_ptr<float[]> m_leftIR;
		std::unique_ptr<float[]> m_rightIR;
	};

	void convolve(float* leftSignal, float* rightSignal, std::unique_ptr<float[]>& leftIR, std::unique_ptr<float[]>& rightIR);

	float m_azi;
	float m_ele;

	State m_state;

	// Expected audio buffer values
	int m_numSamplesPerBlock;
	double m_inputSampleRate;

	// Sofa file
	std::unique_ptr<juce::FileChooser> m_fileChooser;

	// HRTF collection (interpolated at set angle intervals)
	double m_radius; // Distance at which HRTF was measured
	int m_IRNumSamples; // number of samples contained in one IR measurement
	std::vector< HRTFMapping> m_HRTFMappingCollection; // Collect of HRTFs, their posittions and ITDs

	// Convolver output history
	std::unique_ptr<float[]> m_leftConvolveOutput;
	std::unique_ptr<float[]> m_rightConvolveOutput;
};