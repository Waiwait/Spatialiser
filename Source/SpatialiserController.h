#pragma once

#include <JuceHeader.h>
#include "SOFA.h"

class SpatialiserController
{
public:
	struct DistMapping
	{
		int m_mappingIdx;
		double m_distance;
		double m_azi;
		double m_ele;
	};

	SpatialiserController();
	~SpatialiserController();

	void prepareToPlay(int samplesPerBlockExpected, double sampleRate);

	void openSOFAFile();
	void loadHRTFData(sofa::GeneralFIR& file);

	void setPosition(double azi, double ele);
	void spatialise(const juce::AudioSourceChannelInfo& bufferToFill);

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
		// 0 = left ear, 1 = right ear
		int m_ITD[2];
		std::unique_ptr<float[]> m_IR[2];
	};

	void interpolateHRTF(HRTFMapping& targetHRTF, const std::vector< HRTFMapping >& hrtfMappingList);
	void convolve(float* leftSignal, float* rightSignal, std::unique_ptr<float[]>& leftIR, std::unique_ptr<float[]>& rightIR);

	State m_state;

	// Expected audio buffer values
	int m_numSamplesPerBlock;
	double m_inputSampleRate;

	// Sofa file
	std::unique_ptr<juce::FileChooser> m_fileChooser;

	// HRTFs
	double m_radius; // Distance at which HRTF was measured
	int m_IRNumSamples; // number of samples contained in one IR measurement
	std::vector< HRTFMapping> m_HRTFMappingCollection; // Collection of HRTFs, their posittions and ITDs

	// List of points and how far they are from a target point. Used in data prep when interpolating from raw data and
	// during run-time spatialisation when interpolating from prepped data
	std::vector<DistMapping> m_distMappingList;

	// Final interpolated HRTF to use when spatialising
	HRTFMapping m_outputHRTF;

	// Convolver output history
	std::unique_ptr<float[]> m_leftConvolveOutput;
	std::unique_ptr<float[]> m_rightConvolveOutput;
};