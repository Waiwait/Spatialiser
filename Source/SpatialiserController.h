#pragma once

#include <JuceHeader.h>
#include "SOFA.h"

class MainComponent;

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

	explicit SpatialiserController(MainComponent* mainComponentArg);
	~SpatialiserController();

	void prepareToPlay(int samplesPerBlockExpected, double sampleRate);

	void openSOFAFile();
	void loadHRTFData();

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

	// Juce thread class for loading HRTF data off thread
	class LoadHRTFDataThread : private juce::Thread
	{
		friend SpatialiserController;
	public:
		LoadHRTFDataThread(SpatialiserController& spatialiserController)
			: Thread("Load HRTF data thread") 
			, m_spatialiserController(spatialiserController)
		{}
		void run() override {
			m_spatialiserController.loadHRTFData();
		};

	private:
		SpatialiserController& m_spatialiserController;
	};

	void interpolateHRTF(HRTFMapping& targetHRTF, const std::vector< HRTFMapping >& hrtfMappingList);
	void convolve(juce::AudioSampleBuffer& buffer);

	MainComponent* m_mainComponent;

	LoadHRTFDataThread loadHRTFDataThread;
	State m_state;

	// Expected audio buffer values
	int m_numSamplesPerBlock;
	double m_inputSampleRate;

	// Sofa file
	std::unique_ptr<juce::FileChooser> m_fileChooser;
	std::unique_ptr<sofa::GeneralFIR> m_sofaFIRFile;

	// HRTFs
	double m_radius; // Distance at which HRTF was measured
	int m_IRNumSamples; // number of samples contained in one IR measurement
	std::vector< HRTFMapping> m_HRTFMappingCollection; // Collection of HRTFs, their posittions and ITDs

	// List of points and how far they are from a target point. Used in data prep when interpolating from raw data and
	// during run-time spatialisation when interpolating from prepped data
	std::vector<DistMapping> m_distMappingList;

	// Final interpolated HRTF to use when spatialising
	HRTFMapping m_outputHRTF;

	// Delay line to store smaples for FFT and for ITD delays
	int m_currITD[2];
	std::unique_ptr<float[]> m_delayLine[2];
	int m_delayLineIdx[2];
	int m_delayLineSize;

	// FFT Convolution
	std::unique_ptr<juce::dsp::WindowingFunction<float>> m_windowController;
	std::unique_ptr<juce::dsp::FFT> m_fftController;

	std::unique_ptr<float[]> m_irFft;
	std::unique_ptr<float[]> m_inputFft;
	std::unique_ptr<float[]> m_convolveOutput[2];
	int m_samplesConvolved;
	int m_convolveOutputBufferSize;
};