#pragma once

#include <JuceHeader.h>
#include "SOFA.h"

class SpatialiserController
{
public:
	SpatialiserController();
	~SpatialiserController();

	void openSOFAFile();
	void spatialise(const juce::AudioSourceChannelInfo& bufferToFill, float azi, float ele) const;

private:
	struct IRMapping
	{
		double azi;
		double ele;
		float* leftIR;
		float* rightIR;
	};

	// Sofa file
	std::unique_ptr<juce::FileChooser> fileChooser;
	std::unique_ptr<sofa::File> file;

	// IRs
	std::unique_ptr<float[]> rawIRs;
	size_t IRDataSamples; // number of samples contained in one IR measurement

	std::vector< IRMapping> IRMappingCollection;
};