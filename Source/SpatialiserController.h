#pragma once

#include <JuceHeader.h>

#include "SOFA.h"

class SpatialiserController
{
public:
	SpatialiserController();
	~SpatialiserController();

	void openSOFAFile();
	void spatialise(juce::AudioBuffer<float>& buffer, float azi, float ele);

private:
	std::unique_ptr<juce::FileChooser> fileChooser;
	std::unique_ptr<sofa::File> file;
	std::vector<double> sourcePositions;
	std::vector<double> IRs;
};