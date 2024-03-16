#pragma once

#include <JuceHeader.h>

#include "SOFA.h"

class SpatialiserController
{
public:
	SpatialiserController();
	~SpatialiserController();

	void openSOFAFile();

private:
	std::unique_ptr<juce::FileChooser> fileChooser;
	std::unique_ptr<sofa::File> file;
};