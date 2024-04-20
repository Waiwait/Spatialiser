#include "SpatialiserController.h"

SpatialiserController::SpatialiserController()
{
}

SpatialiserController::~SpatialiserController()
{
}

void SpatialiserController::openSOFAFile()
{
    // Load sofa file asynchronously
	fileChooser = std::make_unique<juce::FileChooser>("Select a SOFA HRTF file to load...",
		juce::File{}, "*.SOFA");
	auto chooserFlags = juce::FileBrowserComponent::openMode
		| juce::FileBrowserComponent::canSelectFiles;

    fileChooser->launchAsync(chooserFlags, [this](const juce::FileChooser& fileChooser)
    {
        auto chosenFile = fileChooser.getResult();

        if (chosenFile != juce::File{})
        {
            file = std::make_unique<sofa::File>(chosenFile.getFullPathName().toStdString());

            // Store source positions and IRs
            if (file->IsFIRDataType())
            {
                std::vector< std::string > variableNames;
                file->GetAllVariablesNames( variableNames );

                file->GetValues(sourcePositions, "SourcePosition");
                file->GetValues(IRs, "Data.IR");
            }
        }
    });
}

void SpatialiserController::spatialise(juce::AudioBuffer<float>& buffer, float azi, float ele)
{
    if (file && file->IsFIRDataType())
    {
        // STEP 1 - Find closest IRS

        // STEP 2 INTERPOLATE

        // STEP 3 CONVOLVE
    }
}