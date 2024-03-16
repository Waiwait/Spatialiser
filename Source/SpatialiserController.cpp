#include "SpatialiserController.h"

SpatialiserController::SpatialiserController()
{
}

SpatialiserController::~SpatialiserController()
{
}

void SpatialiserController::openSOFAFile()
{
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
        }
    });

}