#include "SpatialiserController.h"

SpatialiserController::SpatialiserController()
{
}

SpatialiserController::~SpatialiserController()
{
}

void SpatialiserController::openSOFAFile()
{
    // Load SOFA file asynchronously
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

            // Store off azi/ele positions of sources and their IRS
            if (file->IsFIRDataType())
            {
                // number of measurements (M in documentation)
                const size_t numMeasurements = file->GetNumMeasurements();
                // number of receivers (typically 2, one for each ear. R in documentation)
                const size_t numReceivers = file->GetNumReceivers();
                // number of data samples describing one measurement (N in documentation)
                const size_t numDataSamples = file->GetNumDataSamples();

                // Get position of sources. Positions stored in following order: Azi, Ele, Distance.
                std::vector<double> sourcePositions;
                file->GetValues(sourcePositions, "SourcePosition");

                // Store off raw IRs as floats
                std::unique_ptr<double[]> dRawIRs(new double[numMeasurements * numReceivers * numDataSamples]);
                file->GetValues(dRawIRs.get(), numMeasurements, numReceivers, numDataSamples, "Data.IR");
                rawIRs = std::make_unique<float[]>(numMeasurements * numReceivers * numDataSamples);
                for (size_t idx = 0; idx < numMeasurements * numReceivers * numDataSamples; ++idx)
                {
                    rawIRs[idx] = static_cast<float>(dRawIRs[idx]);
                }

                // Map IRs with ele and azi
                for (std::size_t measurementIdx = 0; measurementIdx < numMeasurements; ++measurementIdx)
                {
                    // Ele and azi for current measurement
                    double azi = sourcePositions[measurementIdx * 3];
                    double ele = sourcePositions[(measurementIdx * 3) + 1];

                    // Get left and right IRs
                    float* leftIR = &rawIRs.get()[measurementIdx * numDataSamples * numReceivers];
                    float* rightIR = &rawIRs.get()[(measurementIdx * numDataSamples * numReceivers) + numDataSamples];
                    
                    // Create an IR mapping and add to our list
                    IRMapping irMapping;
                    irMapping.azi = azi;
                    irMapping.ele = ele;
                    irMapping.leftIR = leftIR;
                    irMapping.rightIR = rightIR;
                    IRMappingCollection.push_back(irMapping);
                }
            }
        }
    });
}

void SpatialiserController::spatialise(const juce::AudioSourceChannelInfo& bufferToFill, float inputAzi, float inputEle) const
{
    if (file && file->IsFIRDataType())
    {
        // STEP 1 - Find closest IRS

        // STEP 2 INTERPOLATE

        // STEP 3 CONVOLVE

        // STEP 4: ITD
    }
}