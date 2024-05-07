#include "SpatialiserController.h"

SpatialiserController::SpatialiserController()
    : numSamplesPerBlock(0)
    , inputSampleRate(0)
    , IRNumSamples(0)
{
}

SpatialiserController::~SpatialiserController()
{
}

void SpatialiserController::prepareToPlay(int inputSamplesPerBlock, double sampleRate)
{
    numSamplesPerBlock = inputSamplesPerBlock;
    inputSampleRate = sampleRate;
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
                IRNumSamples = file->GetNumDataSamples();

                // Get position of sources. Positions stored in following order: Azi, Ele, Distance.
                std::vector<double> sourcePositions;
                file->GetValues(sourcePositions, "SourcePosition");

                // Store off raw IRs as floats
                std::unique_ptr<double[]> dRawIRs(new double[numMeasurements * numReceivers * IRNumSamples]);
                file->GetValues(dRawIRs.get(), numMeasurements, numReceivers, IRNumSamples, "Data.IR");
                rawIRs = std::make_unique<float[]>(numMeasurements * numReceivers * IRNumSamples);
                for (size_t idx = 0; idx < numMeasurements * numReceivers * IRNumSamples; ++idx)
                {
                    rawIRs[idx] = static_cast<float>(dRawIRs[idx]);
                }

                // Map IRs with ele and azi
                for (size_t measurementIdx = 0; measurementIdx < numMeasurements; ++measurementIdx)
                {
                    // Ele and azi for current measurement
                    double azi = sourcePositions[measurementIdx * 3];
                    double ele = sourcePositions[(measurementIdx * 3) + 1];

                    // Get left and right IRs
                    float* leftIR = &rawIRs.get()[measurementIdx * IRNumSamples * numReceivers];
                    float* rightIR = &rawIRs.get()[(measurementIdx * IRNumSamples * numReceivers) + IRNumSamples];
                    
                    // Create an IR mapping and add to our list
                    IRMapping irMapping;
                    irMapping.azi = azi;
                    irMapping.ele = ele;
                    irMapping.leftIR = leftIR;
                    irMapping.rightIR = rightIR;
                    IRMappingCollection.push_back(irMapping);
                }

                leftConvolveOutput = std::make_unique<float[]>(numSamplesPerBlock + IRNumSamples - 1);
                rightConvolveOutput = std::make_unique<float[]>(numSamplesPerBlock + IRNumSamples - 1);
            }
        }
    });
}

void SpatialiserController::spatialise(const juce::AudioSourceChannelInfo& bufferToFill, float inputAzi, float inputEle)
{
    if ( file && file->IsFIRDataType() )
    {
        // STEP 1 - Find closest IRS

        // STEP 2 INTERPOLATE

        // STEP 3 CONVOLVE
        convolve(bufferToFill.buffer->getWritePointer(0), bufferToFill.buffer->getWritePointer(1), 
            IRMappingCollection[0].leftIR, IRMappingCollection[0].rightIR);

        // STEP 4: ITD
    }
}

void SpatialiserController::convolve(float* leftSignal, float* rightSignal, float* leftIR, float* rightIR)
{
    // Erase first segment (size of input buffer) of our local convolver output buffer and shift data forward
    std::memcpy(&leftConvolveOutput.get()[0], &leftConvolveOutput.get()[numSamplesPerBlock], (IRNumSamples - 1) * sizeof(float));
    std::memset(&leftConvolveOutput.get()[IRNumSamples - 1], 0, numSamplesPerBlock * sizeof(float));
    std::memcpy(&rightConvolveOutput.get()[0], &rightConvolveOutput.get()[numSamplesPerBlock], (IRNumSamples - 1) * sizeof(float));
    std::memset(&rightConvolveOutput.get()[IRNumSamples - 1], 0, numSamplesPerBlock * sizeof(float));

    // Convolve into local buffer
    int outputNumSamples = numSamplesPerBlock + IRNumSamples - 1;
    for (int outIdx = 0; outIdx < outputNumSamples; ++outIdx) {
        for (int IRIdx = 0; IRIdx < IRNumSamples; ++IRIdx) {
            int inputIdx = outIdx - IRIdx;
            if (inputIdx >= 0 && inputIdx < numSamplesPerBlock) {
                leftConvolveOutput.get()[outIdx] += (leftSignal[inputIdx] * leftIR[IRIdx]);
                rightConvolveOutput.get()[outIdx] += (rightSignal[inputIdx] * rightIR[IRIdx]);
            }
        }
    }
    
    // Copy a buffers worth of data from local buffer into output buffer
    std::memcpy(leftSignal, &leftConvolveOutput.get()[0], numSamplesPerBlock * sizeof(float));
    std::memcpy(rightSignal, &rightConvolveOutput.get()[0], numSamplesPerBlock * sizeof(float));
}