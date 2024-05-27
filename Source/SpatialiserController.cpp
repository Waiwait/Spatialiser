#include "SpatialiserController.h"

SpatialiserController::SpatialiserController()
    : m_state(UNPREPARED)
    , m_numSamplesPerBlock(0)
    , m_inputSampleRate(0)
    , m_IRNumSamples(0)
{
}

SpatialiserController::~SpatialiserController()
{
}

void SpatialiserController::prepareToPlay(int inputSamplesPerBlock, double sampleRate)
{
    m_numSamplesPerBlock = inputSamplesPerBlock;
    m_inputSampleRate = sampleRate;
}

void SpatialiserController::openSOFAFile()
{
    // Load SOFA file asynchronously
    m_fileChooser = std::make_unique<juce::FileChooser>("Select a SOFA HRTF file to load...",
		juce::File{}, "*.SOFA");
	auto chooserFlags = juce::FileBrowserComponent::openMode
		| juce::FileBrowserComponent::canSelectFiles;

    m_fileChooser->launchAsync(chooserFlags, [this](const juce::FileChooser& fileChooser)
    {
        auto chosenFile = fileChooser.getResult();
        if (chosenFile != juce::File{})
        {
            std::unique_ptr<sofa::File> file = std::make_unique<sofa::File>(chosenFile.getFullPathName().toStdString());

            // Store off azi/ele positions of sources and their IRS
            if (file->IsFIRDataType())
            {
                // number of measurements (M in documentation)
                const size_t numMeasurements = file->GetNumMeasurements();
                // number of receivers (typically 2, one for each ear. R in documentation)
                const size_t numReceivers = file->GetNumReceivers();
                // number of data samples describing one measurement (N in documentation)
                m_IRNumSamples = file->GetNumDataSamples();

                // Get position of sources. Positions stored in following order: Azi, Ele, Distance.
                std::vector<double> sourcePositions;
                file->GetValues(sourcePositions, "SourcePosition");

                // Store off raw IRs as floats
                std::unique_ptr<double[]> dRawIRs(new double[numMeasurements * numReceivers * m_IRNumSamples]);
                file->GetValues(dRawIRs.get(), numMeasurements, numReceivers, m_IRNumSamples, "Data.IR");
                m_rawIRs = std::make_unique<float[]>(numMeasurements * numReceivers * m_IRNumSamples);
                for (size_t idx = 0; idx < numMeasurements * numReceivers * m_IRNumSamples; ++idx)
                {
                    m_rawIRs[idx] = static_cast<float>(dRawIRs[idx]);
                }

                // Map IRs with ele and azi
                for (size_t measurementIdx = 0; measurementIdx < numMeasurements; ++measurementIdx)
                {
                    // Ele and azi for current measurement
                    double azi = sourcePositions[measurementIdx * 3];
                    double ele = sourcePositions[(measurementIdx * 3) + 1];

                    // Get left and right IRs
                    float* leftIR = &m_rawIRs[measurementIdx * m_IRNumSamples * numReceivers];
                    float* rightIR = &m_rawIRs[(measurementIdx * m_IRNumSamples * numReceivers) + m_IRNumSamples];
                    
                    // Create an IR mapping and add to our list
                    IRMapping irMapping;
                    irMapping.m_azi = azi;
                    irMapping.m_ele = ele;
                    irMapping.m_leftIR = leftIR;
                    irMapping.m_rightIR = rightIR;
                    m_IRMappingCollection.push_back(irMapping);
                }

                m_leftConvolveOutput = std::make_unique<float[]>(m_numSamplesPerBlock + m_IRNumSamples - 1);
                m_rightConvolveOutput = std::make_unique<float[]>(m_numSamplesPerBlock + m_IRNumSamples - 1);

                m_state = PREPARED;
            }
        }
    });
}

void SpatialiserController::spatialise(const juce::AudioSourceChannelInfo& bufferToFill, float inputAzi, float inputEle)
{
    if (m_state == PREPARED)
    {
        // STEP 1 - Find closest IRS

        // STEP 2 INTERPOLATE

        // STEP 3 CONVOLVE
        convolve(bufferToFill.buffer->getWritePointer(0), bufferToFill.buffer->getWritePointer(1), 
            m_IRMappingCollection[0].m_leftIR, m_IRMappingCollection[0].m_rightIR);

        // STEP 4: ITD
    }
}

void SpatialiserController::convolve(float* leftSignal, float* rightSignal, float* leftIR, float* rightIR)
{
    // Erase first segment (size of input buffer) of our local convolver output buffer and shift data forward
    std::memcpy(&m_leftConvolveOutput[0], &m_leftConvolveOutput[m_numSamplesPerBlock], (m_IRNumSamples - 1) * sizeof(float));
    std::memset(&m_leftConvolveOutput[m_IRNumSamples - 1], 0, m_numSamplesPerBlock * sizeof(float));
    std::memcpy(&m_rightConvolveOutput[0], &m_rightConvolveOutput[m_numSamplesPerBlock], (m_IRNumSamples - 1) * sizeof(float));
    std::memset(&m_rightConvolveOutput[m_IRNumSamples - 1], 0, m_numSamplesPerBlock * sizeof(float));

    // Convolve into local buffer
    int outputNumSamples = m_numSamplesPerBlock + m_IRNumSamples - 1;
    for (int outIdx = 0; outIdx < outputNumSamples; ++outIdx)
    {
        for (int IRIdx = 0; IRIdx < m_IRNumSamples; ++IRIdx)
        {
            int inputIdx = outIdx - IRIdx;
            if (inputIdx >= 0 && inputIdx < m_numSamplesPerBlock)
            {
                m_leftConvolveOutput[outIdx] += (leftSignal[inputIdx] * leftIR[IRIdx]);
                m_rightConvolveOutput[outIdx] += (rightSignal[inputIdx] * rightIR[IRIdx]);
            }
        }
    }
    
    // Copy a buffers worth of data from local buffer into output buffer
    std::memcpy(leftSignal, &m_leftConvolveOutput[0], m_numSamplesPerBlock * sizeof(float));
    std::memcpy(rightSignal, &m_rightConvolveOutput[0], m_numSamplesPerBlock * sizeof(float));
}