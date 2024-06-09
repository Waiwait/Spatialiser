#include "SpatialiserController.h"

const int RESAMPLED_HRTF_ANGLE_INTERVAL = 5;

struct DistMapping
{
    int m_measurementIdx;
    double m_distance;
    double m_aziRad;
    double m_eleRad;
};

bool compareDist(DistMapping& a, DistMapping& b)
{
    return (a.m_distance < b.m_distance);
}

bool checkIfOnSamePlane(double azi1, double ele1, double azi2, double ele2, double azi3, double ele3)
{
    return (juce::approximatelyEqual(azi1, azi2) && juce::approximatelyEqual(azi2, azi3))
        || (juce::approximatelyEqual(ele1, ele2) && juce::approximatelyEqual(ele2, ele3));
}

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
                std::unique_ptr<sofa::GeneralFIR> firFile = std::make_unique<sofa::GeneralFIR>(chosenFile.getFullPathName().toStdString());
                // TODO: Do this off-thread and report when done
                loadHRTFData(*firFile.get());
            }
        }
    });
}

void SpatialiserController::loadHRTFData(sofa::GeneralFIR& file)
{
    // number of measurements (M in documentation)
    const long numMeasurements = file.GetNumMeasurements();
    // number of receivers (typically 2, one for each ear. R in documentation)
    const long numReceivers = file.GetNumReceivers();
    // number of data samples describing one measurement (N in documentation)
    m_IRNumSamples = file.GetNumDataSamples();

    // Get position of sources. Positions stored in following order: Azi, Ele, Distance.
    std::vector<double> sourcePositions;
    file.GetSourcePosition(sourcePositions);

    // Store off distance of measurement (assume that distance is the same for all measurements)
    double radius = sourcePositions[2];
    double circumference = 2.0 * juce::double_Pi * radius;

    // Store raw IRs as floats (since JUCE requires in floats)
    std::unique_ptr<double[]> dRawIRs(new double[numMeasurements * numReceivers * m_IRNumSamples]);
    std::unique_ptr<float[]> fRawIRs(new float[numMeasurements * numReceivers * m_IRNumSamples]);
    file.GetDataIR(dRawIRs.get(), numMeasurements, numReceivers, m_IRNumSamples);
    for (int idx = 0; idx < numMeasurements * numReceivers * m_IRNumSamples; ++idx)
    {
        fRawIRs[idx] = static_cast<float>(dRawIRs[idx]);
    }

    std::vector<DistMapping> distMappingList;

    // Interpolate HRTFs in 5 deg intervals
    for (int tarAzi = -180; tarAzi < 180; tarAzi += RESAMPLED_HRTF_ANGLE_INTERVAL)
    {
        for (int tarEle = -180; tarEle < 180; tarEle += RESAMPLED_HRTF_ANGLE_INTERVAL)
        {
            double tarAziRad = static_cast<double>(tarAzi) * juce::double_Pi / 180.0;
            double tarEleRad = static_cast<double>(tarEle) * juce::double_Pi / 180.0;

            // Get distances between current target azi/ele and all measurements in sofa file
            std::vector<DistMapping> distMappingList;
            for (int measurementIdx = 0; measurementIdx < numMeasurements; ++measurementIdx)
            {
                // Ele and azi for current measurement in SOFA file
                double azi = sourcePositions[measurementIdx * 3];
                double ele = sourcePositions[(measurementIdx * 3) + 1];

                double aziDiffRad = (static_cast<double>(tarAzi) - azi) * juce::double_Pi / 180.0;
                double eleDiffRad = (static_cast<double>(tarEle) - ele) * juce::double_Pi / 180.0;
                double aziRad = azi * juce::double_Pi / 180.0;
                double eleRad = ele * juce::double_Pi / 180.0;

                // Haversine fomula
                double dist = 2.0 * radius * std::asin(std::sqrt((1 - std::cos(aziDiffRad) + std::cos(aziRad) *
                    std::cosf(tarAziRad) * (1.0 - std::cos(eleDiffRad))) / 2.0));

                while (dist > circumference / 2.0)
                {
                    dist -= circumference / 2.0;
                }

                DistMapping distMapping{ measurementIdx, dist, aziRad, eleRad };
                distMappingList.push_back(distMapping);
            }

            // Sort mapping by distances
            std::sort(distMappingList.begin(), distMappingList.end(), compareDist);

            // Get target HRTF from interpolation
            std::shared_ptr<float[]> interpolatedLeftIR(new float[m_IRNumSamples]);
            std::shared_ptr<float[]> interpolatedRightIR(new float[m_IRNumSamples]);

            if (juce::approximatelyEqual(distMappingList[0].m_distance, (double)0.0))
            {
                // We have a source HRTF at this position, so just use this instead of interpolating
                float* leftIR = &fRawIRs[distMappingList[0].m_measurementIdx * m_IRNumSamples * numReceivers];
                float* rightIR = &fRawIRs[(distMappingList[0].m_measurementIdx * m_IRNumSamples * numReceivers) + m_IRNumSamples];

                std::memcpy(interpolatedLeftIR.get(), leftIR, m_IRNumSamples * sizeof(float));
                std::memcpy(interpolatedRightIR.get(), rightIR, m_IRNumSamples * sizeof(float));
            }
            else
            {
                // Barycentric interpolate the IR between 3 closest measurements
                float* leftIR1 = &fRawIRs[distMappingList[0].m_measurementIdx * m_IRNumSamples * numReceivers];
                float* rightIR1 = &fRawIRs[(distMappingList[0].m_measurementIdx * m_IRNumSamples * numReceivers) + m_IRNumSamples];
                double aziRad1 = distMappingList[0].m_aziRad;
                double eleRad1 = distMappingList[0].m_eleRad;

                float* leftIR2 = &fRawIRs[distMappingList[1].m_measurementIdx * m_IRNumSamples * numReceivers];
                float* rightIR2 = &fRawIRs[(distMappingList[1].m_measurementIdx * m_IRNumSamples * numReceivers) + m_IRNumSamples];
                double aziRad2 = distMappingList[1].m_aziRad;
                double eleRad2 = distMappingList[1].m_eleRad;

                int IR3Idx = 2;
                while (checkIfOnSamePlane(distMappingList[0].m_aziRad, distMappingList[0].m_eleRad,
                    distMappingList[1].m_aziRad, distMappingList[1].m_eleRad,
                    distMappingList[IR3Idx].m_aziRad, distMappingList[IR3Idx].m_eleRad))
                {
                    // The three closest HRTFs to the target coordinate are all on the same plane. So find 
                    // the next HRTF that isn't for the third one
                    ++IR3Idx;
                    if (IR3Idx >= numMeasurements)
                    {
                        // TODO: Report error and stop
                    }
                }

                float* leftIR3 = &fRawIRs[distMappingList[IR3Idx].m_measurementIdx * m_IRNumSamples * numReceivers];
                float* rightIR3 = &fRawIRs[(distMappingList[IR3Idx].m_measurementIdx * m_IRNumSamples * numReceivers) + m_IRNumSamples];
                double aziRad3 = distMappingList[IR3Idx].m_aziRad;
                double eleRad3 = distMappingList[IR3Idx].m_eleRad;

                double a = ((aziRad2 - aziRad3) * (tarEleRad - eleRad3) + (eleRad3 - eleRad2) * (tarAziRad - aziRad3))
                    / ((aziRad2 - aziRad3) * (eleRad1 - eleRad3) + (eleRad3 - eleRad2) * (aziRad1 - aziRad3));
                double b = ((aziRad3 - aziRad1) * (tarEleRad - eleRad3) + (eleRad1 - eleRad3) * (tarAziRad - aziRad3))
                    / ((aziRad2 - aziRad3) * (eleRad1 - eleRad3) + (eleRad3 - eleRad2) * (aziRad1 - aziRad3));
                double c = 1.0 - a - b;

                for (int i = 0; i < m_IRNumSamples; ++i)
                {
                    interpolatedLeftIR[i] = a * leftIR1[i] + b * leftIR2[i] + c * leftIR3[i];
                    interpolatedRightIR[i] = a * rightIR1[i] + b * rightIR2[i] + c * rightIR3[i];
                }
            }

            IRMapping irMapping;
            irMapping.m_azi = tarAzi;
            irMapping.m_ele = tarEle;
            irMapping.m_leftIR = interpolatedLeftIR;
            irMapping.m_rightIR = interpolatedRightIR;
            m_IRMappingCollection.push_back(irMapping);
        }
    }

    m_leftConvolveOutput = std::make_unique<float[]>(m_numSamplesPerBlock + m_IRNumSamples - 1);
    m_rightConvolveOutput = std::make_unique<float[]>(m_numSamplesPerBlock + m_IRNumSamples - 1);

    m_state = PREPARED;
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

void SpatialiserController::convolve(float* leftSignal, float* rightSignal, std::shared_ptr<float[]> leftIR, std::shared_ptr<float[]> rightIR)
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