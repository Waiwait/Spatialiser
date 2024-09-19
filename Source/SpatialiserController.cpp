#include "SpatialiserController.h"

#include "MainComponent.h";

const int RESAMPLED_HRTF_ANGLE_INTERVAL = 5;
const float HRTF_REL_ONSET_THRESHOLD = 0.31f;
const int FFT_SIZE = 2048;
const float FFT_HOP = 0.5f;
const int FFT_OVERLAP = FFT_SIZE * FFT_HOP;
const int FFT_BINS = log2(FFT_SIZE);

struct Triangle
{
    double m_azi1;
    double m_ele1;
    double m_azi2;
    double m_ele2;
    double m_azi3;
    double m_ele3;
};

bool compareDist(SpatialiserController::DistMapping& a, SpatialiserController::DistMapping& b)
{
    return (a.m_distance < b.m_distance);
}

double angDiff(double aDeg, double bDeg)
{
    // Calculate difference between angles in degrees, whilst making sure to wrap around
    double result = aDeg - bDeg;
    if (result > 180.0)
    {
        result -= 360.0;
    }
    else if (result < -180.0)
    {
        result += 360.0;
    }
    return result;
}

double calcSphericaldist(double aAzi, double aEle, double bAzi, double bEle, double radius)
{
    double circumference = 2.0 * juce::double_Pi * radius;

    double aAziRad = aAzi * juce::double_Pi / 180.0;
    double aEleRad = aEle * juce::double_Pi / 180.0;
    double bAziRad = bAzi * juce::double_Pi / 180.0;
    double bEleRad = bEle * juce::double_Pi / 180.0;
    double aziDiffRad = aAziRad - bAziRad;
    double eleDiffRad = aEleRad - bEleRad;

    // Haversine formula
    double a = juce::jmin(sin(eleDiffRad / 2.0) * sin(eleDiffRad / 2.0) +
        cos(aEleRad) * cos(bEleRad) *
        sin(aziDiffRad / 2.0) * sin(aziDiffRad / 2.0), 1.0);

    double c = 2 * atan2(sqrt(a), sqrt(1.0 - a));

    return c * radius;
}

bool approximatelyEqual(double a, double b, double c)
{
    return juce::approximatelyEqual(a, b) && juce::approximatelyEqual(b, c);
}

bool getBarycentricCoeffs(double tarAzi, double tarEle, Triangle& t, double& a, double& b, double& c)
{
    // I think this is actually slightly inaccurate. We're using barycentric interpolation as if this were a 2d plane
    // rather than on a sphere. However, this is what is used in 3DTI and it's effect is likely very insignificant.

    double denom = (angDiff(t.m_ele2, t.m_ele3) * angDiff(t.m_azi1, t.m_azi3)
        + angDiff(t.m_azi3, t.m_azi2) * angDiff(t.m_ele1, t.m_ele3));

    a = (angDiff(t.m_ele2, t.m_ele3) * angDiff(tarAzi, t.m_azi3) + angDiff(t.m_azi3, t.m_azi2) * angDiff(tarEle, t.m_ele3))
        / denom;
    b = (angDiff(t.m_ele3, t.m_ele1) * angDiff(tarAzi, t.m_azi3) + angDiff(t.m_azi1, t.m_azi3) * angDiff(tarEle, t.m_ele3))
        / denom;
    c = 1.0 - a - b;

    // We want the coefficient values to be between 0 and 1. If they are not, this is likely because of either -
    // 1. All the points are on the same plane (i.e. all on the same azi or ele plane)
    // 2. The triangle does not enclose the target point
    return(a >= 0.0 && a <= 1.0 && b >= 0.0 && b <= 1.0 && c >= 0.0 && c <= 1.0);
}

void interpolate(float* outputBuffer, size_t outputBufferNumSamples, const float* const inputBuffer, size_t inputBufferNumSamples)
{
    float scale = static_cast<float>(inputBufferNumSamples) / static_cast<float>(outputBufferNumSamples); // 20 / 30 = 0.6666
    for (int outputIdx = 0; outputIdx < outputBufferNumSamples; ++outputIdx)
    {
        float inputIdx = outputIdx * scale;
        size_t index0 = static_cast<size_t>(inputIdx);
        size_t index1 = juce::jmin(index0 + 1, inputBufferNumSamples - 1);
        float factor = inputIdx - static_cast<float>(index0);

        outputBuffer[outputIdx] = inputBuffer[index0] * (1 - factor) + inputBuffer[index1] * factor;
    }
}

SpatialiserController::SpatialiserController(MainComponent* mainComponentArg)
    : m_mainComponent(mainComponentArg)
    , loadHRTFDataThread(*this)
    , m_state(UNPREPARED)
    , m_numSamplesPerBlock(0)
    , m_inputSampleRate(0)
    , m_radius(0.0)
    , m_IRNumSamples(0)
    , m_delayLineSize(0)
    , m_samplesConvolved(0)
    , m_convolveOutputBufferSize(0)
{
    m_outputHRTF.m_azi = 0.0;
    m_outputHRTF.m_ele = 0.0;
    m_currITD[0] = 0;
    m_currITD[1] = 0;
    m_outputHRTF.m_ITD[0] = 0.0;
    m_outputHRTF.m_ITD[1] = 0.0;
    m_delayLineIdx[0] = 0;
    m_delayLineIdx[1] = 0;
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
    m_state = UNPREPARED;

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

            // Store off azi/ele positions of sources and their IRS off-thread
            if (file->IsFIRDataType())
            {
                m_sofaFIRFile = std::make_unique<sofa::GeneralFIR>(chosenFile.getFullPathName().toStdString());
                m_mainComponent->setConsoleText("Loading SOFA HRTF file (this may take a while!)");
                m_mainComponent->setLoadSOFAButtonLoading(true);
                loadHRTFDataThread.startThread();
            }
            else
            {
                m_mainComponent->setConsoleText("SOFA file does not use FIR data type!");
            }
        }
    });
}

void SpatialiserController::loadHRTFData()
{
    sofa::GeneralFIR& file = *m_sofaFIRFile.get();

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
    m_radius = sourcePositions[2];

    // Get raw IRs
    std::unique_ptr<double[]> rawIRs(new double[numMeasurements * numReceivers * m_IRNumSamples]);
    file.GetDataIR(rawIRs.get(), numMeasurements, numReceivers, m_IRNumSamples);

    // Go through each measurement in SOFA file and store them in our HRTF mapping list
    std::vector<HRTFMapping> rawHRTFMappingList;
    for (int measurementIdx = 0; measurementIdx < numMeasurements; ++measurementIdx)
    {
        HRTFMapping& hrtfMapping = rawHRTFMappingList.emplace_back();

        // Store azi and ele positions
        hrtfMapping.m_azi = sourcePositions[measurementIdx * 3];
        hrtfMapping.m_ele = sourcePositions[measurementIdx * 3 + 1];

        for (int receiverIdx = 0; receiverIdx < 2; ++receiverIdx)
        {
            // Store raw IRs as floats (since JUCE requires floats)
            hrtfMapping.m_IR[receiverIdx] = std::make_unique<float[]>(m_IRNumSamples);
            for (int sampleIdx = 0; sampleIdx < m_IRNumSamples; ++sampleIdx)
            {
                hrtfMapping.m_IR[receiverIdx][sampleIdx] = static_cast<float>(rawIRs[(measurementIdx * 2 + receiverIdx) *
                    m_IRNumSamples + sampleIdx]);
            }

            // Find peak val in IR
            float peakVal = 0.0f;
            for (int sampleIdx = 0; sampleIdx < m_IRNumSamples; ++sampleIdx)
            {
                if (hrtfMapping.m_IR[receiverIdx][sampleIdx] > peakVal)
                {
                    peakVal = hrtfMapping.m_IR[receiverIdx][sampleIdx];
                }
            }

            // Calculate IR onset threshold
            float onsetThreshold = HRTF_REL_ONSET_THRESHOLD * peakVal;

            // Find the ITD (first sample in IR that surpasses the threshold) and store this
            int ITD = 0;
            for (int sampleIdx = 0; sampleIdx < m_IRNumSamples; ++sampleIdx)
            {
                if (hrtfMapping.m_IR[receiverIdx][sampleIdx] > onsetThreshold)
                {
                    ITD = sampleIdx;
                    hrtfMapping.m_ITD[receiverIdx] = ITD;
                    break;
                }
                if (sampleIdx == m_IRNumSamples - 1)
                {
                    const juce::MessageManagerLock mmLock;
                    m_mainComponent->setConsoleText("Code error calculating ITD!");
                }
            }

            // Remove ITDs from IRs
            std::memcpy(&hrtfMapping.m_IR[receiverIdx][0], &hrtfMapping.m_IR[receiverIdx][ITD], (m_IRNumSamples - ITD) * sizeof(float));
            std::memset(&hrtfMapping.m_IR[receiverIdx][m_IRNumSamples - ITD], 0.0f, ITD * sizeof(float));
        }

        // For +-90 deg ele, SOFA files will have one measurement at 0 deg azi, since the azi doesn't matter at +90deg
        // ele. But this messes up our barycentric calculation later which doesn't account for spherical coordinates,
        // so throw in some dummy mappings at +-90deg ele at different azi positions.
        if (juce::approximatelyEqual(std::abs(hrtfMapping.m_ele), 90.0))
        {
            for (double azi = 5.0; azi < 360; azi += static_cast<double>(RESAMPLED_HRTF_ANGLE_INTERVAL))
            {
                HRTFMapping& hrtfMapping = rawHRTFMappingList.emplace_back();
                hrtfMapping.m_azi = azi;
                hrtfMapping.m_ele = rawHRTFMappingList[rawHRTFMappingList.size() - 2].m_ele;
                for (int receiverIdx = 0; receiverIdx < 2; ++receiverIdx)
                {
                    hrtfMapping.m_ITD[receiverIdx] = rawHRTFMappingList[rawHRTFMappingList.size() - 2].m_ITD[receiverIdx];
                    hrtfMapping.m_IR[receiverIdx] = std::make_unique<float[]>(m_IRNumSamples);
                    std::memcpy(&hrtfMapping.m_IR[receiverIdx][0], &rawHRTFMappingList[rawHRTFMappingList.size() - 2].m_IR[receiverIdx][0],
                        m_IRNumSamples * sizeof(float));
                }
            }
        }
    }

    m_HRTFMappingCollection.clear();
    // Interpolate HRTFs in 5 deg intervals and store this collection
    for (int tarAzi = 0; tarAzi < 360; tarAzi += RESAMPLED_HRTF_ANGLE_INTERVAL)
    {
        for (int tarEle = -90; tarEle <= 90; tarEle += RESAMPLED_HRTF_ANGLE_INTERVAL)
        {
            HRTFMapping& hrtfMapping = m_HRTFMappingCollection.emplace_back();
            hrtfMapping.m_azi = tarAzi;
            hrtfMapping.m_ele = tarEle;
            hrtfMapping.m_IR[0] = std::make_unique<float[]>(m_IRNumSamples);
            hrtfMapping.m_IR[1] = std::make_unique<float[]>(m_IRNumSamples);
            hrtfMapping.m_ITD[0] = 0;
            hrtfMapping.m_ITD[1] = 0;

            interpolateHRTF(hrtfMapping, rawHRTFMappingList);
        }
    }

    // We don't need as many elements when we use this for realtime interpolation, so reallocate the memory.
    m_distMappingList.shrink_to_fit();

    // Prep window and FFT controller
    m_windowController = std::make_unique<juce::dsp::WindowingFunction<float>>(FFT_SIZE,
        juce::dsp::WindowingFunction<float>::WindowingMethod::hann, false);
    m_fftController = std::make_unique<juce::dsp::FFT>(FFT_BINS);
    m_irFft = std::make_unique<float[]>(2 * FFT_SIZE);
    m_inputFft = std::make_unique<float[]>(2 * FFT_SIZE);

    // Delay line needs to accomodate a block, the maximum ITD (num IR samples) and the size of one FFT
    m_delayLineSize = m_numSamplesPerBlock + m_IRNumSamples + FFT_SIZE;
    // Convoler output buffer needs to accomodate a block, the maximum ITD (num IR samples) and the size of one FFT + an overlap
    m_convolveOutputBufferSize = m_numSamplesPerBlock + m_IRNumSamples + FFT_SIZE * 3; // Should this be hop * 3?

    for (int chan = 0; chan < 2; ++chan)
    {
        // Prep our delay line
        m_delayLine[chan] = std::make_unique<float[]>(m_delayLineSize);

        // Prep our convolution buffers
        m_convolveOutput[chan] = std::make_unique<float[]>(m_convolveOutputBufferSize);

        // Prep out final convolved output HRTF buffers
        m_outputHRTF.m_IR[chan] = std::make_unique<float[]>(m_IRNumSamples);
    }

    // Interpolate for our current azi and ele
    interpolateHRTF(m_outputHRTF, m_HRTFMappingCollection);

    m_state = PREPARED;

    const juce::MessageManagerLock mmLock;
    m_mainComponent->setConsoleText("SOFA HRTF file loaded!");
    m_mainComponent->setLoadSOFAButtonLoading(false);
}

void SpatialiserController::spatialise(const juce::AudioSourceChannelInfo& bufferToFill)
{
    if (m_state == PREPARED)
    {
        // Convolve
        convolve(*bufferToFill.buffer);
    }
}

void SpatialiserController::setPosition(double azi, double ele)
{
    m_outputHRTF.m_azi = azi;
    m_outputHRTF.m_ele = ele;

    if (m_state == PREPARED)
    {
        interpolateHRTF(m_outputHRTF, m_HRTFMappingCollection);
    }
}

void SpatialiserController::interpolateHRTF(HRTFMapping& targetHRTF, const std::vector< HRTFMapping >& hrtfMappingList)
{
    // Get distances between current target azi/ele and all HRTFs in list
    for (int i = 0; i < hrtfMappingList.size(); ++i)
    {
        DistMapping& distMapping = m_distMappingList.emplace_back();
        distMapping.m_mappingIdx = i;
        distMapping.m_azi = hrtfMappingList[i].m_azi;
        distMapping.m_ele = hrtfMappingList[i].m_ele;
        distMapping.m_distance = calcSphericaldist(targetHRTF.m_azi, targetHRTF.m_ele, hrtfMappingList[i].m_azi,
            hrtfMappingList[i].m_ele, m_radius);
    }

    // Sort our list of distance mappings by distance
    std::sort(m_distMappingList.begin(), m_distMappingList.end(), compareDist);

    // Calculate HRTF at our target position
    if (juce::approximatelyEqual(m_distMappingList[0].m_distance, 0.0) ||
        juce::approximatelyEqual(std::abs(targetHRTF.m_ele), 90.0))
    {
        // We have a HRTF at this position in our list, so just use this instead of interpolating
        int mappingIdx = m_distMappingList[0].m_mappingIdx;

        for (int receiverIdx = 0; receiverIdx < 2; ++receiverIdx)
        {
            std::memcpy(&targetHRTF.m_IR[receiverIdx][0], &hrtfMappingList[mappingIdx].m_IR[receiverIdx][0],
                m_IRNumSamples * sizeof(float));
            targetHRTF.m_ITD[receiverIdx] = hrtfMappingList[mappingIdx].m_ITD[receiverIdx];
        }
    }
    else if (approximatelyEqual(targetHRTF.m_azi, m_distMappingList[0].m_azi, m_distMappingList[1].m_azi)
        || approximatelyEqual(targetHRTF.m_ele, m_distMappingList[0].m_ele, m_distMappingList[1].m_ele))
    {
        // The target point lies on the same plane as the closest two points. So just linearly interpolate between these 
        // two points

        double factor = 0.0;
        if (approximatelyEqual(targetHRTF.m_azi, m_distMappingList[0].m_azi, m_distMappingList[1].m_azi))
        {
            // All points on azimuth plane, so calculate factor using elevation differences
            factor = std::abs(angDiff(targetHRTF.m_ele, m_distMappingList[0].m_ele) /
                angDiff(m_distMappingList[0].m_ele, m_distMappingList[1].m_ele));
        }
        else
        {
            // All points on elevation plane, so calculate factor using azimuth differences
            factor = std::abs(angDiff(targetHRTF.m_azi, m_distMappingList[0].m_azi) /
                angDiff(m_distMappingList[0].m_azi, m_distMappingList[1].m_azi));
        }

        for (int receiverIdx = 0; receiverIdx < 2; ++receiverIdx)
        {
            // Interpolate IRs
            for (int idx = 0; idx < m_IRNumSamples; ++idx)
            {
                targetHRTF.m_IR[receiverIdx][idx] = (1.0 - factor) * hrtfMappingList[m_distMappingList[0].m_mappingIdx].m_IR[receiverIdx][idx] +
                    factor * hrtfMappingList[m_distMappingList[1].m_mappingIdx].m_IR[receiverIdx][idx];
            }
            targetHRTF.m_ITD[receiverIdx] = juce::roundToInt((1.0 - factor) * hrtfMappingList[m_distMappingList[0].m_mappingIdx].m_ITD[receiverIdx] +
                factor * hrtfMappingList[m_distMappingList[1].m_mappingIdx].m_ITD[receiverIdx]);
        }
    }
    else
    {
        // Barycentric interpolate the IR between 3 closest measurements

        // Barycentric coefficients
        double a = 0.0f;
        double b = 0.0f;
        double c = 0.0f;

        int distMapping1Idx = 0;
        int distMapping2Idx = 1;
        int distMapping3Idx = 2;

        bool done = false;
        while (!done)
        {
            // Make sure three closest HRTFs to the target coordinate enclose the target position in a triangle.
            int hrtfIdx1 = m_distMappingList[distMapping1Idx].m_mappingIdx;
            double azi1 = m_distMappingList[distMapping1Idx].m_azi;
            double ele1 = m_distMappingList[distMapping1Idx].m_ele;
            int hrtfIdx2 = m_distMappingList[distMapping2Idx].m_mappingIdx;
            double azi2 = m_distMappingList[distMapping2Idx].m_azi;
            double ele2 = m_distMappingList[distMapping2Idx].m_ele;
            int hrtfIdx3 = m_distMappingList[distMapping3Idx].m_mappingIdx;
            double azi3 = m_distMappingList[distMapping3Idx].m_azi;
            double ele3 = m_distMappingList[distMapping3Idx].m_ele;
            Triangle triangle{ azi1, ele1, azi2, ele2, azi3, ele3 };

            if (!getBarycentricCoeffs(targetHRTF.m_azi, targetHRTF.m_ele, triangle, a, b, c))
            {
                ++distMapping3Idx;
                if (distMapping3Idx >= hrtfMappingList.size())
                {
                    ++distMapping2Idx;
                    distMapping3Idx = distMapping2Idx + 1;
                }
                if (distMapping2Idx >= hrtfMappingList.size() - 1)
                {
                    ++distMapping1Idx;
                    distMapping2Idx = distMapping1Idx + 1;
                    distMapping3Idx = distMapping2Idx + 1;
                }
                if (distMapping1Idx >= hrtfMappingList.size() - 2)
                {
                    const juce::MessageManagerLock mmLock;
                    m_mainComponent->setConsoleText("Code error interpolating HRTF!");
                }
            }
            else
            {
                for (int receiverIdx = 0; receiverIdx < 2; ++receiverIdx)
                {
                    for (int idx = 0; idx < m_IRNumSamples; ++idx)
                    {
                        targetHRTF.m_IR[receiverIdx][idx] = a * hrtfMappingList[hrtfIdx1].m_IR[receiverIdx][idx] +
                            b * hrtfMappingList[hrtfIdx2].m_IR[receiverIdx][idx] +
                            c * hrtfMappingList[hrtfIdx3].m_IR[receiverIdx][idx];
                    }
                    targetHRTF.m_ITD[receiverIdx] = a * hrtfMappingList[hrtfIdx1].m_ITD[receiverIdx] +
                        b * hrtfMappingList[hrtfIdx2].m_ITD[receiverIdx] +
                        c * hrtfMappingList[hrtfIdx3].m_ITD[receiverIdx];
                }
                done = true;
            }
        }
    }
    m_distMappingList.clear();
}

void SpatialiserController::convolve(juce::AudioSampleBuffer& buffer)
{
    // Move samples in delay line
    for (int chan = 0; chan < 2; ++chan)
    {
        int delayLineIdx = m_delayLineIdx[chan];
        if (m_currITD[chan] != m_outputHRTF.m_ITD[chan])
        {
            // If ITD has changed, interpolate our buffer (stretch/shrink) to match new ITD
            int itdDiff = m_outputHRTF.m_ITD[chan] - m_currITD[chan];
            interpolate(&m_delayLine[chan][delayLineIdx], buffer.getNumSamples() + itdDiff, buffer.getWritePointer(chan),
                buffer.getNumSamples());

            m_currITD[chan] = m_outputHRTF.m_ITD[chan];
            m_delayLineIdx[chan] += buffer.getNumSamples() + itdDiff;
        }
        else
        {
            // No ITD change, so just copy samples from buffer
            std::memcpy(&m_delayLine[chan][delayLineIdx], buffer.getReadPointer(chan), m_numSamplesPerBlock * sizeof(float));
            m_delayLineIdx[chan] += buffer.getNumSamples();
        }
    }

    // If there are enough samples in our delay line, perform fft
    while (juce::jmin(m_delayLineIdx[0], m_delayLineIdx[1]) >= FFT_SIZE)
    {
        for (int chan = 0; chan < 2; ++chan)
        {
            // Get windowed fft for IR
            std::memset(&m_irFft[0], 0, FFT_SIZE * 2 * sizeof(float));
            std::memcpy(&m_irFft[0], &m_outputHRTF.m_IR[chan][0], m_IRNumSamples * sizeof(float));
            m_fftController.get()->performRealOnlyForwardTransform(&m_irFft[0]);
            
            // Get windowed fft for input buffer
            std::memset(&m_inputFft[0], 0, FFT_SIZE * 2 * sizeof(float));
            std::memcpy(&m_inputFft[0], &m_delayLine[chan][0],
                FFT_SIZE * sizeof(float));
            m_windowController->multiplyWithWindowingTable(&m_inputFft[0], FFT_SIZE);
            m_fftController.get()->performRealOnlyForwardTransform(&m_inputFft[0]);

            // Convolve IR fft with input bufer fft (point wise multiplication)
            for (int i = 0; i < FFT_SIZE * 2; i += 2)
            {
                float real_part = m_inputFft[i] * m_irFft[i] - m_inputFft[i + 1] * m_irFft[i + 1];
                float imag_part = m_inputFft[i] * m_irFft[i + 1] + m_inputFft[i + 1] * m_irFft[i];
                m_inputFft[i] = real_part;
                m_inputFft[i+1] = imag_part;
            }
            
            // Inverse fft into time domain
            m_fftController.get()->performRealOnlyInverseTransform(&m_inputFft[0]);

            // Add convolved signal to our local buffer
            for (int i = 0; i < FFT_SIZE*2; ++i)
            {
                m_convolveOutput[chan][m_samplesConvolved + i] += m_inputFft[i];
            }

            // Move delay line forward
            std::memcpy(&m_delayLine[chan][0], &m_delayLine[chan][FFT_OVERLAP], (m_delayLineSize - FFT_OVERLAP) * sizeof(float));
            m_delayLineIdx[chan] -= FFT_OVERLAP;
        }
        m_samplesConvolved += FFT_OVERLAP;
    }

    // If we have enough samples convolved, copy the samples into the buffer
    if (m_samplesConvolved >= m_numSamplesPerBlock)
    {
        for (int chan = 0; chan < 2; ++chan)
        {
            std::memcpy(buffer.getWritePointer(chan), &m_convolveOutput[chan][0], m_numSamplesPerBlock * sizeof(float));

            // Moce convolve buffer forward
            std::memcpy(&m_convolveOutput[chan][0], &m_convolveOutput[chan][m_numSamplesPerBlock], 
                (m_convolveOutputBufferSize - m_numSamplesPerBlock) * sizeof(float));
            std::memset(&m_convolveOutput[chan][m_convolveOutputBufferSize - m_numSamplesPerBlock], 0, 
                m_numSamplesPerBlock * sizeof(float));
        }
        m_samplesConvolved -= m_numSamplesPerBlock;
    }
    else
    {
        for (int chan = 0; chan < 2; ++chan)
        {
            std::memset(buffer.getWritePointer(chan), 0, m_numSamplesPerBlock * sizeof(float));
        }
    }
}