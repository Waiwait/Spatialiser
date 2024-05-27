#include "AudioFileController.h";

#include "MainComponent.h";

AudioFileController::AudioFileController(MainComponent* mainComponent)
    : m_mainComponent(mainComponent)
	, m_playbackState(Stopped)
{
    // Register basic audio file formats
    m_formatManager.registerBasicFormats();

    // Add change listener for transport source
    m_transportSource.addChangeListener(this);
}

AudioFileController::~AudioFileController()
{
}

void AudioFileController::prepareToPlay(int samplesPerBlockExpected, double sampleRate)
{
    m_transportSource.prepareToPlay(samplesPerBlockExpected, sampleRate);
}

void AudioFileController::releaseResources()
{
    m_transportSource.releaseResources();
}

void AudioFileController::openAudioFile()
{
    // Open audio file asynchronously
    m_fileChooser = std::make_unique<juce::FileChooser>("Select a Wave file to play...",
        juce::File{}, "*.wav;*.mp3");
    auto chooserFlags = juce::FileBrowserComponent::openMode
        | juce::FileBrowserComponent::canSelectFiles;

    m_fileChooser->launchAsync(chooserFlags, [this](const juce::FileChooser& fileChooser)
    {
        auto file = fileChooser.getResult();

        if (file != juce::File{})
        {
            auto* fileReader = m_formatManager.createReaderFor(file);

            if (fileReader != nullptr)
            {
                // Create a new reader source from our file reader
                auto newReaderSource =
                    std::make_unique<juce::AudioFormatReaderSource>(fileReader, true);

                // Set input source in our AudioTransportSource to this new reader source
                m_transportSource.setSource(newReaderSource.get(), 0, nullptr, fileReader->sampleRate);

                // Set our local reader source to be this newly created reader source
                m_readerSource.reset(newReaderSource.release());

                m_mainComponent->setPlayButtonEnabled(true);
            }
        }
    });
}

void AudioFileController::getNextAudioBlock(const juce::AudioSourceChannelInfo& bufferToFill)
{
    m_transportSource.getNextAudioBlock(bufferToFill);
}

void AudioFileController::changeListenerCallback(juce::ChangeBroadcaster* source)
{
    if (source == &m_transportSource)
    {
        if (m_transportSource.isPlaying())
            changePlaybackState(Playing);
        else
            changePlaybackState(Stopped);
    }
}

void AudioFileController::changePlaybackState(TransportState newState)
{
    if (m_playbackState != newState)
    {
        m_playbackState = newState;
        switch (m_playbackState)
        {
            case Stopped:
                m_mainComponent->setStopButtonEnabled(false);
                m_mainComponent->setPlayButtonEnabled(true);
                m_transportSource.setPosition(0.0);
                break;

            case Starting:
                m_mainComponent->setPlayButtonEnabled(false);
                m_transportSource.start();
                break;

            case Playing:
                m_mainComponent->setStopButtonEnabled(true);
                break;

            case Stopping:
                m_transportSource.stop();
                break;
        }
    }
}