#include "AudioFileController.h";

#include "MainComponent.h";

AudioFileController::AudioFileController(MainComponent* mainComponentArg)
    : mainComponent(mainComponentArg)
	, playbackState(Stopped)
{
    // Register basic audio file formats
    formatManager.registerBasicFormats();

    // Add change listener for transport source
    transportSource.addChangeListener(this);
}

AudioFileController::~AudioFileController()
{
}

void AudioFileController::prepareToPlay(int samplesPerBlockExpected, double sampleRate)
{
    transportSource.prepareToPlay(samplesPerBlockExpected, sampleRate);
}

void AudioFileController::releaseResources()
{
    transportSource.releaseResources();
}

void AudioFileController::openAudioFile()
{
    // Open audio file asynchronously
    fileChooser = std::make_unique<juce::FileChooser>("Select a Wave file to play...",
        juce::File{}, "*.wav;*.mp3");
    auto chooserFlags = juce::FileBrowserComponent::openMode
        | juce::FileBrowserComponent::canSelectFiles;

    fileChooser->launchAsync(chooserFlags, [this](const juce::FileChooser& fileChooser)
    {
        auto file = fileChooser.getResult();

        if (file != juce::File{})
        {
            auto* fileReader = formatManager.createReaderFor(file);

            if (fileReader != nullptr)
            {
                // Create a new reader source from our file reader
                auto newReaderSource =
                    std::make_unique<juce::AudioFormatReaderSource>(fileReader, true);

                // Set input source in our AudioTransportSource to this new reader source
                transportSource.setSource(newReaderSource.get(), 0, nullptr, fileReader->sampleRate);

                // Set our local reader source to be this newly created reader source
                readerSource.reset(newReaderSource.release());

                mainComponent->setPlayButtonEnabled(true);
            }
        }
    });
}

void AudioFileController::getNextAudioBlock(const juce::AudioSourceChannelInfo& bufferToFill)
{
    transportSource.getNextAudioBlock(bufferToFill);
}

void AudioFileController::changeListenerCallback(juce::ChangeBroadcaster* source)
{
    if (source == &transportSource)
    {
        if (transportSource.isPlaying())
            changePlaybackState(Playing);
        else
            changePlaybackState(Stopped);
    }
}

void AudioFileController::changePlaybackState(TransportState newState)
{
    if (playbackState != newState)
    {
        playbackState = newState;
        switch (playbackState)
        {
            case Stopped:
                mainComponent->setStopButtonEnabled(false);
                mainComponent->setPlayButtonEnabled(true);
                transportSource.setPosition(0.0);
                break;

            case Starting:
                mainComponent->setPlayButtonEnabled(false);
                transportSource.start();
                break;

            case Playing:
                mainComponent->setStopButtonEnabled(true);
                break;

            case Stopping:
                transportSource.stop();
                break;
        }
    }
}