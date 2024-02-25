#include "MainComponent.h"

//==============================================================================
MainComponent::MainComponent()
    : playbackState( Stopped )
{
    // Make sure you set the size of the component after
    // you add any child components.
    setSize (800, 600);

    // Some platforms require permissions to open input channels so request that here
    if (juce::RuntimePermissions::isRequired (juce::RuntimePermissions::recordAudio)
        && ! juce::RuntimePermissions::isGranted (juce::RuntimePermissions::recordAudio))
    {
        juce::RuntimePermissions::request (juce::RuntimePermissions::recordAudio,
                                           [&] (bool granted) { setAudioChannels (granted ? 2 : 0, 2); });
    }
    else
    {
        // Specify the number of input and output channels that we want to open
        setAudioChannels (2, 2);
    }

    // Add transport buttons
    addAndMakeVisible(&openButton);
    openButton.setButtonText("Open...");
    openButton.onClick = [this] { openAudioFile(); };

    addAndMakeVisible(&playButton);
    playButton.setButtonText("Play");
    playButton.onClick = [this] { startPlayback(); };
    playButton.setColour(juce::TextButton::buttonColourId, juce::Colours::green);
    playButton.setEnabled(false);

    addAndMakeVisible(&stopButton);
    stopButton.setButtonText("Stop");
    stopButton.onClick = [this] { stopPlayback(); };
    stopButton.setColour(juce::TextButton::buttonColourId, juce::Colours::red);
    stopButton.setEnabled(false);

    // Register basic audio file formats
    formatManager.registerBasicFormats();

    // Add change listener for transpor source
    transportSource.addChangeListener(this);
}

MainComponent::~MainComponent()
{
    // This shuts down the audio device and clears the audio source.
    shutdownAudio();
}

//==============================================================================
void MainComponent::prepareToPlay (int samplesPerBlockExpected, double sampleRate)
{
    // This function will be called when the audio device is started, or when
    // its settings (i.e. sample rate, block size, etc) are changed.

    // You can use this function to initialise any resources you might need,
    // but be careful - it will be called on the audio thread, not the GUI thread.

    transportSource.prepareToPlay(samplesPerBlockExpected, sampleRate);
}

void MainComponent::getNextAudioBlock (const juce::AudioSourceChannelInfo& bufferToFill)
{
    if (readerSource.get() == nullptr)
    {
        // Right now we are not producing any data, in which case we need to clear the buffer
        // (to prevent the output of random noise)
        bufferToFill.clearActiveBufferRegion();
        return;
    }

    transportSource.getNextAudioBlock(bufferToFill);
}

void MainComponent::releaseResources()
{
    // This will be called when the audio device stops, or when it is being
    // restarted due to a setting change.

    transportSource.releaseResources();
}

//==============================================================================
void MainComponent::paint (juce::Graphics& g)
{
    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));

    // You can add your drawing code here!
}

void MainComponent::resized()
{
    // This is called when the MainContentComponent is resized.
    // If you add any child components, this is where you should
    // update their positions.

    openButton.setBounds(10, 10, getWidth() - 20, 20);
    playButton.setBounds(10, 40, getWidth() - 20, 20);
    stopButton.setBounds(10, 70, getWidth() - 20, 20);
}

//==============================================================================
void MainComponent::openAudioFile()
{   
    // Open audio file asynchronously
    fileChooser = std::make_unique<juce::FileChooser>("Select a Wave file to play...", 
        juce::File{}, "*.wav");
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

                playButton.setEnabled(true);
            }
        }
    });
}

void MainComponent::changeListenerCallback(juce::ChangeBroadcaster* source)
{
    if (source == &transportSource)
    {
        if (transportSource.isPlaying())
            changePlaybackState(Playing);
        else
            changePlaybackState(Stopped);
    }
}

void MainComponent::changePlaybackState(TransportState newState)
{
    if (playbackState != newState)
    {
        playbackState = newState;
        switch (playbackState)
        {
            case Stopped:
                stopButton.setEnabled(false);
                playButton.setEnabled(true);
                transportSource.setPosition(0.0);
                break;

            case Starting:
                playButton.setEnabled(false);
                transportSource.start();
                break;

            case Playing:
                stopButton.setEnabled(true);
                break;

            case Stopping:
                transportSource.stop();
                break;
        }
    }
}

void MainComponent::startPlayback()
{
    changePlaybackState(Starting);
}

void MainComponent::stopPlayback()
{
    changePlaybackState(Stopping);
}