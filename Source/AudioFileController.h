#pragma once

#include <JuceHeader.h>

class MainComponent;

class AudioFileController : public juce::ChangeListener
{
public:
    enum TransportState
    {
        Stopped,
        Starting,
        Playing,
        Stopping
    };

    //==============================================================================
    explicit AudioFileController(MainComponent* mainComponentArg);
    ~AudioFileController() override;

    void prepareToPlay(int samplesPerBlockExpected, double sampleRate);
    void releaseResources();

    void openAudioFile();
    void startPlayback(){ changePlaybackState(Starting); }
    void stopPlayback(){ changePlaybackState(Stopping); }

    bool isProducingData() { return readerSource.get() != nullptr; }
    void getNextAudioBlock(const juce::AudioSourceChannelInfo& bufferToFill);

private:
    void changeListenerCallback(juce::ChangeBroadcaster* source) override;
    void changePlaybackState(TransportState newState);

    MainComponent* mainComponent;

    TransportState playbackState;

    std::unique_ptr<juce::FileChooser> fileChooser;
    juce::AudioFormatManager formatManager;
    std::unique_ptr<juce::AudioFormatReaderSource> readerSource;
    juce::AudioTransportSource transportSource;
};