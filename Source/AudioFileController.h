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

    bool isProducingData() { return m_readerSource.get() != nullptr; }
    void getNextAudioBlock(const juce::AudioSourceChannelInfo& bufferToFill);

private:
    void changeListenerCallback(juce::ChangeBroadcaster* source) override;
    void changePlaybackState(TransportState newState);

    MainComponent* m_mainComponent;

    TransportState m_playbackState;

    std::unique_ptr<juce::FileChooser> m_fileChooser;
    juce::AudioFormatManager m_formatManager;
    std::unique_ptr<juce::AudioFormatReaderSource> m_readerSource;
    juce::AudioTransportSource m_transportSource;
};