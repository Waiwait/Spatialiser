#pragma once

#include <JuceHeader.h>

//==============================================================================
/*
    This component lives inside our window, and this is where you should put all
    your controls and content.
*/

enum TransportState
{
    Stopped,
    Starting,
    Playing,
    Stopping
};

class MainComponent : public juce::AudioAppComponent
                    , public juce::ChangeListener
{
public:
    //==============================================================================
    MainComponent();
    ~MainComponent() override;

    //==============================================================================
    void prepareToPlay (int samplesPerBlockExpected, double sampleRate) override;
    void getNextAudioBlock (const juce::AudioSourceChannelInfo& bufferToFill) override;
    void releaseResources() override;

    //==============================================================================
    void paint (juce::Graphics& g) override;
    void resized() override;

    //==============================================================================
    void openAudioFile();
    void changeListenerCallback(juce::ChangeBroadcaster* source) override;
    void changePlaybackState(TransportState newState);
    void startPlayback();
    void stopPlayback();

private:
    //==============================================================================
    juce::TextButton openButton;
    juce::TextButton playButton;
    juce::TextButton stopButton;
    TransportState playbackState;

    std::unique_ptr<juce::FileChooser> fileChooser;
    juce::AudioFormatManager formatManager;
    std::unique_ptr<juce::AudioFormatReaderSource> readerSource;
    juce::AudioTransportSource transportSource;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
