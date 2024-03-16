#pragma once

#include <JuceHeader.h>

#include "AudioFileController.h"
#include "SpatialiserController.h"

//==============================================================================
/*
    This component lives inside our window, and this is where you should put all
    your controls and content.
*/

class MainComponent : public juce::AudioAppComponent
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
    void setPlayButtonEnabled(bool enable) { playButton.setEnabled(enable); }
    void setStopButtonEnabled(bool enable) { stopButton.setEnabled(enable); }

private:
    //==============================================================================
    void openButtonPressed() { audioFileController.openAudioFile(); }
    void playButtonPressed() { audioFileController.startPlayback(); }
    void stopButtonPressed() { audioFileController.stopPlayback(); }

    AudioFileController audioFileController;
    SpatialiserController spatialiserController;

    juce::TextButton openSofaFileButton;
    juce::TextButton openAudioFileButton;
    juce::TextButton playButton;
    juce::TextButton stopButton;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
