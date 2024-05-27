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
    void setPlayButtonEnabled(bool enable) { m_playButton.setEnabled(enable); }
    void setStopButtonEnabled(bool enable) { m_stopButton.setEnabled(enable); }

private:
    //==============================================================================
    void openButtonPressed() { m_audioFileController.openAudioFile(); }
    void playButtonPressed() { m_audioFileController.startPlayback(); }
    void stopButtonPressed() { m_audioFileController.stopPlayback(); }

    AudioFileController m_audioFileController;
    SpatialiserController m_spatialiserController;

    juce::TextButton m_openSofaFileButton;
    juce::TextButton m_openAudioFileButton;
    juce::TextButton m_playButton;
    juce::TextButton m_stopButton;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (MainComponent)
};
