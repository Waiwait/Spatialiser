#include "MainComponent.h"

//==============================================================================
MainComponent::MainComponent()
    : m_audioFileController( this )
    , m_spatialiserController( this )
    , m_aziDial(juce::Slider::Rotary, juce::Slider::TextBoxBelow)
    , m_eleSlider(juce::Slider::LinearVertical, juce::Slider::TextBoxBelow)
{
    // Make sure you set the size of the component after
    // you add any child components.
    setSize (400, 400);

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
    addAndMakeVisible(&m_openSofaFileButton);
    m_openSofaFileButton.setButtonText("Open SOFA HRTF File");
    m_openSofaFileButton.onClick = [this] { m_spatialiserController.openSOFAFile(); };

    addAndMakeVisible(&m_openAudioFileButton);
    m_openAudioFileButton.setButtonText("Open Audio File");
    m_openAudioFileButton.onClick = [this] { m_audioFileController.openAudioFile(); };

    addAndMakeVisible(&m_playButton);
    m_playButton.setButtonText("Play");
    m_playButton.onClick = [this] { m_audioFileController.startPlayback(); };
    m_playButton.setColour(juce::TextButton::buttonColourId, juce::Colours::green);
    m_playButton.setEnabled(false);

    addAndMakeVisible(&m_stopButton);
    m_stopButton.setButtonText("Stop");
    m_stopButton.onClick = [this] { m_audioFileController.stopPlayback(); };
    m_stopButton.setColour(juce::TextButton::buttonColourId, juce::Colours::red);
    m_stopButton.setEnabled(false);

    addAndMakeVisible(&m_aziDial);
    m_aziDial.setRotaryParameters(0.0f, 2 * juce::float_Pi, false);
    m_aziDial.setColour(juce::Slider::ColourIds::rotarySliderFillColourId, 
        m_aziDial.findColour(juce::Slider::ColourIds::rotarySliderOutlineColourId));
    m_aziDial.setRange(0, 360.0, 0.1);
    m_aziDial.setNumDecimalPlacesToDisplay(1);
    m_aziDial.setTextValueSuffix(" deg");
    m_aziDial.onValueChange = [this] {onSliderChange();};

    m_aziLabel.setJustificationType(juce::Justification::centredBottom);
    m_aziLabel.attachToComponent(&m_aziDial, false);
    m_aziLabel.setText("Azimuth Angle", juce::dontSendNotification);

    addAndMakeVisible(&m_eleSlider);
    m_eleSlider.setColour(juce::Slider::ColourIds::trackColourId,
        m_eleSlider.findColour(juce::Slider::ColourIds::backgroundColourId));
    m_eleSlider.setRange(-90.0, 90.0, 0.1);
    m_eleSlider.setNumDecimalPlacesToDisplay(1);
    m_eleSlider.setTextValueSuffix(" deg");
    m_eleSlider.onValueChange = [this] {onSliderChange(); };

    m_eleLabel.setJustificationType(juce::Justification::centredBottom);
    m_eleLabel.attachToComponent(&m_eleSlider, false);
    m_eleLabel.setText("Elevation Angle", juce::dontSendNotification);

    addAndMakeVisible(&m_ConsoleText);
    m_ConsoleText.setJustificationType(juce::Justification::topLeft);
}

MainComponent::~MainComponent()
{
    // This shuts down the audio device and clears the audio source.
    shutdownAudio();
}

//==============================================================================
void MainComponent::prepareToPlay(int samplesPerBlockExpected, double sampleRate)
{
    // This function will be called when the audio device is started, or when
    // its settings (i.e. sample rate, block size, etc) are changed.

    // You can use this function to initialise any resources you might need,
    // but be careful - it will be called on the audio thread, not the GUI thread.

    m_audioFileController.prepareToPlay(samplesPerBlockExpected, sampleRate);
    m_spatialiserController.prepareToPlay(samplesPerBlockExpected, sampleRate);
}

void MainComponent::getNextAudioBlock (const juce::AudioSourceChannelInfo& bufferToFill)
{
    if (!m_audioFileController.isProducingData())
    {
        // Right now we are not producing any data, in which case we need to clear the buffer
        // (to prevent the output of random noise)
        bufferToFill.clearActiveBufferRegion();
        return;
    }

    m_audioFileController.getNextAudioBlock(bufferToFill);
    m_spatialiserController.spatialise(bufferToFill);
}

void MainComponent::releaseResources()
{
    // This will be called when the audio device stops, or when it is being
    // restarted due to a setting change.

    m_audioFileController.releaseResources();
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

    m_openSofaFileButton.setBounds(10, 10, getWidth() - 20, 20);
    m_openAudioFileButton.setBounds(10, 40, getWidth() - 20, 20);
    m_playButton.setBounds(10, 70, getWidth() - 20, 20);
    m_stopButton.setBounds(10, 100, getWidth() - 20, 20);

    m_aziDial.setBounds(10, 155, getWidth()/2, getWidth()/2);
    m_eleSlider.setBounds(getWidth() / 2, 155, getWidth() / 2, getWidth() / 2);

    m_ConsoleText.setBounds(20, 170 + getWidth()/2, getWidth(), getWidth());
}

void MainComponent::setLoadSOFAButtonLoading(bool enable)
{
    m_openSofaFileButton.setEnabled(!enable);
    if (enable)
    {
        m_openSofaFileButton.setButtonText("Loading SOFA HRTF File");
    }
    else
    {
        m_openSofaFileButton.setButtonText("Open SOFA HRTF File");
    }
}

void MainComponent::onSliderChange()
{
    // Azimuth angles in SOFA files increase anticlockwise (i.e. towards the left), whereas the juce rotary slider 
    // increases clockwise. The best thing to do would be to make a new loolandfeel for the rotary slider, but I'm lazy. 
    // So just the value from the azimuth slider to be negative it's value.
    m_spatialiserController.setPosition(-m_aziDial.getValue(), m_eleSlider.getValue());
}