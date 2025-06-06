#pragma once

#include <JuceHeader.h>

/**
 * Signal averager used for BBD companding.
 */
class BBDAverager
{
public:
    BBDAverager() = default;

    virtual void prepare(double sampleRate, int samplesPerBlock)
    {
        fs = sampleRate;
        onePole.prepare({ sampleRate, (uint32)samplesPerBlock, 1 });
    }

    void setCutoff(float fc)
    {
        onePole.coefficients = dsp::IIR::Coefficients<float>::makeFirstOrderLowPass(fs, fc);
    }

    virtual inline float process(float x) noexcept
    {
        return onePole.processSample(std::pow(std::abs(x), 1.25f));
    }

private:
    double fs = 48000.0;
    dsp::IIR::Filter<float> onePole;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BBDAverager)
};

class BBDExpander : public BBDAverager
{
public:
    BBDExpander() = default;

    inline float process(float x) noexcept override
    {
        return BBDAverager::process(x) * x;
    }

private:
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BBDExpander)
};

class BBDCompressor : public BBDAverager
{
public:
    BBDCompressor() = default;

    void prepare(double sampleRate, int samplesPerBlock) override
    {
        BBDAverager::prepare(sampleRate, samplesPerBlock);
        y1 = 0.0f;
    }

    inline float process(float x) noexcept override
    {
        float y = x / jmax(BBDAverager::process(y1), 0.0001f);
        y1 = y;
        return y;
    }

private:
    float y1 = 0.0f;

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BBDCompressor)
};