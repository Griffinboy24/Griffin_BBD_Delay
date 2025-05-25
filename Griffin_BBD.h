/*
    Griffin_BBD: true-stereo 4-chip BBD delay
*/

#pragma once

#include <JuceHeader.h>
#include "src/XSimd_Helpers.h"
#include "src/griffin_BBDDelayLine.h"
#include "src/griffin_BBDDelayWrapper.h"
#include "src/griffin_BBDFilterBank.h"
#include "src/griffin_DelayInterpolation.h"
#include "src/griffin_Compander.h"

namespace project
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    template <int NV>
    struct Griffin_BBD : public data::base
    {
        SNEX_NODE(Griffin_BBD);
        struct MetadataClass { SN_NODE_ID("Griffin_BBD"); };

        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return true; }
        static constexpr bool isSuspendedOnSilence() { return true; }
        static constexpr int  getFixChannelAmount() { return 2; }

        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        // Core BBD processing unit
        class AudioEffect
        {
        public:
            AudioEffect(float initTotalMs = 333.0f,
                float initFeedback = 0.35f,
                float initWetGain = 1.0f,
                float initDryGain = 1.0f,
                int   initMode = 1) noexcept
                : totalDelayMs(initTotalMs),
                feedback(initFeedback),
                wetGain(initWetGain),
                dryGain(initDryGain),
                voicingMode(initMode)
            {
                wetBuffer.allocate(kMaxBlock, true);
            }

            // Initialize DSP components
            void prepare(double newFs)
            {
                fs = float(newFs);
                dsp::ProcessSpec spec{ newFs, uint32(kMaxBlock), 1 };
                for (auto& d : line)
                    d.prepare(spec);

                comp.prepare(newFs, kMaxBlock);
                exp.prepare(newFs, kMaxBlock);

                constexpr float compFc = 40.0f;
                comp.setCutoff(compFc);
                exp.setCutoff(compFc);

                updateParams();
            }

            // Process one block of audio
            inline void process(float* buf, int n)
            {
                jassert(n <= kMaxBlock);
                auto* wet = wetBuffer.get();
                FloatVectorOperations::copy(wet, buf, n);

                const float fb = feedback;
                float last = lastWetSample;

                for (int i = 0; i < n; ++i)
                {
                    float in = wet[i] + last * fb;
                    float x = comp.process(in);

                    for (auto& d : line)
                    {
                        d.pushSample(0, x);
                        x = d.popSample(0);
                    }

                    float out = exp.process(x);
                    last = out;
                    wet[i] = out;
                }

                lastWetSample = last;

                FloatVectorOperations::multiply(buf, dryGain, n);
                FloatVectorOperations::addWithMultiply(buf, wet, wetGain, n);

                // once per block – keeps filter clocks aligned
                updateParams();
            }

            // Setters update internal parameters as needed
            void setTotalDelayMs(float v) { totalDelayMs = v; updateParams(); }
            void setFeedback(float v) { feedback = jlimit(0.0f, 0.99f, v); }
            void setWetGain(float v) { wetGain = v; }
            void setDryGain(float v) { dryGain = v; }
            void setVoicingMode(int v) { voicingMode = jlimit(1, 4, v); updateParams(); }
            void setTrim(float dTrim, float fTrim) { delayTrim = dTrim; filterTrim = fTrim; updateParams(); }

        private:
            void updateParams()
            {
                static constexpr float bright[4] = { 0.5f, 1.0f, 1.4f, 1.8f };
                float perChipMs = totalDelayMs * delayTrim * 0.25f;
                float samples = fs * perChipMs * 0.001f;

                for (auto& d : line)
                    d.setDelay(samples);

                constexpr float refSec = 0.020f;
                float delaySec = samples / fs;
                float cutoff = chowdsp::BBD::BBDFilterSpec::inputFilterOriginalCutoff
                    * (refSec / delaySec)
                    * bright[voicingMode - 1]
                    * filterTrim;

                for (auto& d : line)
                    d.setFilterFreq(cutoff);
            }

            static constexpr int kMaxBlock = 4096;
            float fs = 48000.0f, totalDelayMs, feedback, wetGain, dryGain;
            int voicingMode;
            float delayTrim = 1.0f, filterTrim = 1.0f;
            float lastWetSample = 0.0f;

            chowdsp::BBD::BBDDelayWrapper<4096, false> line[4];
            BBDCompressor comp;
            BBDExpander   exp;
            juce::HeapBlock<float, 32> wetBuffer;
        };

        // Prepare stereo processors
        void prepare(PrepareSpecs s)
        {
            msScratch.allocate(kMaxBlock, false);
            L.prepare(s.sampleRate);
            R.prepare(s.sampleRate);
            applyTolerances();
            updateWidthFactor();
        }

        void reset() {}

        template <typename PD>
        void process(PD& d)
        {
            auto& f = d.template as<ProcessData<2>>();
            auto blk = f.toAudioBlock();
            float* l = blk.getChannelPointer(0);
            float* r = blk.getChannelPointer(1);
            int n = d.getNumSamples();

            L.process(l, n);
            R.process(r, n);

            // Optional stereo width reduction
            if (widthFactor < 1.0f)
            {
                float* tmp = msScratch.get();
                float midScale = 0.5f;
                float sideScale = 0.5f * widthFactor;

                FloatVectorOperations::subtract(tmp, l, r, n);
                FloatVectorOperations::multiply(tmp, sideScale, n);

                FloatVectorOperations::add(r, l, r, n);
                FloatVectorOperations::multiply(r, midScale, n);

                FloatVectorOperations::add(l, r, tmp, n);
                FloatVectorOperations::subtract(r, r, tmp, n);
            }
        }

        // Parameter callbacks
        template <int P>
        inline void setParameter(double v)
        {
            if constexpr (P == 0) { L.setTotalDelayMs(float(v)); R.setTotalDelayMs(float(v)); }
            else if constexpr (P == 1) { L.setFeedback(float(v));       R.setFeedback(float(v)); }
            else if constexpr (P == 2) { L.setWetGain(float(v));        R.setWetGain(float(v)); }
            else if constexpr (P == 3) { L.setDryGain(float(v));        R.setDryGain(float(v)); }
            else if constexpr (P == 4)
            {
                int m = int(v + 0.5);
                L.setVoicingMode(m);
                R.setVoicingMode(m);
            }
            else if constexpr (P == 5)
            {
                toleranceScale = float(v);
                applyTolerances();
                updateWidthFactor();
            }
        }

        void createParameters(ParameterDataList& data)
        {
            parameter::data p0("Delay (ms)", { 40.0, 2000.0, 0.1 }); registerCallback<0>(p0); p0.setDefaultValue(300.0); data.add(std::move(p0));
            parameter::data p1("Feedback", { 0.0,  0.99,   0.001 }); registerCallback<1>(p1); p1.setDefaultValue(0.3);   data.add(std::move(p1));
            parameter::data p2("Wet Gain", { 0.0,  4.0,    0.001 }); registerCallback<2>(p2); p2.setDefaultValue(1.0);   data.add(std::move(p2));
            parameter::data p3("Dry Gain", { 0.0,  2.0,    0.001 }); registerCallback<3>(p3); p3.setDefaultValue(1.0);   data.add(std::move(p3));
            parameter::data p4("Brightness", { 1.0,  4.0,    1.0 }); registerCallback<4>(p4); p4.setDefaultValue(1.0);   data.add(std::move(p4));
            parameter::data p5("Analog Width", { -1.0, 2.0,    0.001 }); registerCallback<5>(p5); p5.setDefaultValue(1.0);   data.add(std::move(p5));
        }

        SN_EMPTY_PROCESS_FRAME;
        SN_EMPTY_HANDLE_EVENT;
        SN_EMPTY_SET_EXTERNAL_DATA;

    private:
        static constexpr int kMaxBlock = 4096;
        juce::HeapBlock<float, 32> msScratch;

        // Apply analog mismatch tolerances based on control value
        void applyTolerances()
        {
            constexpr float baseDelayTol = 0.001f;
            constexpr float baseFiltTolR = -0.005f;
            constexpr float tolAtZero = 0.5f;
            float tolScale;

            if (toleranceScale >= 0.0f)
                tolScale = tolAtZero + toleranceScale;
            else
            {
                float frac = jlimit(0.0f, 1.0f, (toleranceScale + 0.3f) / 0.3f);
                tolScale = tolAtZero * frac;
            }

            float ldTrim = 1.0f - tolScale * baseDelayTol;
            float rdTrim = 1.0f + tolScale * baseDelayTol;
            float lfTrim = 1.0f;
            float rfTrim = 1.0f + tolScale * baseFiltTolR;

            L.setTrim(ldTrim, lfTrim);
            R.setTrim(rdTrim, rfTrim);
        }

        // Compute stereo width factor for side attenuation
        void updateWidthFactor() noexcept
        {
            if (toleranceScale >= 0.0f)
                widthFactor = 1.0f;
            else
            {
                widthFactor = powf(10.0f, 3.0f * toleranceScale);
                widthFactor = jlimit(0.0f, 1.0f, widthFactor);
            }
        }

        AudioEffect L, R;
        float toleranceScale = 1.0f;  // control for analog mismatch
        float widthFactor = 1.0f;  // stereo width (1.0 = full)
    };

} // namespace project