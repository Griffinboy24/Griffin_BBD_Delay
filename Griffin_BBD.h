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


#include <cmath>

namespace project
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef M_LN10
#define M_LN10 2.30258509299404568402
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

        class AudioEffect
        {
        public:
            static constexpr float kMinDelayMs = 40.0f;
            static constexpr float kMaxDelayMs = 3000.0f;
            static constexpr float kMaxDarknessOffsetHz = 300.0f;  // max reduction in cutoff

            AudioEffect(float initTotalMs = 333.0f,
                float initFeedback = 0.35f,
                float initWetGain = 1.0f,
                float initDryGain = 1.0f,
                float initBrightness = 0.285714f) noexcept
                : totalDelayMs(initTotalMs),
                feedback(initFeedback),
                wetGain(initWetGain),
                dryGain(initDryGain),
                brightness(initBrightness)
            {
                wetBuffer.allocate(kMaxBlock, true);
            }

            void prepare(double newFs)
            {
                fs = float(newFs);
                dsp::ProcessSpec spec{ newFs, uint32(kMaxBlock), 1 };
                for (auto& d : line) d.prepare(spec);
                comp.prepare(newFs, kMaxBlock);
                exp.prepare(newFs, kMaxBlock);
                constexpr float compFc = 40.0f;
                comp.setCutoff(compFc);
                exp.setCutoff(compFc);
                updateParams();
            }

            inline void process(float* buf, int n)
            {
                jassert(n <= kMaxBlock);

                // ensure filters and delays updated once per block
                updateParams();

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

                const float dGain = dryGain * volumeCompGain;
                const float wGain = wetGain * volumeCompGain;
                FloatVectorOperations::multiply(buf, dGain, n);
                FloatVectorOperations::addWithMultiply(buf, wet, wGain, n);
            }

            void setTotalDelayMs(float v) { totalDelayMs = v;               updateParams(); }
            void setFeedback(float v) { feedback = jlimit(0.0f, 0.99f, v); }
            void setWetGain(float v) { wetGain = v; }
            void setDryGain(float v) { dryGain = v; }
            void setBrightness(float v) { brightness = jlimit(0.0f, 1.0f, v); updateParams(); }
            void setTrim(float d, float f) { delayTrim = d; filterTrim = f; updateParams(); }

        private:
            inline void updateParams()
            {
                // Delay per chip
                const float perChipMs = totalDelayMs * delayTrim * 0.25f;
                const float samples = fs * perChipMs * 0.001f;
                for (auto& d : line) d.setDelay(samples);

                // Darkness offset based on total delay
                const float normDelay = jlimit(0.0f, 1.0f,
                    (totalDelayMs - kMinDelayMs) /
                    (kMaxDelayMs - kMinDelayMs));
                const float darknessOffset = normDelay * kMaxDarknessOffsetHz;

                // Brightness to cutoff mapping
                const float baseCutoff = brightness * 2800.0f + 200.0f;
                const float adjCutoffHz = jlimit(200.0f, 3000.0f, baseCutoff - darknessOffset);
                const float cutoffFreq = adjCutoffHz * filterTrim;

                for (auto& d : line) d.setFilterFreq(cutoffFreq);
                volumeCompGain = computeVolumeCompGain(cutoffFreq);
            }

            static constexpr float comp_m = -4.3913563f;
            static constexpr float comp_c = 18.3949866f;
            static constexpr float comp_m_ln = comp_m / M_LN10;
            static constexpr float compScale = 0.05f * M_LN10;

            static inline float computeVolumeCompGain(float f) noexcept
            {
                // faster alternative to log10f + powf
                return expf((comp_m_ln * logf(f) + comp_c) * compScale);
            }

            static constexpr int kMaxBlock = 4096;

            float fs = 48000.0f, totalDelayMs, feedback, wetGain, dryGain;
            float brightness;
            float delayTrim = 1.0f, filterTrim = 1.0f;
            float lastWetSample = 0.0f;
            float volumeCompGain = 1.0f;

            chowdsp::BBD::BBDDelayWrapper<4096, false> line[4];
            BBDCompressor comp;
            BBDExpander   exp;
            juce::HeapBlock<float, 32> wetBuffer;
        };

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

            if (widthFactor < 1.0f)
            {
                float* tmp = msScratch.get();
                const float mid = 0.5f;
                const float sd = 0.5f * widthFactor;

                FloatVectorOperations::subtract(tmp, l, r, n);
                FloatVectorOperations::multiply(tmp, sd, n);
                FloatVectorOperations::add(r, l, r, n);
                FloatVectorOperations::multiply(r, mid, n);
                FloatVectorOperations::add(l, r, tmp, n);
                FloatVectorOperations::subtract(r, r, tmp, n);
            }
        }

        template <int P>
        inline void setParameter(double v)
        {
            if constexpr (P == 0) { L.setTotalDelayMs(float(v));    R.setTotalDelayMs(float(v)); }
            else if constexpr (P == 1) { L.setFeedback(float(v));    R.setFeedback(float(v)); }
            else if constexpr (P == 2) { L.setWetGain(float(v));     R.setWetGain(float(v)); }
            else if constexpr (P == 3) { L.setDryGain(float(v));     R.setDryGain(float(v)); }
            else if constexpr (P == 4) { L.setBrightness(float(v));  R.setBrightness(float(v)); }
            else if constexpr (P == 5)
            {
                toleranceScale = float(v);
                applyTolerances();
                updateWidthFactor();
            }
        }

        void createParameters(ParameterDataList& data)
        {
            parameter::data p0("Delay (ms)", { 40.0,   3000.0, 0.1 }); registerCallback<0>(p0); p0.setDefaultValue(300.0); data.add(std::move(p0));
            parameter::data p1("Feedback", { 0.0,    0.99,  0.001 }); registerCallback<1>(p1); p1.setDefaultValue(0.3);   data.add(std::move(p1));
            parameter::data p2("Wet Gain", { 0.0,    4.0,   0.001 }); registerCallback<2>(p2); p2.setDefaultValue(1.0);   data.add(std::move(p2));
            parameter::data p3("Dry Gain", { 0.0,    2.0,   0.001 }); registerCallback<3>(p3); p3.setDefaultValue(1.0);   data.add(std::move(p3));
            parameter::data p4("Brightness", { 0.0,    1.0,   0.001 }); registerCallback<4>(p4); p4.setDefaultValue(0.3);   data.add(std::move(p4));
            parameter::data p5("Stereo Width", { -1.0,   1.0,   0.001 }); registerCallback<5>(p5); p5.setDefaultValue(0.0);   data.add(std::move(p5));
        }

        SN_EMPTY_PROCESS_FRAME;
        SN_EMPTY_HANDLE_EVENT;
        SN_EMPTY_SET_EXTERNAL_DATA;

    private:
        static constexpr int kMaxBlock = 4096;
        juce::HeapBlock<float, 32> msScratch;

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

            const float ldTrim = 1.0f - tolScale * baseDelayTol;
            const float rdTrim = 1.0f + tolScale * baseDelayTol;
            const float lfTrim = 1.0f;
            const float rfTrim = 1.0f + tolScale * baseFiltTolR;

            L.setTrim(ldTrim, lfTrim);
            R.setTrim(rdTrim, rfTrim);
        }

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
        float toleranceScale = 1.0f;
        float widthFactor = 1.0f;
    };

} // namespace project
