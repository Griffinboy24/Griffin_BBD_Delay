
#pragma once
#include <JuceHeader.h>

#include "src\Utils_XSimd.h"
#include "src\Utils_General.h"
#include "src\chowdsp_CoefficientCalculators.h"

#include "src\chowdsp_VicanekHelpers.h"
#include "src\chowdsp_JacobiElliptic.h"
#include "src\chowdsp_Ratio.h"
#include "src\chowdsp_LinearTransforms.h"
#include "src\chowdsp_QValCalcs.h"
#include "src\chowdsp_ConformalMaps.h"
#include "src\chowdsp_FilterChain.h"
#include "src\chowdsp_Power.h"
#include "src\chowdsp_Combinatorics.h"

#include "src\chowdsp_EllipticFilter.h"
#include "src\chowdsp_IIRFilter.h"
#include "src\chowdsp_SOSFilter.h"


namespace project
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    // pre C++20
    template <int NV>
    struct Griffin_Elliptic : public data::base
    {
        SNEX_NODE(Griffin_Elliptic);

        struct MetadataClass
        {
            SN_NODE_ID("Griffin_Elliptic");
        };

        //==============================================================================
        // Node Properties 
        //==============================================================================
        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return false; }
        static constexpr int  getFixChannelAmount() { return 2; }

        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        //==============================================================================
        // Audio Effect Class
        //==============================================================================
        class AudioEffect
        {
        public:
            AudioEffect(float initCutoff = 5000.0f)
                : cutoffHz(initCutoff)
            {
            }

            void prepare(double sr)
            {
                sampleRate = sr;
                // our filter is mono, so prepare(1)
                filter.prepare(1);
                // calculate initial coefficients (qVal=1.0 for default)
                filter.calcCoefs(cutoffHz, NumericType(1.0), NumericType(sampleRate));
            }

            inline void process(float* samples, int numSamples)
            {
                // channel = 0 for this mono filter
                filter.processBlock(samples, numSamples, 0);
            }

            inline void updateParam1(float newCutoff)
            {
                cutoffHz = newCutoff;
                filter.calcCoefs(cutoffHz, NumericType(1.0), NumericType(sampleRate));
            }

        private:
            using NumericType = chowdsp::SampleTypeHelpers::NumericType<float>;

            float cutoffHz;
            double sampleRate = 44100.0;
            // 4th-order elliptic filter
            chowdsp::EllipticFilter<4> filter;
        };

        //==============================================================================
        // Main Node
        //==============================================================================
        Griffin_Elliptic() = default;

        void prepare(PrepareSpecs specs)
        {
            leftChannelEffect.prepare(specs.sampleRate);
            rightChannelEffect.prepare(specs.sampleRate);
        }

        void reset() {}

        template <typename PD>
        inline void process(PD& d)
        {
            auto& fix = d.template as<ProcessData<2>>();
            auto  blk = fix.toAudioBlock();
            float* L = blk.getChannelPointer(0);
            float* R = blk.getChannelPointer(1);
            int    n = d.getNumSamples();

            leftChannelEffect.process(L, n);
            rightChannelEffect.process(R, n);
        }

        //==============================================================================
        // Parameter Handling
        //==============================================================================
        template <int P>
        inline void setParameter(double v)
        {
            if (P == 0)
            {
                leftChannelEffect.updateParam1((float)v);
                rightChannelEffect.updateParam1((float)v);
            }
        }

        void createParameters(ParameterDataList& data)
        {
            parameter::data p("Cutoff", { 20.0, 20000.0, 1.0 });
            registerCallback<0>(p);
            p.setDefaultValue(2500.0);
            data.add(std::move(p));
        }

        SN_EMPTY_PROCESS_FRAME;
        SN_EMPTY_HANDLE_EVENT;
        SN_EMPTY_SET_EXTERNAL_DATA;

    private:
        AudioEffect leftChannelEffect, rightChannelEffect;
    };

} // namespace project