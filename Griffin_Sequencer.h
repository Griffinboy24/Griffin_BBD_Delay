#pragma once
#include <JuceHeader.h>
#include <cmath>

namespace project
{

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    // pre C++20
    // You cannot change the way this node is templated
    template <int NV>
    struct Griffin_Sequencer : public data::base
    {
        SNEX_NODE(Griffin_Sequencer);

        struct MetadataClass
        {
            SN_NODE_ID("Griffin_Sequencer");
        };

        Griffin_Sequencer()
            : rampValue_(0.0),
            baseDelay_(500.0),
            semitone1_(0.0),
            semitone2_(0.0),
            currentStep_(-1)
        {
            updateTable();
        }

        //==============================================================================
        // Node Properties 
        //==============================================================================

        static constexpr bool isModNode() { return true; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return false; }
        static constexpr int   getFixChannelAmount() { return 2; }

        // Add external data slots to the node UI
        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        //==============================================================================
        // Main Processing Functions
        //==============================================================================

        void prepare(PrepareSpecs /*specs*/) {}

        void reset()
        {
            rampValue_ = 0.0;
            currentStep_ = -1;
        }

        template <typename PD>
        inline void process(PD& d)
        {
            // determine which of the 3 steps we're in (0,1 or 2)
            int newStep = static_cast<int>(rampValue_ * 3.0);
            if (newStep < 0) newStep = 0;
            if (newStep > 2) newStep = 2;

            if (newStep != currentStep_)
            {
                currentStep_ = newStep;
                // apply the precomputed delay for this step
                modOut.setModValueIfChanged(delays_[currentStep_]);
            }
        }

        //==============================================================================
        // Parameter Handling
        //==============================================================================

        template <int P>
        inline void setParameter(double v)
        {
            if (P == 0)
            {
                // time ramp (0 1)
                rampValue_ = v;
            }
            else if (P == 1)
            {
                // base delay (ms)
                baseDelay_ = v;
                updateTable();
                if (currentStep_ == 0)
                    modOut.setModValueIfChanged(static_cast<float>(baseDelay_));
            }
            else if (P == 2)
            {
                // first interval (semitones)
                semitone1_ = v;
                updateTable();
                if (currentStep_ == 1)
                    modOut.setModValueIfChanged(delays_[1]);
            }
            else if (P == 3)
            {
                // second interval (semitones)
                semitone2_ = v;
                updateTable();
                if (currentStep_ == 2)
                    modOut.setModValueIfChanged(delays_[2]);
            }
        }

        void createParameters(ParameterDataList& data)
        {
            {
                parameter::data p("time ramp", { 0.0, 1.0, 0.00001 });
                registerCallback<0>(p);
                p.setDefaultValue(0.0);
                data.add(std::move(p));
            }
            {
                parameter::data p("base delay", { 200.0, 1500.0, 1.0 });
                registerCallback<1>(p);
                p.setDefaultValue(500.0);
                data.add(std::move(p));
            }
            {
                parameter::data p("step 1", { -24.0, 24.0, 1.0 });
                registerCallback<2>(p);
                p.setDefaultValue(0.0);
                data.add(std::move(p));
            }
            {
                parameter::data p("step 2", { -24.0, 24.0, 1.0 });
                registerCallback<3>(p);
                p.setDefaultValue(0.0);
                data.add(std::move(p));
            }
        }

        //==============================================================================
        // Modulation Output
        //==============================================================================

        ModValue modOut;
        bool handleModulation(double& value)
        {
            return modOut.getChangedValue(value);
        }

        SN_EMPTY_PROCESS_FRAME;
        SN_EMPTY_HANDLE_EVENT;
        SN_EMPTY_SET_EXTERNAL_DATA;

    private:

        void updateTable()
        {
            // compute three relative intervals summing to zero
            double s1 = semitone1_;
            double s2 = semitone2_;
            double s3 = -(s1 + s2);

            double r1 = std::pow(2.0, -s1 / 12.0);
            double r2 = std::pow(2.0, -s2 / 12.0);

            // build absolute delay times for each of the three steps
            delays_[0] = static_cast<float>(baseDelay_);
            delays_[1] = static_cast<float>(baseDelay_ * r1);
            delays_[2] = static_cast<float>(baseDelay_ * r1 * r2);
        }

        // Sequencer state
        double rampValue_;            // 0 1 clock phase
        double baseDelay_;            // ms
        double semitone1_, semitone2_;
        float  delays_[3];            // precomputed [base, step1, step2]
        int    currentStep_;
    };

} // namespace project
