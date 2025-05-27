// ==========================| NodeTemplate v1.0 : by Griffinboy |==========================

#pragma once
#include <JuceHeader.h>
#include <vector>
#include <cmath>
#include <tuple>
#include <algorithm>
#include <iostream>

namespace project
{
    using namespace juce;
    using namespace hise;
    using namespace scriptnode;

    template <int NV>
    struct Griffin_Diode : public data::base
    {
        SNEX_NODE(Griffin_Diode);

        struct MetadataClass
        {
            SN_NODE_ID("Griffin_Diode");
        };

        // ==========================| Node Properties |==========================
        static constexpr bool isModNode() { return false; }
        static constexpr bool isPolyphonic() { return NV > 1; }
        static constexpr bool hasTail() { return false; }
        static constexpr bool isSuspendedOnSilence() { return true; }
        static constexpr int getFixChannelAmount() { return 2; }

        static constexpr int NumTables = 0;
        static constexpr int NumSliderPacks = 0;
        static constexpr int NumAudioFiles = 0;
        static constexpr int NumFilters = 0;
        static constexpr int NumDisplayBuffers = 0;

        // ==========================| TriodeProcessor Class |==========================
        class TriodeProcessor
        {
        public:
            TriodeProcessor(double sampleRate, double Vdd, double Ri, double Rk, double Rp, double Rg, double Ci, double Ck, double Co,
                double kp, double kp2, double kpg)
                : fs(sampleRate), Vdd(Vdd), Ri(Ri), Rk(Rk), Rp(Rp), Rg(Rg), Ci(Ci), Ck(Ck), Co(Co),
                kp(kp), kp2(kp2), kpg(kpg)
            {
                prepare();
            }

            void prepare()
            {
                double E = limit(Vdd, 1e-6, std::numeric_limits<double>::infinity(), "E");
                double Rp = limit(this->Rp, 1e-6, std::numeric_limits<double>::infinity(), "Rp");
                double Rk = limit(this->Rk, 1e-6, std::numeric_limits<double>::infinity(), "Rk");
                double Ck = limit(this->Ck, 1e-18, std::numeric_limits<double>::infinity(), "Ck");

                // Initial voltages
                double k1 = kpg / (2 * kp2) + Rp / Rk + 1;
                double k2 = k1 * (kp / kp2 + 2 * E) * kp2;
                double k3 = Rk * k2 + 1;
                Vk0 = (k3 - std::copysign(std::sqrt(2 * k3 - 1), k1)) / (2 * Rk * k1 * k1 * kp2);
                Vp0 = E - Rp / Rk * Vk0;

                wCi_s = 0;
                wCk_s = Vk0;
                wCo_s = Vp0;
            }

            std::vector<double> process(const std::vector<double>& inputSignal)
            {
                return triode_stage_quadric(inputSignal);
            }

        private:
            double fs, Vdd, Ri, Rk, Rp, Rg, Ci, Ck, Co;
            double kp, kp2, kpg;
            double Vk0, Vp0;
            double wCi_s, wCk_s, wCo_s;

            double limit(double x, double low, double up, const std::string& name)
            {
                if (x < low || x > up)
                {
                    return std::clamp(x, low, up);
                }
                return x;
            }

            std::tuple<double, double, double> triode(double ag, double ak, double ap, double R0g, double R0k, double R0p, double kp, double kp2, double kpg)
            {
                double bk_bp = R0k / R0p;
                double k_eta = 1 / (bk_bp * (0.5 * kpg + kp2) + kp2);
                double k_delta = kp2 * k_eta * k_eta / (R0p + R0p);
                double k_bp_s = k_eta * std::sqrt((kp2 + kp2) / R0p);
                double bp_k = 1 / (R0p + R0k);
                double bp_ap_0 = bp_k * (R0k - R0p);
                double bp_ak_0 = bp_k * (R0p + R0p);

                double v1 = 0.5 * ap;
                double v2 = ak + v1 * bk_bp;
                double alpha = kpg * (ag - v2) + kp;
                double beta = kp2 * (v1 - v2);
                double eta = k_eta * (beta + beta + alpha);
                double v3 = eta + k_delta;
                double delta = ap + v3;

                double bp, bk, Vpk;
                if (delta >= 0)
                {
                    bp = k_bp_s * std::sqrt(delta) - v3 - k_delta;
                    double d = bk_bp * (ap - bp);
                    bk = ak + d;
                    double Vpk2 = ap + bp - ak - bk;

                    if (kpg * (ag - ak - 0.5 * d) + kp2 * Vpk2 + kp < 0)
                    {
                        bp = ap;
                        bk = ak;
                        Vpk = ap - ak;
                    }
                    else
                    {
                        Vpk = 0.5 * Vpk2;
                    }
                }
                else
                {
                    bp = ap;
                    bk = ak;
                    Vpk = ap - ak;
                }

                if (Vpk < 0)
                {
                    bp = bp_ap_0 * ap + bp_ak_0 * ak;
                }

                double bg = ag;
                return std::make_tuple(bg, bk, bp);
            }

            std::vector<double> triode_stage_quadric(const std::vector<double>& x)
            {
                double E = limit(Vdd, 1e-6, std::numeric_limits<double>::infinity(), "E");
                double Rp = limit(this->Rp, 1e-6, std::numeric_limits<double>::infinity(), "Rp");
                double Rk = limit(this->Rk, 1e-6, std::numeric_limits<double>::infinity(), "Rk");
                double Ck = limit(this->Ck, 1e-18, std::numeric_limits<double>::infinity(), "Ck");

                // Constants
                double Ro = 1e6;

                // WDF element values
                double wVi_R = 1e-6;
                double wCi_R = 1 / (2 * fs * Ci);
                double wCk_R = 1 / (2 * fs * Ck);
                double wCo_R = 1 / (2 * fs * Co);
                double wsi_kl = wCi_R / (wCi_R + wVi_R);
                double wsi_R = wCi_R + wVi_R;
                double wpg_kt = wsi_R / (wsi_R + Ri);
                double wpg_R = (wsi_R * Ri) / (wsi_R + Ri);
                double wsg_kl = Rg / (Rg + wpg_R);
                double wsg_R = Rg + wpg_R;
                double wpk_kt = wCk_R / (Rk + wCk_R);
                double wpk_R = (Rk * wCk_R) / (Rk + wCk_R);
                double wsp_kl = wCo_R / (wCo_R + Ro);
                double wsp_R = wCo_R + Ro;
                double wpp_kt = wsp_R / (wsp_R + Rp);
                double wpp_R = (wsp_R * Rp) / (wsp_R + Rp);

                // Filter coefficients
                double kTxCi = 1 - wpg_kt;
                double kTCk = 1 - wpk_kt;
                double kTCo = 1 - wpp_kt;
                double kT0 = wpp_kt * E;
                double kyT = 0.5 * (1 - wsp_kl);
                double kyCo = -0.5 * (1 - wsp_kl) * (1 + wpp_kt);
                double ky0 = 0.5 * (1 - wsp_kl) * wpp_kt * E;
                double kCiT = wsi_kl * (1 - wsg_kl);
                double kCixCi = wsi_kl * ((1 - wpg_kt) * (wsg_kl + 1) - 2);
                double kCoCo = 1 - wsp_kl * (1 + wpp_kt);
                double kCo0 = wsp_kl * wpp_kt * E;

                // Filter
                std::vector<double> y(x.size(), 0.0);

                for (size_t i = 0; i < x.size(); ++i)
                {
                    double xCi = x[i] + wCi_s;
                    double wT_ag = kTxCi * xCi;
                    double wT_ak = kTCk * wCk_s;
                    double wT_ap = kTCo * wCo_s + kT0;
                    auto [wT_bg, wT_bk, wT_bp] = triode(wT_ag, wT_ak, wT_ap, wsg_R, wpk_R, wpp_R, kp, kp2, kpg);
                    y[i] = kyT * wT_bp + kyCo * wCo_s + ky0;
                    wCi_s = kCiT * wT_bg + kCixCi * xCi + wCi_s;
                    wCk_s = wT_bk - wpk_kt * wCk_s;
                    wCo_s = wsp_kl * wT_bp + kCoCo * wCo_s + kCo0;
                }

                return y;
            }
        };

        // ==========================| Prepare |==========================
        void prepare(PrepareSpecs specs)
        {
            double fs = specs.sampleRate;
            double Vdd = 250.0;  // Supply voltage
            double Ri = 1e6;     // Input resistance
            double Rk = 1e3;     // Cathode resistance
            double Rp = 100e3;   // Plate resistance
            double Rg = 20e3;    // Grid resistance
            double Ci = 100e-9;  // Input capacitance
            double Ck = 10e-6;   // Cathode capacitance
            double Co = 10e-9;   // Output capacitance

            double kp = 1.014e-5;
            double kp2 = 5.498e-8;
            double kpg = 1.076e-5;

            leftChannelProcessor = std::make_unique<TriodeProcessor>(fs, Vdd, Ri, Rk, Rp, Rg, Ci, Ck, Co, kp, kp2, kpg);
            rightChannelProcessor = std::make_unique<TriodeProcessor>(fs, Vdd, Ri, Rk, Rp, Rg, Ci, Ck, Co, kp, kp2, kpg);
        }

        // ==========================| Reset |==========================
        void reset()
        {
            leftChannelProcessor->prepare();
            rightChannelProcessor->prepare();
        }

        // ==========================| Process |==========================
        template <typename ProcessDataType>
        void process(ProcessDataType& data)
        {
            auto& fixData = data.template as<ProcessData<getFixChannelAmount()>>();
            auto audioBlock = fixData.toAudioBlock();
            auto* leftChannelData = audioBlock.getChannelPointer(0);
            auto* rightChannelData = audioBlock.getChannelPointer(1);
            int numSamples = data.getNumSamples();

            std::vector<double> leftInputSignal(leftChannelData, leftChannelData + numSamples);
            std::vector<double> rightInputSignal(rightChannelData, rightChannelData + numSamples);

            std::vector<double> leftOutputSignal = leftChannelProcessor->process(leftInputSignal);
            std::vector<double> rightOutputSignal = rightChannelProcessor->process(rightInputSignal);

            for (int i = 0; i < numSamples; ++i)
            {
                leftChannelData[i] = static_cast<float>(leftOutputSignal[i]);
                rightChannelData[i] = static_cast<float>(rightOutputSignal[i]);
            }
        }

        // ==========================| External Data |==========================
        void setExternalData(const ExternalData& data, int index) {}

        // ==========================| Handle HISE Event |==========================
        void handleHiseEvent(HiseEvent& e) {}

        // processFrame: Needed for compiler, does nothing
        template <typename FrameDataType>
        void processFrame(FrameDataType& data) {}

        template <int P>
        void setParameter(double v)
        {

        }

        void createParameters(ParameterDataList& data)
        {

        }

    private:
        std::unique_ptr<TriodeProcessor> leftChannelProcessor;
        std::unique_ptr<TriodeProcessor> rightChannelProcessor;
    };
}