// Copyright 2021 Mobilinkd LLC.

#pragma once

#include <array>
#include <cmath>
#include <complex>
#include <cstddef>

namespace mobilinkd
{

/**
 * A sliding DFT algorithm.
 * 
 * Based on 'Understanding and Implementing the Sliding DFT'
 * Eric Jacobsen, 2015-04-23
 * https://www.dsprelated.com/showarticle/776.php
 */
template <typename FloatType, size_t SampleRate, size_t Frequency, size_t Accuracy = 1000>
class SlidingDFT
{
    using ComplexType = std::complex<FloatType>;

    static constexpr size_t N = SampleRate / Accuracy;
    static constexpr ComplexType j{0, 1};
    static constexpr FloatType pi2 = M_PI * 2.0;
    static constexpr FloatType kth = FloatType(Frequency) / FloatType(SampleRate);

    // We'd like this to be static constexpr, but std::exp is not a constexpr.
    const ComplexType coeff_ = std::exp(-j * pi2 * kth);
    std::array<FloatType, N> samples_;
    ComplexType result_{0,0};
    size_t index_ = 0;
    size_t prev_index_ = N - 1;

public:
    SlidingDFT()
    {
        samples_.fill(0);
    }

    ComplexType operator()(FloatType sample)
    {
        auto index = index_;
        index_ += 1;
        if (index_ == N) index_ = 0;

        FloatType delta = sample - samples_[index];
        ComplexType result = (result_ + delta) * coeff_;
        result_ = result * FloatType(0.999999999999999);
        samples_[index] = sample;
        prev_index_ = index;
        return result;
    }
};

/**
 * A sliding DFT algorithm.
 * 
 * Based on 'Understanding and Implementing the Sliding DFT'
 * Eric Jacobsen, 2015-04-23
 * https://www.dsprelated.com/showarticle/776.php
 * 
 * @tparam FloatType is the floating point type to use.
 * @tparam SampleRate is the sample rate of the incoming data.
 * @tparam N is the length of the DFT. Frequency resolution is SampleRate / N.
 * @tparam K is the number of frequencies whose DFT will be calculated.
 */
template <typename FloatType, size_t SampleRate, size_t N, size_t K>
class NSlidingDFT
{
    using ComplexType = std::complex<FloatType>;

    static constexpr ComplexType j{0, 1};
    static constexpr FloatType pi2 = M_PI * 2.0;

    // We'd like this to be static constexpr, but std::exp is not a constexpr.
    const std::array<ComplexType, K> coeff_;
    std::array<FloatType, N> samples_;
    std::array<ComplexType, K> result_{0,0};
    size_t index_ = 0;
    size_t prev_index_ = N - 1;

    static constexpr std::array<ComplexType, K>
    make_coefficients(const std::array<size_t, K>& frequencies)
    {
        std::array<ComplexType, K> result;
        for (size_t i = 0; i != K; ++i)
        {
            FloatType k = FloatType(frequencies[i]) / FloatType(SampleRate);
            result[i] = std::exp(-j * pi2 * k);
        }
        return result;
    }

public:
    using result_type = std::array<ComplexType, K>;

    /**
     * Construct the DFT with an array of frequencies.  These frequencies
     * should be less than @tparam SampleRate / 2 and a mulitple of
     * @tparam SampleRate / @tparam N.  No validation is performed on
     * these frequencies passed to the constructor.
     */
    NSlidingDFT(const std::array<size_t, K>& frequencies)
    : coeff_(make_coefficients(frequencies))
    {
        samples_.fill(0);
    }

    /**
     * Calculate the streaming DFT from the sample, returning an array
     * of results which correspond to the frequencies passed in to the
     * constructor.  The result is only valid after at least N samples
     * have been cycled in.
     */
    result_type operator()(FloatType sample)
    {
        auto index = index_;
        index_ += 1;
        if (index_ == N) index_ = 0;

        FloatType delta = sample - samples_[index];

        for (size_t i = 0; i != K; ++i)
        {
            result_[i] = (result_[i] + delta) * coeff_[i];
        }
        samples_[index] = sample;
        return result_;
    }
};

} // mobilinkd
