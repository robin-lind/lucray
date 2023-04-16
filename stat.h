// MIT License
//
// Copyright (c) 2023 Robin Lind
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef STAT_H
#define STAT_H

#include <cmath>

//based on: https://www.johndcook.com/blog/standard_deviation/
template<typename T>
struct number_stat {
    int count{};
    T m{}, s{};

    void push(const T& x)
    {
        count++;
        if (count == 1) {
            m = x;
        }
        else {
            const auto old_m = m;
            m += (x - m) / count;
            s += (x - old_m) * (x - m);
        }
    }

    T mean() const
    {
        return count > 0 ? m : T();
    }

    T variance() const
    {
        return count > 1 ? s / (count - 1) : T();
    }

    T stdev() const
    {
        return std::sqrt(variance());
    }

    T quality() const
    {
        return -std::log2(variance());
    }
};
#endif
