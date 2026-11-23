#pragma once

// 固定容量的滚动时间序列（M1-15，性能图表）：新值追加在末尾，超出容量时丢弃最旧的值。
// 只依赖标准库；按从旧到新的顺序访问。

#include <algorithm>
#include <cstddef>
#include <vector>

namespace terra::core {

class TimeSeries {
public:
    explicit TimeSeries(std::size_t capacity = 600) : values_(std::max<std::size_t>(capacity, 1)) {}

    void push(double value) {
        values_[(start_ + size_) % values_.size()] = value;
        if (size_ < values_.size()) {
            ++size_;
        } else {
            start_ = (start_ + 1) % values_.size();
        }
    }
    void clear() noexcept {
        start_ = 0;
        size_ = 0;
    }

    std::size_t size() const noexcept { return size_; }
    std::size_t capacity() const noexcept { return values_.size(); }
    bool empty() const noexcept { return size_ == 0; }
    // 第 i 个值，0 为最旧；越界时返回 0。
    double at(std::size_t i) const noexcept { return i < size_ ? values_[(start_ + i) % values_.size()] : 0.0; }
    double latest() const noexcept { return size_ > 0 ? at(size_ - 1) : 0.0; }

    double minimum() const noexcept {
        double m = size_ > 0 ? at(0) : 0.0;
        for (std::size_t i = 1; i < size_; ++i) m = std::min(m, at(i));
        return m;
    }
    double maximum() const noexcept {
        double m = size_ > 0 ? at(0) : 0.0;
        for (std::size_t i = 1; i < size_; ++i) m = std::max(m, at(i));
        return m;
    }
    double average() const noexcept {
        if (size_ == 0) return 0.0;
        double sum = 0.0;
        for (std::size_t i = 0; i < size_; ++i) sum += at(i);
        return sum / static_cast<double>(size_);
    }

private:
    std::vector<double> values_;
    std::size_t start_ = 0;
    std::size_t size_ = 0;
};

}  // namespace terra::core
