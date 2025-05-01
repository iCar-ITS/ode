#pragma once

#include <mutex>

template <typename T>
class DuoBuffer 
{
private:
    T data_[2];
    size_t idx_;
    std::mutex mutex_;
public:
    DuoBuffer(): idx_(0) {}
    DuoBuffer(const DuoBuffer& other) = delete;
    DuoBuffer(DuoBuffer&& other) = default;

    void set(const T& setter)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        data_[idx_] = setter;
    }

    void claim() 
    {
        std::lock_guard<std::mutex> lock(mutex_);
        idx_ = !idx_;
    }

    T& get()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return data_[!idx_];
    }
};