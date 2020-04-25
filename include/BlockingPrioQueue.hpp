#ifndef __BLOCKINGPRIOQUEUE_HPP__
#define __BLOCKINGPRIOQUEUE_HPP__

/*
 * Copyright (c) 2017-2020, SeungRyeol Lee
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <queue>


namespace cppbase {

template <typename T, typename U>
class BlockingPrioQueue final
{
public:
  bool empty();
  T front();
  void push(T& item);
  T dequeue();
  void pop();
  size_t size() const;

public:     // constructor and destructor
  explicit BlockingPrioQueue(U compare);
  explicit BlockingPrioQueue(size_t capacity, U compare);
  ~BlockingPrioQueue();

private:
  size_t capacity_ = 1;

  std::priority_queue<T, std::vector<T>, U> queue_;
  std::mutex mutex_;
  std::condition_variable push_avail_;
  std::condition_variable pop_avail_;
};


template <typename T, typename U>
bool BlockingPrioQueue<T, U>::empty()
{
  const std::lock_guard<std::mutex> lock(mutex_);

  return queue_.empty();
}

template <typename T, typename U>
T BlockingPrioQueue<T, U>::front()
{
  std::unique_lock<std::mutex> lock(mutex_);
  pop_avail_.wait(lock,
      [&queue_ = queue_]{ return !queue_.empty(); });

  return queue_.top();
}

template <typename T, typename U>
void BlockingPrioQueue<T, U>::push(T& item)
{
  std::unique_lock<std::mutex> lock(mutex_);
  push_avail_.wait(lock,
      [&queue_ = queue_, capacity_ = capacity_]{ return queue_.size() < capacity_; });

  queue_.push(item);
  if (queue_.size() == 1)
    pop_avail_.notify_all();
}

template <typename T, typename U>
T BlockingPrioQueue<T, U>::dequeue()
{
  std::unique_lock<std::mutex> lock(mutex_);
  pop_avail_.wait(lock,
      [&queue_ = queue_]{ return !queue_.empty(); });

  T item = queue_.top();
  queue_.pop();

  if (queue_.size() == capacity_ - 1)
    push_avail_.notify_all();

  return item;
}

template <typename T, typename U>
void BlockingPrioQueue<T, U>::pop()
{
  std::unique_lock<std::mutex> lock(mutex_);
  pop_avail_.wait(lock,
      [&queue_ = queue_]{ return !queue_.empty(); });

  queue_.pop();

  if (queue_.size() == capacity_ - 1)
    push_avail_.notify_all();
}

template <typename T, typename U>
BlockingPrioQueue<T, U>::BlockingPrioQueue(U compare)
  : BlockingPrioQueue(1, compare)
{
}

template <typename T, typename U>
BlockingPrioQueue<T, U>::BlockingPrioQueue(size_t capacity, U compare)
  : capacity_(capacity == 0 ? 1 : capacity), queue_(compare)
{
}

template <typename T, typename U>
BlockingPrioQueue<T, U>::~BlockingPrioQueue()
{
}

} // namespace cppbase

#endif
