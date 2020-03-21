#ifndef __BLOCKINGQUEUE_HPP__
#define __BLOCKINGQUEUE_HPP__

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
#include <mutex>
#include <queue>


namespace cppbase {

template <typename T>
class BlockingQueue
{
public:
  bool empty();
  T front();
  void push(T& item);
  T dequeue();
  void pop();
  size_t size() const;

public:     // constructor and destructor
  BlockingQueue() = default;
  BlockingQueue(size_t capacity);
  virtual ~BlockingQueue();

private:
  size_t capacity_ = 1;

  std::queue<T> queue_;
  std::mutex mutex_;
  std::condition_variable push_avail_;
  std::condition_variable pop_avail_;
};


template <typename T>
bool BlockingQueue<T>::empty()
{
  const std::lock_guard<std::mutex> lock(mutex_);

  return queue_.empty();
}

template <typename T>
T BlockingQueue<T>::front()
{
  std::unique_lock<std::mutex> lock(mutex_);

  while (queue_.empty())
    pop_avail_.wait(lock);

  return queue_.front();
}

template <typename T>
void BlockingQueue<T>::push(T& item)
{
  std::unique_lock<std::mutex> lock(mutex_);

  size_t size;
  while ((size = queue_.size()) == capacity_)
    push_avail_.wait(lock);

  queue_.push(item);
  if (size == 0)
    pop_avail_.notify_all();
}

template <typename T>
T BlockingQueue<T>::dequeue()
{
  std::unique_lock<std::mutex> lock(mutex_);

  while (queue_.empty())
    pop_avail_.wait(lock);

  T item = queue_.front();
  queue_.pop();

  if (queue_.size() == capacity_ - 1)
    push_avail_.notify_all();

  return item;
}

template <typename T>
void BlockingQueue<T>::pop()
{
  std::unique_lock<std::mutex> lock(mutex_);

  while (queue_.empty())
    pop_avail_.wait(lock);

  queue_.pop();

  if (queue_.size() == capacity_ - 1)
    push_avail_.notify_all();
}

template <typename T>
BlockingQueue<T>::BlockingQueue(size_t capacity)
  : capacity_(capacity == 0 ? 1 : capacity)
{
}

template <typename T>
BlockingQueue<T>::~BlockingQueue()
{
}

} // namespace cppbase

#endif
