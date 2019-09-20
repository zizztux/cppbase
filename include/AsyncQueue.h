#ifndef __ASYNCQUEUE_H__
#define __ASYNCQUEUE_H__

/*
 * Copyright (c) 2017-2018, SeungRyeol Lee
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
#include <standard.h>


template <typename T>
class AsyncQueue
{
public:
  bool empty() const;
  T front();
  bool push(T item);
  T dequeue();
  void pop();
  size_t size() const;

  AsyncQueue();
  AsyncQueue(size_t capacity);
  virtual ~AsyncQueue();

private:
  size_t _capacity;

  std::queue<T> _queue;
  std::mutex _mutex;
  std::condition_variable _cond;
};


template <typename T>
bool AsyncQueue<T>::empty() const
{
  std::unique_lock<std::mutex> lock(_mutex);

  return _queue.empty();
}

template <typename T>
T &AsyncQueue<T>::front()
{
  std::unique_lock<std::mutex> lock(_mutex);

  while (_queue.empty())
    _cond.wait(lock);

  return _queue.front();
}

template <typename T>
bool AsyncQueue<T>::push(T item)
{
  std::unique_lock<std::mutex> lock(_mutex);

  size_t size = _queue.size();
  assert(size <= _capacity);

  if (size >= _capacity)
    return false;

  _queue.push(item);
  if (size == 0)
    _cond.notify_all();

  return true;
}

template <typename T>
T AsyncQueue<T>::dequeue()
{
  std::unique_lock<std::mutex> lock(_mutex);

  while (_queue.empty())
    _cond.wait(lock);

  T item = _queue.front();
  _queue.pop();

  return item;
}

template <typename T>
void AsyncQueue<T>::pop()
{
  std::unique_lock<std::mutex> lock(_mutex);

  while (_queue.empty())
    _cond.wait(lock);

  _queue.pop();
}

template <typename T>
AsyncQueue<T>::AsyncQueue()
  : _capacity(-1)
{
}

template <typename T>
AsyncQueue<T>::AsyncQueue(size_t capacity)
  : _capacity(capacity)
{
}

template <typename T>
AsyncQueue<T>::~AsyncQueue()
{
}

// vi: set ts=2 sw=2 sts=2 expandtab:
#endif
