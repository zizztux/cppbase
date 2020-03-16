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
#include <mutex>
#include <queue>


template <typename T, typename U>
class AsyncPrioQueue
{
public:
  bool enqueue(T item);
  T dequeue();
  T top();
  void pop();
  bool empty();

public:
  AsyncPrioQueue(U compare);
  AsyncPrioQueue(size_t capacity, U compare);
  virtual ~AsyncPrioQueue();

private:
  size_t _capacity;

  std::priority_queue<T, std::vector<T>, U> _queue;
  std::mutex _mutex;
  std::condition_variable _cond;
};


template <typename T, typename U>
bool AsyncPrioQueue<T, U>::enqueue(T item)
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

template <typename T, typename U>
T AsyncPrioQueue<T, U>::dequeue()
{
  std::unique_lock<std::mutex> lock(_mutex);

  while (_queue.empty())
    _cond.wait(lock);

  T item = _queue.top();
  _queue.pop();

  return item;
}

template <typename T, typename U>
T AsyncPrioQueue<T, U>::top()
{
  std::unique_lock<std::mutex> lock(_mutex);

  while (_queue.empty())
    _cond.wait(lock);

  return _queue.top();
}

template <typename T, typename U>
void AsyncPrioQueue<T, U>::pop()
{
  std::unique_lock<std::mutex> lock(_mutex);

  while (_queue.empty())
    _cond.wait(lock);

  _queue.pop();
}

template <typename T, typename U>
bool AsyncPrioQueue<T, U>::empty()
{
  std::unique_lock<std::mutex> lock(_mutex);

  return _queue.empty();
}

template <typename T, typename U>
AsyncPrioQueue<T, U>::AsyncPrioQueue(U compare)
  : _capacity(-1), _queue(compare)
{
}

template <typename T, typename U>
AsyncPrioQueue<T, U>::AsyncPrioQueue(size_t capacity, U compare)
  : _capacity(capacity), _queue(compare)
{
}

template <typename T, typename U>
AsyncPrioQueue<T, U>::~AsyncPrioQueue()
{
}

#endif
