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

#include <cassert>

#include <BlockingQueue.hpp>
#include <GenericWorker.hpp>
#include <JobBase.hpp>
#include <JobChannel.hpp>
#include <WorkerHandler.hpp>


namespace cppbase {

JobChannel*
GenericWorker::newChannel(size_t q_depth)
{
  BlockingQueue<std::shared_ptr<JobBase>>* queue = new BlockingQueue<std::shared_ptr<JobBase>>(q_depth);
  if (!queue)
    return nullptr;

  JobChannel* channel = new JobChannel(queue);
  if (!channel) {
    delete queue;
    return nullptr;
  }

  queues_.push_back(queue);
  channels_.push_back(channel);

  return channel;
}

JobChannel*
GenericWorker::getChannel(unsigned int channel)
{
  if (channel >= nchannels())
    return nullptr;

  return channels_[channel];
}

void
GenericWorker::thread_loop()
{
  assert(handler_);

  std::shared_ptr<JobBase> first = nullptr;

  do {
    std::shared_ptr<JobBase> prev = nullptr;
    std::shared_ptr<JobBase> next = nullptr;

    for (BlockingQueue<std::shared_ptr<JobBase>>* queue : queues_) {
      next = queue->dequeue();
      if (prev) prev->next_ = next;
      else first = next;
      prev = next;
    }
  } while (handler_->onWorkerHandle(first));
}


GenericWorker::GenericWorker()
  : GenericWorker(1)
{
}

GenericWorker::GenericWorker(const std::string& name)
  : GenericWorker(name, 1)
{
}

GenericWorker::GenericWorker(size_t q_depth)
{
  newChannel(q_depth);
}

GenericWorker::GenericWorker(const std::string& name, size_t q_depth)
  : WorkerBase(name)
{
  newChannel(q_depth);
}

GenericWorker::~GenericWorker()
{
  for (JobChannel* channel : channels_)
    delete channel;

  for (BlockingQueue<std::shared_ptr<JobBase>>* queue : queues_)
    delete queue;
}

} // namespace cppbase
