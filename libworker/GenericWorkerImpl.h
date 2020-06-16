#ifndef __LIBWORKER_GENERICWORKERIMPL_H__
#define __LIBWORKER_GENERICWORKERIMPL_H__

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

#include <memory>
#include <vector>

#include <worker/GenericWorker.h>


namespace cppbase {

template <typename T>
class BlockingQueue;
class JobBase;
class JobChannel;

class GenericWorkerImpl : public GenericWorker
{
public:
  virtual bool start() override;
  virtual void stop() override;

  virtual void registerHandler(WorkerHandler* handler) override final;

  virtual const std::string& name() const override final;
  virtual void setName(std::string_view name) override final;

  virtual JobChannel* newChannel(std::size_t depth = 1) override;
  virtual JobChannel* channel(unsigned long channel) const override;
  virtual std::size_t nchannels() const override;


  JobChannel* newChannel(size_t q_depth = 1);
  JobChannel* getChannel(unsigned int channel);
  size_t nchannels() const { return channels_.size(); }

private:    // overridings
  virtual void thread_loop() override;

public:     // constructor and destructor
  explicit GenericWorker();
  explicit GenericWorker(const std::string& name);
  explicit GenericWorker(size_t q_depth);
  explicit GenericWorker(const std::string& name, size_t q_depth);
  virtual ~GenericWorker();

private:
  std::vector<BlockingQueue<std::shared_ptr<JobBase>>*> queues_;
  std::vector<JobChannel*> channels_;
};

} // namespace cppbase

#endif