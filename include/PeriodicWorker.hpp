#ifndef __PERIODICWORKER_HPP__
#define __PERIODICWORKER_HPP__

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

#include <chrono>

#include <WorkerBase.hpp>

using namespace std::chrono_literals;


namespace cppbase {

class WorkerHandler;

class PeriodicWorker : public WorkerBase
{
public:
  void setPeriod(const std::chrono::milliseconds& period) { period_ = period; }
  void setFrequency(unsigned int freq) { period_ = std::chrono::microseconds(1s) / freq; }

public:     // overridings
  void registerHandler(WorkerHandler* handler) override { handler_ = handler; }

private:
  virtual void thread_loop() override;

public:     // constructor and destructor
  PeriodicWorker() = default;
  PeriodicWorker(const std::string& name);
  PeriodicWorker(const std::chrono::milliseconds& period);
  PeriodicWorker(unsigned int freq);
  PeriodicWorker(const std::string& name, const std::chrono::milliseconds& period);
  PeriodicWorker(const std::string& name, unsigned int freq);
  virtual ~PeriodicWorker();

private:
  WorkerHandler* handler_ = nullptr;

  std::chrono::microseconds period_ = std::chrono::microseconds(1s) / 30;
};

} // namespace cppbase

#endif
