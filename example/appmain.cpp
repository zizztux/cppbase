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

#include <iostream>

#include <GenericWorker.hpp>
#include <PeriodicWorker.hpp>


class PreProcessor : public GenericWorker
{
private:
  virtual bool processJob(void* job) { std::cout << "Job: " << job << std::endl; return job != 0; }

public:
  PreProcessor() = default;
  virtual ~PreProcessor() { }
};

class PeriodicCall : public PeriodicWorker
{
private:
  virtual bool processJob() { std::cout << __FUNCTION__ << std::endl; return true; }

public:
  PeriodicCall() = default;
  PeriodicCall(unsigned int freq) : PeriodicWorker(freq) { }
  virtual ~PeriodicCall() { }
};


int
main(int argc, char *argv[])
{
  PreProcessor worker;
  worker.start();

  worker.scheduleJob((void*)5);
  worker.scheduleJob((void*)2);
  worker.scheduleJob((void*)9);
  worker.scheduleJob((void*)0);

  worker.finalize();

  PeriodicCall periodic;
  periodic.start();
  periodic.finalize();

  return 0;
}
