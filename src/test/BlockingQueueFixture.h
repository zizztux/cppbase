#ifndef __BLOCKINGQUEUE_FIXTURE_H__
#define __BLOCKINGQUEUE_FIXTURE_H__

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

#include <string>
#include <gtest/gtest.h>

#include <libcppbase/BlockingQueue.h>


constexpr size_t q_depth = 10;

struct Item
{
  int id_;
  std::string name_;
};
using Item = struct Item;

class BlockingQueueFixture : public ::testing::Test
{
protected:
  void SetUp() override {
  }

  void TearDown() override {
  }

private:
  void setupQueueInt(size_t size) {
    for (unsigned int i = 0; i < size; ++i)
      queue_int_.enqueue(i);
  }

  void setupQueueIntp(size_t size) {
    for (unsigned int i = 0; i < size; ++i) {
      int* p = new int;
      *p = i;
      queue_intp_.enqueue(p);
    }
  }

  void setupQueueItem(size_t size) {
    for (unsigned int i = 0; i < size; ++i)
      queue_item_.enqueue(Item{i, std::string("Item_") + std::to_string(i)});
  }

  void setupQueueItemp(size_t size) {
    for (unsigned int i = 0; i < size; ++i) {
      int* p = new Item{i, "Item_"};
      p->name_ += std::to_string(i);
      queue_item_.enqueue(p);
    }
  }

public:
  explicit BlockingQueueFixture()
    : queue_int_(q_depth), queue_intp_(q_depth),
      queue_item_(q_depth), queue_itemp_(q_depth)
  {
    setupQueueInt(q_depth / 2);
    setupQueueIntp(q_depth / 2);
    setupQueueItem(q_depth / 2);
    setupQueueItemp(q_depth / 2);
  }

  virtual ~BlockingQueueFixture() {
  }

protected:
  cppbase::BlockingQueue<int> queue_int_;
  cppbase::BlockingQueue<int*> queue_intp_;

  cppbase::BlockingQueue<Item> queue_item_;
  cppbase::BlockingQueue<Item*> queue_itemp_;
};

#endif
